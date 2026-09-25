package triangulate

import (
	"math"

	"delaunaysvc/internal/geom"
)

// ChangeSet is the net local difference a single incremental edit made
// to the mesh. Only triangles that actually disappeared or appeared are
// listed; triangles outside the cavity are untouched and never appear.
// Every triangle is given CCW in the mesh's stable point-index space.
// Applying Removed then Added to the old triangle set yields exactly the
// new triangle set (see Diff).
type ChangeSet struct {
	Removed []geom.Triangle `json:"removed"`
	Added   []geom.Triangle `json:"added"`
}

// empty reports whether the edit changed no triangle.
func (c ChangeSet) empty() bool { return len(c.Removed) == 0 && len(c.Added) == 0 }

// triKey is a triangle's canonical identity: its three stable point
// indices sorted ascending. A moved ghost-free triangle compares by
// vertex set regardless of the (always CCW) winding it was created with.
type triKey struct{ a, b, c int }

func keyOf(t geom.Triangle) triKey {
	s := [3]int{t.A, t.B, t.C}
	for i := 1; i < 3; i++ {
		for j := i; j > 0 && s[j] < s[j-1]; j-- {
			s[j], s[j-1] = s[j-1], s[j]
		}
	}
	return triKey{s[0], s[1], s[2]}
}

// realTriangles snapshots the ghost-free living triangles as CCW
// geom.Triangles in stable-index order.
func (m *Mesh) realTriangles() []geom.Triangle {
	out := make([]geom.Triangle, 0, 2*len(m.Points))
	for _, t := range m.store {
		if !t.alive || isGhost(t.a) || isGhost(t.b) || isGhost(t.c) {
			continue
		}
		out = append(out, geom.Triangle{A: t.a, B: t.b, C: t.c})
	}
	canonicalize(out)
	return out
}

// diff returns the net set difference before→after over canonical
// triangle identities. Triangles present in both are unchanged and
// omitted even if they were killed and re-created with a new store id
// during the edit — the wire identity is the vertex triple.
func diff(before, after []geom.Triangle) ChangeSet {
	had := make(map[triKey]geom.Triangle, len(before))
	for _, t := range before {
		had[keyOf(t)] = t
	}
	have := make(map[triKey]geom.Triangle, len(after))
	for _, t := range after {
		have[keyOf(t)] = t
	}
	cs := ChangeSet{Removed: []geom.Triangle{}, Added: []geom.Triangle{}}
	for k, t := range had {
		if _, ok := have[k]; !ok {
			cs.Removed = append(cs.Removed, t)
		}
	}
	for k, t := range have {
		if _, ok := had[k]; !ok {
			cs.Added = append(cs.Added, t)
		}
	}
	canonicalize(cs.Removed)
	canonicalize(cs.Added)
	return cs
}

// ---- insert -------------------------------------------------------------

// Insert appends one point and locally re-fans the containing cavity.
// It returns the new point's stable id and the exact triangle change.
// The caller (session layer) is responsible for input legality; the
// kernel assumes p is finite and not coincident with a live point.
func (m *Mesh) Insert(p geom.Point) (int, ChangeSet, *geom.Error) {
	before := m.realTriangles()

	id := len(m.Points)
	local := geom.Point{X: p.X - m.Origin.X, Y: p.Y - m.Origin.Y}
	wp := jitterPoint(local, id, m.h)

	m.Points = append(m.Points, p)
	m.work = append(m.work, wp)
	m.growBounds(wp)

	// A point outside the current super-triangle would defeat the
	// Bowyer–Watson seed; widen the virtual frame locally first.
	if gerr := m.ensureContains(wp); gerr != nil {
		if rerr := m.rebuildRobust(m.Points, m.Origin); rerr != nil {
			return 0, ChangeSet{}, rerr
		}
		return id, diff(before, m.realTriangles()), nil
	}
	if gerr := m.insertVertex(id); gerr != nil {
		if rerr := m.rebuildRobust(m.Points, m.Origin); rerr != nil {
			return 0, ChangeSet{}, rerr
		}
		return id, diff(before, m.realTriangles()), nil
	}
	// The local Bowyer–Watson fan is Delaunay in general position. At a
	// nearly collinear hull sliver it can leave a real wedge unfilled;
	// detect that from the mesh's own invariants, close the wedge locally,
	// and finally fall back to an order-robust whole rebuild if needed.
	if !m.locallyConsistent() {
		if !m.repairSliver(id) || !m.locallyConsistent() {
			if rerr := m.rebuildRobust(m.Points, m.Origin); rerr != nil {
				return 0, ChangeSet{}, rerr
			}
		}
	}
	return id, diff(before, m.realTriangles()), nil
}

// repairSliver closes a ghost-sliver fan gap around the freshly inserted
// point and restores local Delaunay by edge flips. It returns false if it
// could not make the star consistent.
func (m *Mesh) repairSliver(p int) bool {
	filled := m.closeStarGap(p)
	if filled == nil {
		return false
	}
	if gerr := m.lawson(filled); gerr != nil {
		return false
	}
	return true
}

// ---- move ---------------------------------------------------------------

// Move relocates live point id to p and restores the mesh locally. It
// is, as presented to the caller, one atomic edit implemented with the
// same cavity machinery as insertion:
//
//  1. gather the point's full star (its incident triangles, ghost
//     triangles included) and its link ring — the ordered chain of
//     neighbours opposite the point;
//  2. collapse the star: kill the incident triangles, leaving one
//     star-shaped hole bounded by the link ring (closed for a hull
//     vertex across the virtual frame);
//  3. re-triangulate the hole without the point by ear clipping, then
//     drive the local fill to Delaunay with edge flips (Lawson), using
//     the same exact predicates;
//  4. move the coordinate and run insertVertex — the identical
//     dig-hole/re-fan step an ordinary Insert runs — to stitch the point
//     back in at its new position.
//
// The caller validates legality first, so on success the mesh never
// passes through an externally visible non-Delaunay state.
func (m *Mesh) Move(id int, p geom.Point) (ChangeSet, *geom.Error) {
	before := m.realTriangles()

	local := geom.Point{X: p.X - m.Origin.X, Y: p.Y - m.Origin.Y}
	wp := jitterPoint(local, id, m.h)

	// 1. star + ordered link ring.
	ring, star, gerr := m.starRing(id)
	if gerr != nil {
		return m.fallbackMove(id, p, before)
	}

	// 2. collapse the star.
	for _, tid := range star {
		if m.store[tid].alive {
			m.killTri(tid)
		}
	}

	// 3. fill the hole (without the point) and restore local Delaunay.
	fill, gerr := m.earClipFill(ring)
	if gerr != nil {
		return m.fallbackMove(id, p, before)
	}
	if gerr := m.lawson(fill); gerr != nil {
		return m.fallbackMove(id, p, before)
	}

	// 4. relocate and stitch back in with the ordinary insertion step.
	m.Points[id] = p
	m.work[id] = wp
	m.minX, m.minY = math.Min(m.minX, wp.X), math.Min(m.minY, wp.Y)
	m.maxX, m.maxY = math.Max(m.maxX, wp.X), math.Max(m.maxY, wp.Y)
	if gerr := m.ensureContains(wp); gerr != nil {
		return m.fallbackMove(id, p, before)
	}
	if gerr := m.insertVertex(id); gerr != nil {
		return m.fallbackMove(id, p, before)
	}
	if !m.locallyConsistent() {
		if !m.repairSliver(id) || !m.locallyConsistent() {
			return m.fallbackMove(id, p, before)
		}
	}
	return diff(before, m.realTriangles()), nil
}

// fallbackMove rebuilds the whole mesh over the point set with id moved
// to p and returns the resulting net change. It is the safety net for a
// local edit that hit a numerical or topological snag; jitter is a
// function of the stable id, so the rebuild is deterministic and the
// session is left holding a valid Delaunay net regardless.
func (m *Mesh) fallbackMove(id int, p geom.Point, before []geom.Triangle) (ChangeSet, *geom.Error) {
	// The partial local topology is discarded wholesale by the rebuild;
	// Points[id] is set to the target so the rebuild triangulates exactly
	// the post-edit set.
	m.Points[id] = p
	if gerr := m.rebuildRobust(m.Points, m.Origin); gerr != nil {
		return ChangeSet{}, gerr
	}
	return diff(before, m.realTriangles()), nil
}

// starRing returns the link ring of vertex v in counter-clockwise
// neighbour order together with the ids of every living triangle
// incident to v. The ring is closed for interior vertices by the chain
// of real neighbours, and for hull vertices it passes through the
// virtual super-triangle vertices; either way it is a simple CCW
// polygon around v in the work frame.
func (m *Mesh) starRing(v int) ([]int, []int, *geom.Error) {
	star := append([]int(nil), m.vertTris[v]...)
	if len(star) < 1 {
		return nil, nil, &geom.Error{
			Code: geom.ErrDegenerate, Message: "vertex has an empty star",
		}
	}

	// next[u] = w means triangle (v, u, w) is in the star and is wound so
	// that u->w runs CCW as seen standing at v. Each incident triangle
	// contributes exactly one such directed link edge; chaining them is
	// the ring walk.
	next := make(map[int]int, len(star)+1)
	for _, tid := range star {
		t := m.store[tid]
		u, w := t.linkPair(v)
		if _, dup := next[u]; dup {
			return nil, nil, &geom.Error{
				Code: geom.ErrDegenerate, Message: "non-manifold star around vertex",
			}
		}
		next[u] = w
	}
	start, _ := m.store[star[0]].linkPair(v)
	ring := make([]int, 0, len(next))
	cur := start
	for {
		ring = append(ring, cur)
		nxt, ok := next[cur]
		if !ok {
			return nil, nil, &geom.Error{
				Code: geom.ErrDegenerate, Message: "vertex link ring is not closed",
			}
		}
		cur = nxt
		if cur == start {
			break
		}
		if len(ring) > len(next) {
			return nil, nil, &geom.Error{
				Code: geom.ErrDegenerate, Message: "vertex link ring walk did not close",
			}
		}
	}
	return ring, star, nil
}

// linkPair returns the triangle's two vertices other than v, ordered so
// that (v, u, w) is counter-clockwise — i.e. u->w is the CCW link edge
// as seen from v.
func (t workTri) linkPair(v int) (u, w int) {
	switch v {
	case t.a:
		return t.b, t.c
	case t.b:
		return t.c, t.a
	default:
		return t.a, t.b
	}
}

// earClipFill triangulates the simple CCW polygon ring (which may
// contain ghost vertices) and returns the ids of the triangles created,
// flagged in the fill set. Standard ear clipping with the kernel's
// exact predicates: a candidate ear is convex and contains no other ring
// vertex in its interior.
func (m *Mesh) earClipFill(ring []int) (map[int]bool, *geom.Error) {
	k := len(ring)
	if k < 3 {
		return nil, &geom.Error{
			Code: geom.ErrDegenerate, Message: "move cavity ring has fewer than 3 vertices",
		}
	}
	alive := make(map[int]int, k) // ring vertex -> its successor index
	order := append([]int(nil), ring...)
	for i := range order {
		alive[order[i]] = order[(i+1)%k]
	}
	cur := order[0]
	fill := make(map[int]bool, 2*k)
	created := 0

	for len(alive) >= 3 {
		nxt := alive[cur]
		prv := cur
		for alive[prv] != cur {
			prv = alive[prv]
		}
		a, b, c := prv, cur, nxt
		pa, pb, pc := m.at(a), m.at(b), m.at(c)

		clipped := false
		if geom.OrientSign(pa, pb, pc) > 0 { // convex (strictly CCW) ear
			empty := true
			for q := range alive {
				if q == a || q == b || q == c {
					continue
				}
				if pointInTriangleCCW(m.at(q), pa, pb, pc) {
					empty = false
					break
				}
			}
			if empty {
				// Orient strictly CCW defensively; the ear is already convex.
				t := workTri{a: a, b: b, c: c}
				if geom.OrientSign(m.at(t.a), m.at(t.b), m.at(t.c)) < 0 {
					t.b, t.c = t.c, t.b
				}
				id := m.addTri(t)
				fill[id] = true
				created++
				alive[a] = c
				delete(alive, b)
				clipped = true
			}
		}

		if len(alive) < 3 {
			break
		}
		if clipped {
			cur = c
			continue
		}
		// Reflex ear or non-empty: advance to the next live vertex.
		cur = nxt
		created++
		// Every simple polygon has an ear; if the walk has lapped the
		// polygon many times without clipping, topology was broken.
		if created > 64*k*k+64 {
			return nil, &geom.Error{
				Code: geom.ErrDegenerate, Message: "ear clipping failed to find an ear",
			}
		}
	}
	return fill, nil
}

// pointInTriangleCCW reports q strictly inside a positively oriented
// triangle (a,b,c). Boundary points report false; ear emptiness cares
// about strict containment.
func pointInTriangleCCW(q, a, b, c geom.Point) bool {
	return geom.OrientSign(a, b, q) > 0 &&
		geom.OrientSign(b, c, q) > 0 &&
		geom.OrientSign(c, a, q) > 0
}

// lawson flips interior edges of the freshly ear-clipped fill until the
// whole fill is locally Delaunay. Only edges shared by two fill
// triangles are flipped: boundary edges of the hole are edges of the
// surviving Delaunay mesh (or of the virtual frame) and stay fixed.
// Flips strictly improve the (finite) global edge-flipping potential,
// so the stack walk terminates at the Delaunay triangulation of the
// hole consistent with the untouched outside.
func (m *Mesh) lawson(fill map[int]bool) *geom.Error {
	// Seed with every edge shared by two fill triangles.
	type edgePair struct {
		e      edge
		t1, t2 int
	}
	var stack []edgePair
	seen := make(map[edge]bool)
	for id := range fill {
		t := m.store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			if seen[e] {
				continue
			}
			ns := m.edgeAdj[e]
			if len(ns) != 2 {
				continue
			}
			if fill[ns[0]] && fill[ns[1]] {
				seen[e] = true
				stack = append(stack, edgePair{e: e, t1: ns[0], t2: ns[1]})
			}
		}
	}

	iter, maxIter := 0, 4096+64*len(fill)*len(fill)
	for len(stack) > 0 {
		item := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		iter++
		if iter > maxIter {
			return &geom.Error{
				Code: geom.ErrDegenerate, Message: "Lawson flips did not converge",
			}
		}
		t1, t2 := m.store[item.t1], m.store[item.t2]
		if !t1.alive || !t2.alive {
			continue // edge was already flipped away
		}
		u, v := item.e.u, item.e.v
		if !fill[item.t1] || !fill[item.t2] {
			continue
		}
		// The two triangles must still share exactly this edge.
		if !t1.hasEdge(u, v) || !t2.hasEdge(u, v) {
			continue
		}
		c := thirdVertex(t1, u, v)
		d := thirdVertex(t2, u, v)

		// Convex quadrilateral only: c and d must lie on opposite sides
		// of line uv. A reflex pair means uv is a forced diagonal of the
		// simple polygon and may never be flipped.
		s1 := geom.OrientSign(m.at(u), m.at(v), m.at(c))
		s2 := geom.OrientSign(m.at(u), m.at(v), m.at(d))
		if s1 == 0 || s2 == 0 || s1 == s2 {
			continue
		}
		// Local Delaunay: d outside (or, degenerate, on) t1's circle.
		if geom.InCircleSign(m.at(t1.a), m.at(t1.b), m.at(t1.c), m.at(d)) <= 0 {
			continue
		}
		// Flip uv -> cd. With c left of directed u->v and d right, the
		// CCW successors are (c, u, d) and (d, v, c); orient explicitly
		// rather than relying on the convention.
		nt1 := workTri{a: c, b: u, c: d}
		if geom.OrientSign(m.at(nt1.a), m.at(nt1.b), m.at(nt1.c)) < 0 {
			nt1.b, nt1.c = nt1.c, nt1.b
		}
		nt2 := workTri{a: d, b: v, c: c}
		if geom.OrientSign(m.at(nt2.a), m.at(nt2.b), m.at(nt2.c)) < 0 {
			nt2.b, nt2.c = nt2.c, nt2.b
		}
		m.killTri(item.t1)
		m.killTri(item.t2)
		id1 := m.addTri(nt1)
		id2 := m.addTri(nt2)
		fill[id1] = true
		fill[id2] = true

		// Re-test the four outer edges of the quad.
		for _, e := range [3]edge{newEdge(nt1.a, nt1.b), newEdge(nt1.b, nt1.c), newEdge(nt1.c, nt1.a)} {
			ns := m.edgeAdj[e]
			if len(ns) == 2 && fill[ns[0]] && fill[ns[1]] {
				stack = append(stack, edgePair{e: e, t1: ns[0], t2: ns[1]})
			}
		}
		for _, e := range [3]edge{newEdge(nt2.a, nt2.b), newEdge(nt2.b, nt2.c), newEdge(nt2.c, nt2.a)} {
			ns := m.edgeAdj[e]
			if len(ns) == 2 && fill[ns[0]] && fill[ns[1]] {
				stack = append(stack, edgePair{e: e, t1: ns[0], t2: ns[1]})
			}
		}
	}
	return nil
}

// ---- super-triangle maintenance -----------------------------------------

// growBounds enlarges the recorded real-point bounding box to contain q.
func (m *Mesh) growBounds(q geom.Point) {
	m.minX = math.Min(m.minX, q.X)
	m.minY = math.Min(m.minY, q.Y)
	m.maxX = math.Max(m.maxX, q.X)
	m.maxY = math.Max(m.maxY, q.Y)
}

// ensureContains widens the virtual super-triangle so that q sits
// strictly inside it, whenever an edit's coordinate escaped the current
// frame. Existing real points are all inside by construction, so the
// same widened frame covers them too. Only the virtual geometry moves;
// the ghost triangles' vertex triples and orientation are preserved, so
// no real topology changes and the cost is proportional to the
// (small, constant) number of ghost triangles.
func (m *Mesh) ensureContains(q geom.Point) *geom.Error {
	if m.pointInSuper(q) {
		return nil
	}
	minX, minY := math.Min(m.minX, q.X), math.Min(m.minY, q.Y)
	maxX, maxY := math.Max(m.maxX, q.X), math.Max(m.maxY, q.Y)
	s0, s1, s2 := superOfBounds(minX, minY, maxX, maxY)

	// Move the corners, then re-file every ghost triangle. Verify each
	// is still strictly CCW: the 100x margin makes a reversal impossible
	// for any sane geometry, and if it ever happens the caller rebuilds.
	old := [3]geom.Point{m.s0, m.s1, m.s2}
	m.s0, m.s1, m.s2 = s0, s1, s2
	for _, tid := range m.ghostTriangleIDs() {
		t := m.store[tid]
		if geom.OrientSign(m.at(t.a), m.at(t.b), m.at(t.c)) <= 0 {
			m.s0, m.s1, m.s2 = old[0], old[1], old[2]
			return &geom.Error{
				Code: geom.ErrDegenerate, Message: "super-triangle expansion reversed a ghost triangle",
			}
		}
		m.idx.removeIn(tid, t)
		nt := m.idx.insertTriangle(tid, t, m)
		m.store[tid] = nt
	}
	return nil
}

// pointInSuper reports q strictly inside the CCW virtual triangle.
func (m *Mesh) pointInSuper(q geom.Point) bool {
	return geom.OrientSign(m.s0, m.s1, q) > 0 &&
		geom.OrientSign(m.s1, m.s2, q) > 0 &&
		geom.OrientSign(m.s2, m.s0, q) > 0
}

// ghostTriangleIDs returns the living triangles touching a ghost.
func (m *Mesh) ghostTriangleIDs() []int {
	out := make([]int, 0, len(m.vertTris[ghost0])+len(m.vertTris[ghost1])+len(m.vertTris[ghost2]))
	seen := make(map[int]bool)
	for _, v := range [3]int{ghost0, ghost1, ghost2} {
		for _, tid := range m.vertTris[v] {
			if !seen[tid] && m.store[tid].alive {
				seen[tid] = true
				out = append(out, tid)
			}
		}
	}
	return out
}

// hasEdge reports whether t is incident to undirected edge (u,v).
func (t workTri) hasEdge(u, v int) bool {
	for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
		if e == (edge{u, v}) {
			return true
		}
	}
	return false
}
