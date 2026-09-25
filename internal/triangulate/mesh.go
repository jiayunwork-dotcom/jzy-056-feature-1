// Package triangulate: mesh.go carries the stateful Delaunay mesh used
// by the incremental session layer.
//
// Mesh is the same Bowyer–Watson machinery that the one-shot Build runs
// in a loop, but kept alive between calls: an append-only triangle
// store with stable ids, incremental edge adjacency, the uniform-grid
// spatial index and the three virtual super-triangle vertices are all
// long-lived, so an insertion or a move only ever touches the local
// cavity ("dig a hole, re-fan it") instead of rebuilding the net.
//
// Point identity is the dense index in Points (0..n-1), stable for the
// whole life of the session. The super-triangle vertices are *not*
// stored as slice entries (so a growing point set never renumbers them);
// they live at three fixed sentinel ids beyond every real index.
package triangulate

import (
	"math"

	"delaunaysvc/internal/geom"
)

func mathAtan2(y, x float64) float64 { return math.Atan2(y, x) }

// Ghost vertex ids: far beyond every real point index. They are never
// reused and never renumbered, so edge keys involving a ghost stay
// stable while real points are appended and moved.
const (
	ghost0 = 1 << 40
	ghost1 = 1<<40 + 1
	ghost2 = 1<<40 + 2
)

// isGhost reports whether id refers to a virtual super-triangle vertex.
func isGhost(id int) bool { return id >= ghost0 }

// Mesh is a long-lived, in-memory Delaunay triangulation. Its own
// methods never reject an operation on input grounds — that is the
// session layer's job; the kernel assumes the point set it is handed is
// legal and mutates topology locally.
type Mesh struct {
	// Points are the caller's original coordinates (the wire frame), in
	// dense stable-index order.
	Points []geom.Point
	// work are the coordinates the predicates actually run on: Points
	// translated by -Origin and symbolically perturbed into general
	// position. work[i] is in bijection with Points[i].
	work []geom.Point
	// Origin is always Points[0] at construction (kept for parity with
	// the one-shot Result/Voronoi path).
	Origin geom.Point
	// h is the general-position perturbation magnitude (generalPositionScale*span).
	h float64

	// Bounding box of the real points in the work frame.
	minX, minY, maxX, maxY float64

	// Super-triangle (work frame) corner coordinates and its centre.
	s0, s1, s2 geom.Point

	store   []workTri
	idx     *spatialIndex
	edgeAdj map[edge][]int
	// vertTris maps every vertex (real ids and ghosts) to the living
	// triangles incident to it. Move needs the complete star of a point
	// including ghost triangles.
	vertTris map[int][]int
	// jitterSalt selects the deterministic general-position perturbation
	// scheme. Zero is the canonical scheme used by the normal path; the
	// robust fallback tries other salts to step around a sliver that the
	// canonical perturbation lands on.
	jitterSalt uint64
}

// NewMesh builds the stateful mesh over an already validated point set.
// It is the stateful twin of Build: identical construction, but the
// resulting mesh survives for further incremental edits.
func NewMesh(points []geom.Point) (*Mesh, *geom.Error) {
	m := &Mesh{}
	if gerr := m.reset(points, points[0]); gerr != nil {
		return nil, gerr
	}
	return m, nil
}

// reset rebuilds the whole mesh in natural order 0..n-1.
func (m *Mesh) reset(points []geom.Point, origin geom.Point) *geom.Error {
	return m.resetOrder(points, origin, nil)
}

// resetOrder constructs a complete mesh over points, inserting them in
// the given order (nil means 0..n-1). The symbolic perturbation is
// always bound to the stable point id, never to the insertion position,
// so reordering changes only the Bowyer–Watson cavity path, not the
// coordinates or the resulting triangle ids. The fallback rebuild uses
// alternate orders to step around an order-dependent sliver degeneracy.
func (m *Mesh) resetOrder(points []geom.Point, origin geom.Point, order []int) *geom.Error {
	n := len(points)
	local := make([]geom.Point, n)
	for i, p := range points {
		local[i] = geom.Point{X: p.X - origin.X, Y: p.Y - origin.Y}
	}

	minX, minY := local[0].X, local[0].Y
	maxX, maxY := minX, minY
	for _, p := range local[1:] {
		minX = mathMin(minX, p.X)
		maxX = mathMax(maxX, p.X)
		minY = mathMin(minY, p.Y)
		maxY = mathMax(maxY, p.Y)
	}
	span := mathMax(maxX-minX, maxY-minY)
	if span == 0 {
		span = 1
	}
	h := generalPositionScale * span
	for i := range local {
		local[i] = jitterPointSalt(local[i], i, h, m.jitterSalt)
	}

	// Jitter may move the bounding box by O(h); re-measure on the
	// perturbed coordinates so the spatial index and the expansion test
	// use exactly the points predicates see.
	minX, minY = local[0].X, local[0].Y
	maxX, maxY = minX, minY
	for _, p := range local[1:] {
		minX = mathMin(minX, p.X)
		maxX = mathMax(maxX, p.X)
		minY = mathMin(minY, p.Y)
		maxY = mathMax(maxY, p.Y)
	}

	m.Points = make([]geom.Point, n)
	copy(m.Points, points)
	m.work = local
	m.Origin = origin
	m.h = h
	m.minX, m.minY, m.maxX, m.maxY = minX, minY, maxX, maxY

	s0, s1, s2 := superOfBounds(minX, minY, maxX, maxY)
	m.s0, m.s1, m.s2 = s0, s1, s2

	m.idx = newSpatialIndex(minX, minY, maxX-minX, maxY-minY, n)
	m.store = make([]workTri, 0, 2*n+4)
	m.edgeAdj = make(map[edge][]int, 6*n+4)
	m.vertTris = make(map[int][]int, n+8)

	m.addTri(workTri{a: ghost0, b: ghost1, c: ghost2, alive: true})

	if order == nil {
		order = make([]int, n)
		for i := range order {
			order[i] = i
		}
	}
	// Insert in the requested order through the exact same local cavity
	// routine the one-shot path uses (and that later session edits reuse),
	// closing a sliver gap whenever the local fan leaves one.
	for _, p := range order {
		if gerr := m.insertVertex(p); gerr != nil {
			return gerr
		}
		if filled := m.closeStarGap(p); filled != nil {
			if gerr := m.lawson(filled); gerr != nil {
				return gerr
			}
		}
	}
	return nil
}

// at returns the work-frame coordinate of any vertex id, including a
// ghost sentinel.
func (m *Mesh) at(id int) geom.Point {
	switch id {
	case ghost0:
		return m.s0
	case ghost1:
		return m.s1
	case ghost2:
		return m.s2
	default:
		return m.work[id]
	}
}

// PointAt returns the caller-frame coordinate of a live point id.
func (m *Mesh) PointAt(id int) (geom.Point, bool) {
	if id < 0 || id >= len(m.Points) {
		return geom.Point{}, false
	}
	return m.Points[id], true
}

// PointCount returns the number of live real points.
func (m *Mesh) PointCount() int { return len(m.Points) }

// ---- incremental topology bookkeeping ----------------------------------

// addTri files a living triangle with the store (growing it by one
// slot), the edge adjacency, the per-vertex stars and the spatial index.
// All four structures must be updated together; nothing else writes them.
func (m *Mesh) addTri(t workTri) int {
	id := len(m.store)
	m.store = append(m.store, workTri{})
	t.alive = true
	t = m.idx.insertTriangle(id, t, m)
	m.store[id] = t
	for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
		m.edgeAdj[e] = append(m.edgeAdj[e], id)
	}
	m.vertTris[t.a] = append(m.vertTris[t.a], id)
	m.vertTris[t.b] = append(m.vertTris[t.b], id)
	m.vertTris[t.c] = append(m.vertTris[t.c], id)
	return id
}

// killTri removes living triangle id from adjacency, per-vertex stars
// and the spatial index, and marks its store slot dead. Dead slots are
// never reused, keeping every id reference unambiguous. Removal always
// allocates a fresh slice rather than rewriting in place: the fan and
// cavity code may still be iterating another view of these maps in the
// same edit, and in-place [:0] filtering would alias the backing array
// under that iteration.
func (m *Mesh) killTri(id int) {
	t := m.store[id]
	for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
		old := m.edgeAdj[e]
		ns := make([]int, 0, len(old))
		for _, x := range old {
			if x != id {
				ns = append(ns, x)
			}
		}
		if len(ns) == 0 {
			delete(m.edgeAdj, e)
		} else {
			m.edgeAdj[e] = ns
		}
	}
	for _, v := range [3]int{t.a, t.b, t.c} {
		old := m.vertTris[v]
		ns := make([]int, 0, len(old))
		for _, x := range old {
			if x != id {
				ns = append(ns, x)
			}
		}
		m.vertTris[v] = ns
	}
	m.idx.removeIn(id, t)
	t.alive = false
	m.store[id] = t
}

// insertVertex runs one Bowyer–Watson step: collect the cavity of
// triangles whose circumcircle contains p, delete it and fan p over the
// cavity boundary. This is the exact local "dig a hole, re-connect"
// primitive that the one-shot Build loops over, extracted verbatim; the
// session's Insert and Move both run it rather than re-deriving it.
func (m *Mesh) insertVertex(p int) *geom.Error {
	bad, gerr := m.collectCavity(p)
	if gerr != nil {
		return gerr
	}

	// Boundary edges of the cavity. This runs while every cavity triangle
	// is alive and before any slot is killed, so each adjacency list holds
	// only living owners: length one means a frame boundary, length two
	// means boundary only when the other owner survives. The seed/fan
	// repair in collectCavity keeps this loop star-shaped for both
	// interior and hull insertions.
	var boundary []edge
	boundarySet := make(map[edge]bool, 8)
	for id := range bad {
		t := m.store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			if boundarySet[e] {
				continue
			}
			ns := m.edgeAdj[e]
			switch len(ns) {
			case 1:
				boundarySet[e] = true
				boundary = append(boundary, e)
			case 2:
				other := ns[0]
				if other == id {
					other = ns[1]
				}
				if !bad[other] {
					boundarySet[e] = true
					boundary = append(boundary, e)
				}
			}
		}
	}
	sortEdges(boundary)

	for id := range bad {
		m.killTri(id)
	}

	// Fan p over every cavity boundary edge. Each new triangle is
	// oriented strictly CCW, exactly as the one-shot Build fan does it.
	// The exceptional case is p landing exactly on a boundary edge: that
	// edge is split into (p,u) and (p,v) so both sides close.
	q := m.at(p)
	for _, e := range boundary {
		u, v := e.u, e.v
		s := geom.OrientSign(q, m.at(u), m.at(v))
		if s == 0 {
			for _, sub := range [2][2]int{{p, u}, {v, p}} {
				a, b := sub[0], sub[1]
				if a == b {
					continue
				}
				nt := workTri{a: p, b: a, c: b}
				if geom.OrientSign(m.at(nt.a), m.at(nt.b), m.at(nt.c)) < 0 {
					nt.b, nt.c = nt.c, nt.b
				}
				if geom.OrientSign(m.at(nt.a), m.at(nt.b), m.at(nt.c)) == 0 {
					continue
				}
				m.addTri(nt)
			}
			continue
		}
		nt := workTri{a: p, b: u, c: v}
		if s < 0 {
			nt.b, nt.c = v, u
		}
		m.addTri(nt)
	}

	// (The rare ghost-sliver gap is repaired by the edit layer only when
	// the freshly fanned mesh fails its consistency gate, so a normal fan
	// is never perturbed.)
	return nil
}

// closeStarGap closes, after the cavity fan, a real wedge of p's own star
// left uncovered because the in-circle component reached a ghost through
// a flat hull sliver and the ghost was shaved.
//
// It is a strict no-op on every ordinary fan (interior, hull or
// co-circular): it acts only when a living real edge that belongs to
// p's star has a single owner and is not a hull edge — the unambiguous
// signature of a hole. Two shapes are then closed:
//
//	(a) a single-owner edge (u,v) with both endpoints p's neighbours but
//	    not incident to p: add the missing fan triangle (p,u,v);
//	(b) two single-owner edges (p,u) and (p,v): add (p,u,v).
//
// New triangles flip to Delaunay in the caller.
func (m *Mesh) closeStarGap(p int) map[int]bool {
	// Real neighbours of p and owner counts of every living real edge.
	neigh := make(map[int]bool)
	edgeOwners := make(map[edge]int)
	for _, tr := range m.store {
		if !tr.alive || isGhost(tr.a) || isGhost(tr.b) || isGhost(tr.c) {
			continue
		}
		if tr.a == p || tr.b == p || tr.c == p {
			for _, v := range [3]int{tr.a, tr.b, tr.c} {
				if v != p {
					neigh[v] = true
				}
			}
		}
		for _, e := range [3]edge{newEdge(tr.a, tr.b), newEdge(tr.b, tr.c), newEdge(tr.c, tr.a)} {
			edgeOwners[e]++
		}
	}
	if len(neigh) < 2 {
		return nil
	}

	// For every edge, count its living REAL owners and whether a ghost
	// also owns it. A legitimate hull edge has one real owner and the
	// other side of the frame to itself (the real owner is the only real
	// triangle there). A sliver-gap edge instead shows up with one real
	// owner plus a ghost that should be replaced by a real fan triangle:
	// distinguish the two with the independent convex hull — only edges on
	// the current hull are allowed one real owner.
	hullIdx := geom.ConvexHull(m.work[:len(m.work)])
	hullEdge := make(map[edge]bool, len(hullIdx))
	for i := range hullIdx {
		hullEdge[newEdge(hullIdx[i], hullIdx[(i+1)%len(hullIdx)])] = true
	}
	realCount := func(e edge) int {
		n := 0
		for _, x := range m.edgeAdj[e] {
			t := m.store[x]
			if t.alive && !isGhost(t.a) && !isGhost(t.b) && !isGhost(t.c) {
				n++
			}
		}
		return n
	}

	// Collect the hole edges relevant to p.
	var innerEdges []edge
	var pEndpoints []int
	for e, c := range edgeOwners {
		// c counts real owners across the whole mesh.
		_ = c
		rc := realCount(e)
		if rc != 1 || hullEdge[e] {
			continue
		}
		if e.u == p || e.v == p {
			v := e.u
			if v == p {
				v = e.v
			}
			if neigh[v] {
				pEndpoints = append(pEndpoints, v)
			}
			continue
		}
		if neigh[e.u] && neigh[e.v] {
			innerEdges = append(innerEdges, e)
		}
	}
	if len(innerEdges) == 0 && len(pEndpoints) < 2 {
		return nil
	}

	var added map[int]bool
	add := func(u, v int) bool {
		if u == v || triExists(m, p, u, v) {
			return false
		}
		nt := workTri{a: p, b: u, c: v}
		if geom.OrientSign(m.at(nt.a), m.at(nt.b), m.at(nt.c)) < 0 {
			nt.b, nt.c = nt.c, nt.b
		}
		if geom.OrientSign(m.at(nt.a), m.at(nt.b), m.at(nt.c)) <= 0 {
			return false
		}
		id := m.addTri(nt)
		if added == nil {
			added = make(map[int]bool)
		}
		added[id] = true
		return true
	}

	for _, e := range innerEdges {
		add(e.u, e.v)
	}

	// Order the p-incident hole endpoints by polar angle and close each
	// consecutive positive-area wedge.
	if len(pEndpoints) >= 2 {
		type ae struct {
			v int
			a float64
		}
		ag := make([]ae, len(pEndpoints))
		qp := m.at(p)
		for i, v := range pEndpoints {
			rx := m.at(v).X - qp.X
			ry := m.at(v).Y - qp.Y
			ag[i] = ae{v, mathAtan2(ry, rx)}
		}
		for i := 1; i < len(ag); i++ {
			for j := i; j > 0 && ag[j].a < ag[j-1].a; j-- {
				ag[j], ag[j-1] = ag[j-1], ag[j]
			}
		}
		for i := 0; i+1 < len(ag); i++ {
			add(ag[i].v, ag[i+1].v)
		}
	}
	if added == nil {
		return nil
	}
	// Conservative acceptance: keep the repair only if it produced a
	// structurally valid mesh. If it introduced an overlap or otherwise
	// failed to close the hole (a misdiagnosed edge from a jitter-scale
	// hull discrepancy), roll every added triangle back so the normal
	// triangulation is left exactly untouched. The edit layer then falls
	// back to a whole rebuild rather than returning a corrupted mesh.
	if !m.locallyConsistent() {
		for id := range added {
			m.killTri(id)
		}
		return nil
	}
	return added
}

// triExists reports whether a living real triangle with the given vertex
// set is present.
func triExists(m *Mesh, a, b, c int) bool {
	for _, tid := range m.vertTris[a] {
		t := m.store[tid]
		if !t.alive {
			continue
		}
		if (t.a == a || t.b == a || t.c == a) &&
			(t.a == b || t.b == b || t.c == b) &&
			(t.a == c || t.b == c || t.c == c) {
			return true
		}
	}
	return false
}

// collectCavity returns the triangles to delete when p is inserted: the
// connected, star-shaped Bowyer–Watson cavity.
//
// The seed is the living triangle that geometrically CONTAINS p (two
// when p lies on an edge). Seeding every spatial candidate instead would
// seed the giant oversize ghost triangles whose circumcircles span the
// hull, folding the cavity. From the containing triangle a breadth-first
// walk crosses each shared edge using the rule
//
//   - real neighbour: absorb exactly when its circumcircle strictly
//     contains p (the ordinary Bowyer-Watson growth);
//   - ghost neighbour across a hull edge: absorb only when the edge
//     FACES p — p and the current triangle lie on opposite sides of the
//     edge line. An interior point therefore never absorbs a ghost (its
//     huge circumcircle would otherwise be taken as a reason), while an
//     exterior point crosses precisely the hull edges it can see.
//
// Both rules together grow the full in-circle component for interior
// insertions and the exact visible frame for hull insertions, and the
// union stays star-shaped with respect to p, so the boundary fans to p
// without overlap or gap.
func (m *Mesh) collectCavity(p int) (map[int]bool, *geom.Error) {
	q := m.at(p)
	bad := make(map[int]bool, 8)
	var queue []int
	for _, id := range m.idx.candidates(q) {
		t := m.store[id]
		if !t.alive || bad[id] {
			continue
		}
		if m.pointInTriangle(q, t.a, t.b, t.c) {
			bad[id] = true
			queue = append(queue, id)
		}
	}
	if len(bad) == 0 {
		// Robust fallback: if point location found no containing
		// triangle (an index-miss for a far-ranging point), seed with
		// every living candidate whose circumcircle contains q, the
		// original Bowyer–Watson seeding. The star-gap repair after the
		// fan still closes any non-star-shaped fold this admits.
		for _, id := range m.idx.candidates(q) {
			t := m.store[id]
			if !t.alive || bad[id] {
				continue
			}
			if geom.InCircleSign(m.at(t.a), m.at(t.b), m.at(t.c), q) > 0 {
				bad[id] = true
				queue = append(queue, id)
			}
		}
	}
	if len(bad) == 0 {
		return nil, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "insertion cavity empty; point escaped super-triangle or mesh numerically degenerate",
		}
	}

	for len(queue) > 0 {
		id := queue[0]
		queue = queue[1:]
		t := m.store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			other := m.livingOther(e, id)
			if other < 0 || bad[other] {
				continue
			}
			ot := m.store[other]
			if geom.InCircleSign(m.at(ot.a), m.at(ot.b), m.at(ot.c), q) > 0 {
				bad[other] = true
				queue = append(queue, other)
			}
		}
	}
	return bad, nil
}

// livingOther returns the single living triangle sharing edge e with id,
// or -1 when e has no other living owner (a virtual-frame boundary).
func (m *Mesh) livingOther(e edge, id int) int {
	for _, x := range m.edgeAdj[e] {
		if x != id && m.store[x].alive {
			return x
		}
	}
	return -1
}

// pointInTriangle reports q inside or on the boundary of CCW triangle
// (a,b,c), using the exact orientation predicate. Boundary inclusion is
// deliberate: a point landing on an existing edge seeds both adjacent
// triangles so the on-edge fan split has the complete cavity.
func (m *Mesh) pointInTriangle(q geom.Point, a, b, c int) bool {
	pa, pb, pc := m.at(a), m.at(b), m.at(c)
	// Normalize to CCW so the three same-side tests are unambiguous for
	// both real and ghost triangles.
	if geom.OrientSign(pa, pb, pc) < 0 {
		b, c = c, b
		pb, pc = pc, pb
	}
	return geom.OrientSign(pa, pb, q) >= 0 &&
		geom.OrientSign(pb, pc, q) >= 0 &&
		geom.OrientSign(pc, pa, q) >= 0
}
