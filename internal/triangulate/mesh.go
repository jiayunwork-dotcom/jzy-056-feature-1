package triangulate

import (
	"delaunaysvc/internal/geom"
)

// Coordinate-slot convention used throughout the live mesh:
//
//   - slots 0,1,2 are the three virtual super-triangle vertices. They
//     are permanent: when the frame has to grow (a point escapes the
//     current super triangle) their *coordinates* are overwritten in
//     place and the ghost fan is rebuilt, but the slot ids never change.
//   - real points occupy slots 3, 3+1, ... in insertion order. A real
//     point's stable external index (the index clients see) is its slot
//     minus 3; it never changes for the session's lifetime.
//
// Keeping the virtual vertices at fixed low slots lets Insert simply
// append a new coordinate at len(coords) while every real slot stays in
// the contiguous range [3, 3+nReal); ghosts are exactly the triangles
// touching slots 0,1,2.
const (
	slotS0 = 0
	slotS1 = 1
	slotS2 = 2
	superN = 3
)

// extID maps a real coordinate slot to its stable external index.
func extID(slot int) int { return slot - superN }

// coordSlot maps a stable external point index to its coordinate slot.
func coordSlot(ext int) int { return ext + superN }

// Mesh is a long-lived, mutable Delaunay triangulation maintained in the
// internal (origin-translated, deterministically jittered) coordinate
// frame. Unlike Build, which triangulates a whole point set and forgets
// it, a Mesh keeps every structure the Bowyer–Watson insertion step
// needs — the append-only triangle store, undirected-edge adjacency,
// per-vertex incidence and the uniform-grid spatial index — so that
// inserting or moving a single point only reworks the affected locality.
//
// The three virtual super-triangle vertices stay part of the living
// mesh for its whole lifetime; the ghost triangles touching them fill
// the exterior of the convex hull and are filtered from snapshots. The
// super frame is enlarged only when a coordinate escapes it
// (ensureFrame), and only ghost triangles are rebuilt then — interior
// topology is never touched.
//
// Every mutating operation runs inside a journaled transaction. On any
// failure the mesh is rewound to exactly its pre-operation state, so a
// rejected request can never leave a broken topology behind.
type Mesh struct {
	// coords holds the three super vertices followed by the real points
	// in internal-frame coordinates.
	coords []geom.Point
	// originals are the caller-provided coordinates of the real points,
	// indexed by external index; reported in snapshots and used for exact
	// duplicate detection at the session layer.
	originals []geom.Point
	origin    geom.Point

	store []workTri

	// edgeAdj: undirected edge -> living incident triangle IDs.
	edgeAdj map[edge][]int
	// vertAdj: vertex slot (virtual or real) -> living triangle IDs.
	vertAdj map[int][]int

	idx *spatialIndex

	nReal int

	// Fixed general-position perturbation magnitude for the mesh's whole
	// lifetime: every point, including later inserts and moves, receives
	// the same scale-free nudge derived from its stable external index.
	jitter float64
	// Bounding box of the jittered real coordinates; the active super
	// triangle strictly contains it.
	minX, minY, maxX, maxY float64

	// Hull ring in CCW order, given in external point indices.
	hull []int
}

// MeshChange is the incremental change set of one mutating operation:
// the triangles that disappeared and the triangles that appeared, given
// as CCW index triples in the stable external point-index space.
// Removing Removed from the old mesh and adding Added yields exactly the
// new mesh; the two sets never overlap.
type MeshChange struct {
	Removed []geom.Triangle `json:"removed"`
	Added   []geom.Triangle `json:"added"`
}

// NewMesh builds a stateful mesh over an already validated point set
// (the same preconditions Build requires). The existing one-shot
// validation and the verified per-point insertion machinery are reused;
// this constructor only adds the persistent frame around them.
func NewMesh(points []geom.Point) (*Mesh, *geom.Error) {
	n := len(points)
	origin := points[0]

	// Internal-frame (translated) real coordinates and their bounds.
	real := make([]geom.Point, n)
	minX, minY := 0.0, 0.0
	maxX, maxY := 0.0, 0.0
	for i, p := range points {
		q := geom.Point{X: p.X - origin.X, Y: p.Y - origin.Y}
		real[i] = q
		if i > 0 {
			minX = mathMin(minX, q.X)
			maxX = mathMax(maxX, q.X)
			minY = mathMin(minY, q.Y)
			maxY = mathMax(maxY, q.Y)
		}
	}
	span := mathMax(maxX-minX, maxY-minY)
	if span == 0 {
		span = 1
	}
	jh := jitterSpan(span)
	for i := range real {
		fx, fy := jitterFractions(i)
		real[i].X += jh * fx
		real[i].Y += jh * fy
	}

	m := &Mesh{
		originals: append([]geom.Point(nil), points...),
		origin:    origin,
		edgeAdj:   make(map[edge][]int),
		vertAdj:   make(map[int][]int),
		nReal:     n,
		jitter:    jh,
		minX:      minX, minY: minY, maxX: maxX, maxY: maxY,
	}
	if gerr := m.bootstrap(real); gerr != nil {
		return nil, gerr
	}
	return m, nil
}

// bootstrap opens the initial super triangle in slots 0,1,2, appends the
// real points at slots 3.. and inserts them in order with the shared
// cavity/fan routine.
func (m *Mesh) bootstrap(real []geom.Point) *geom.Error {
	s0, s1, s2 := superTriangleFromBounds(m.minX, m.minY, m.maxX, m.maxY)
	m.coords = make([]geom.Point, 0, superN+len(real))
	m.coords = append(m.coords, s0, s1, s2)
	m.coords = append(m.coords, real...)

	m.idx = newSpatialIndex(m.minX, m.minY, m.maxX-m.minX, m.maxY-m.minY, m.nReal)

	m.store = m.store[:0]
	root := workTri{a: slotS0, b: slotS1, c: slotS2, alive: true}
	root = m.idx.insertTriangle(0, root, m.coords)
	m.store = append(m.store, root)
	m.edgeAdj[newEdge(slotS0, slotS1)] = []int{0}
	m.edgeAdj[newEdge(slotS1, slotS2)] = []int{0}
	m.edgeAdj[newEdge(slotS2, slotS0)] = []int{0}
	m.vertAdj[slotS0] = []int{0}
	m.vertAdj[slotS1] = []int{0}
	m.vertAdj[slotS2] = []int{0}

	// Bootstrap failures discard the half-built mesh; the transaction's
	// undo bookkeeping is irrelevant there, but its maps must exist.
	x := m.begin()
	for i := 0; i < m.nReal; i++ {
		if _, gerr := m.insertCavityFanTx(x, coordSlot(i)); gerr != nil {
			return gerr
		}
	}

	hull, herr := m.boundaryRingFromStore()
	if herr != nil {
		return herr
	}
	m.hull = externalSlots(hull)
	return nil
}

// PointCount returns the number of real points currently in the mesh.
func (m *Mesh) PointCount() int { return m.nReal }

// Origin returns the translation vector of the internal frame.
func (m *Mesh) Origin() geom.Point { return m.origin }

// isGhost reports whether a triangle touches a virtual vertex.
func isGhost(t workTri) bool {
	return t.a < superN || t.b < superN || t.c < superN
}

// Snapshot is a read-only view of the current mesh in the same shape as
// the one-shot Build result: ghost-free CCW triangles, the hull ring and
// both coordinate frames, all in external point indices. The returned
// slices are freshly allocated.
func (m *Mesh) Snapshot() *Result {
	final := make([]geom.Triangle, 0, 2*m.nReal)
	for _, t := range m.store {
		if !t.alive || isGhost(t) {
			continue
		}
		final = append(final, geom.Triangle{
			A: extID(t.a), B: extID(t.b), C: extID(t.c),
		})
	}
	canonicalize(final)

	// Work coordinates in external order.
	work := make([]geom.Point, m.nReal)
	for i := 0; i < m.nReal; i++ {
		work[i] = m.coords[coordSlot(i)]
	}
	return &Result{
		Points:    append([]geom.Point(nil), m.originals...),
		work:      work,
		Triangles: final,
		Hull:      append([]int(nil), m.hull...),
		Origin:    m.origin,
	}
}

// OriginalPoint returns the caller-provided coordinate of real point i
// (external index).
func (m *Mesh) OriginalPoint(i int) geom.Point { return m.originals[i] }

// HasPointAt reports whether any real point has exactly the given
// caller-provided coordinate.
func (m *Mesh) HasPointAt(p geom.Point) bool {
	for _, q := range m.originals {
		if q == p {
			return true
		}
	}
	return false
}

// ---- journaled transactions ---------------------------------------------

type undoKind int

const (
	undoNop    undoKind = iota
	undoKill            // revive a triangle that existed before the operation
	undoSpawn           // remove a triangle spawned by the operation
	undoCoord           // restore one coordinate slot (real move OR super vertex)
	undoAppend          // truncate appended coordinate slots
	undoBounds
	undoFrame
)

type undo struct {
	kind     undoKind
	id       int
	tri      workTri
	slot     int
	p        geom.Point
	ext      int
	op       geom.Point // old external coordinate (real move)
	b        bounds
	frame    *frameUndo
	appended []geom.Point // slots dropped on truncation (not needed; len only)
}

type bounds struct{ minX, minY, maxX, maxY float64 }

type killedGhost struct {
	id  int
	tri workTri
}

type frameUndo struct {
	// Super vertex coordinates to restore and the index in use before
	// the rebuild.
	s0, s1, s2 geom.Point
	idx        *spatialIndex
	b          bounds
	// Old living ghost triangles, revived on rollback.
	killed []killedGhost
	// Number of coordinate slots present before the rebuild appended any.
	coordsLen int
	// Slots appended by a rebuild (currently always zero because super
	// vertices live at fixed slots; kept for completeness).
	newSuper [3]geom.Point
	// Ghost triangle IDs spawned by the rebuild (dropped on rollback).
	newGhostIDs []int
}

type txn struct {
	m       *Mesh
	undos   []undo
	born    map[int]bool // triangles spawned during this transaction
	killed  []int        // pre-existing triangles killed (change set)
	spawned []int        // all triangles spawned (change set)
}

func (m *Mesh) begin() *txn {
	return &txn{m: m, born: map[int]bool{}}
}

func (x *txn) rollback() {
	m := x.m
	for k := len(x.undos) - 1; k >= 0; k-- {
		u := x.undos[k]
		switch u.kind {
		case undoSpawn:
			t := m.store[u.id]
			if t.alive {
				x.removeAdj(u.id, t)
				m.idx.removeIn(u.id, t)
				t.alive = false
				m.store[u.id] = t
			}
		case undoKill:
			t := u.tri
			t.alive = true
			m.store[u.id] = t
			x.addAdj(u.id, t)
			m.idx.insertTriangle(u.id, t, m.coords)
		case undoCoord:
			if u.ext >= 0 {
				m.coords[u.slot] = u.p
				m.originals[u.ext] = u.op
			} else {
				m.coords[u.slot] = u.p
			}
		case undoAppend:
			m.coords = m.coords[:u.id]
		case undoBounds:
			m.minX, m.minY, m.maxX, m.maxY = u.b.minX, u.b.minY, u.b.maxX, u.b.maxY
		case undoFrame:
			rollbackFrame(x, u.frame)
		}
	}
}

// rollbackFrame undoes a ghost-frame rebuild: the frame's new ghost
// triangles are removed, old ghosts revived, super coordinates and the
// old spatial index restored.
func rollbackFrame(x *txn, f *frameUndo) {
	m := x.m
	for _, id := range f.newGhostIDs {
		t := m.store[id]
		if !t.alive {
			continue
		}
		x.removeAdj(id, t)
		m.idx.removeIn(id, t)
		t.alive = false
		m.store[id] = t
	}
	m.coords = m.coords[:f.coordsLen]
	m.coords[slotS0] = f.s0
	m.coords[slotS1] = f.s1
	m.coords[slotS2] = f.s2
	m.idx = f.idx
	for _, kg := range f.killed {
		t := kg.tri
		t.alive = true
		m.store[kg.id] = t
		x.addAdj(kg.id, t)
		m.idx.insertTriangle(kg.id, t, m.coords)
	}
	m.minX, m.minY, m.maxX, m.maxY = f.b.minX, f.b.minY, f.b.maxX, f.b.maxY
}

// commit finalizes a successful operation and builds its change set in
// external indices: real triangles present before but not after
// (Removed), present after but not before (Added).
//
// Triangle identity in a change set is the vertex SET, independent of
// orientation: when a point moves, a surviving triangle whose vertex set
// is unchanged may reverse its CCW cyclic order purely because one
// vertex changed coordinates, so the set algebra must never key on cyclic
// orientation. Added triangles are then rendered in the current CCW
// order (identical to Snapshot's representation); removed triangles are
// rendered with ascending vertex indices, since under the new point
// coordinates the old cyclic order need not even be CCW.
func (x *txn) commit() MeshChange {
	m := x.m
	before := make(map[[3]int]bool, 8)
	after := make(map[[3]int]bool, 8)
	afterCCW := make(map[[3]int]geom.Triangle, 8)
	for _, id := range x.killed {
		t := m.store[id] // dead slot; vertices unchanged
		if !isGhost(t) {
			before[vertexSet(t)] = true
		}
	}
	for _, id := range x.spawned {
		t := m.store[id]
		if t.alive && !isGhost(t) {
			k := vertexSet(t)
			after[k] = true
			afterCCW[k] = externalTri(t)
		}
	}
	ch := MeshChange{Removed: []geom.Triangle{}, Added: []geom.Triangle{}}
	for k := range before {
		if !after[k] {
			ch.Removed = append(ch.Removed, geom.Triangle{A: k[0], B: k[1], C: k[2]})
		}
	}
	for k := range after {
		if !before[k] {
			ch.Added = append(ch.Added, afterCCW[k])
		}
	}
	canonicalize(ch.Removed)
	canonicalize(ch.Added)
	return ch
}

// vertexSet returns the ascending-sorted external vertex indices of a
// work triangle: an orientation-independent identity.
func vertexSet(t workTri) [3]int {
	a := []int{extID(t.a), extID(t.b), extID(t.c)}
	for i := 1; i < 3; i++ {
		for j := i; j > 0 && a[j] < a[j-1]; j-- {
			a[j], a[j-1] = a[j-1], a[j]
		}
	}
	return [3]int{a[0], a[1], a[2]}
}

// externalTri renders a (slot-indexed) workTri as a CCW triangle in
// external indices, with A the smallest external index.
func externalTri(t workTri) geom.Triangle {
	vs := [3]int{extID(t.a), extID(t.b), extID(t.c)}
	k := 0
	for i := 1; i < 3; i++ {
		if vs[i] < vs[k] {
			k = i
		}
	}
	return geom.Triangle{A: vs[k], B: vs[(k+1)%3], C: vs[(k+2)%3]}
}

// addAdj/removeAdj maintain the two adjacency maps.
func (x *txn) addAdj(id int, t workTri) {
	m := x.m
	for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
		m.edgeAdj[e] = append(m.edgeAdj[e], id)
	}
	for _, v := range []int{t.a, t.b, t.c} {
		m.vertAdj[v] = append(m.vertAdj[v], id)
	}
}

func (x *txn) removeAdj(id int, t workTri) {
	m := x.m
	for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
		ns := m.edgeAdj[e][:0]
		for _, v := range m.edgeAdj[e] {
			if v != id {
				ns = append(ns, v)
			}
		}
		m.edgeAdj[e] = ns
	}
	for _, v := range []int{t.a, t.b, t.c} {
		ns := m.vertAdj[v][:0]
		for _, w := range m.vertAdj[v] {
			if w != id {
				ns = append(ns, w)
			}
		}
		m.vertAdj[v] = ns
	}
}

// kill removes a living triangle from the mesh. A triangle born earlier
// in the same operation simply disappears; a pre-existing triangle
// records a revive undo.
func (x *txn) kill(id int) {
	m := x.m
	t := m.store[id]
	x.removeAdj(id, t)
	m.idx.removeIn(id, t)
	t.alive = false
	m.store[id] = t

	if x.born[id] {
		for k := range x.undos {
			if x.undos[k].kind == undoSpawn && x.undos[k].id == id {
				x.undos[k].kind = undoNop
				break
			}
		}
		// Drop it from the change set: a triangle born and later removed
		// within the same operation never existed in either the pre- or
		// post-operation mesh.
		for k, sid := range x.spawned {
			if sid == id {
				x.spawned = append(x.spawned[:k], x.spawned[k+1:]...)
				break
			}
		}
		delete(x.born, id)
		return
	}
	x.undos = append(x.undos, undo{kind: undoKill, id: id, tri: t})
	x.killed = append(x.killed, id)
}

// spawnCCW creates a living triangle (p,u,v), orienting it strictly CCW;
// collinear triples produce no triangle (ok=false), matching the
// one-shot fan's on-edge handling.
func (x *txn) spawnCCW(p, u, v int) (id int, ok bool, gerr *geom.Error) {
	m := x.m
	t := workTri{a: p, b: u, c: v, alive: true}
	switch geom.OrientSign(m.coords[p], m.coords[u], m.coords[v]) {
	case -1:
		t.b, t.c = v, u
	case 0:
		return 0, false, nil
	}
	id = len(m.store)
	m.store = append(m.store, workTri{})
	t = m.idx.insertTriangle(id, t, m.coords)
	m.store[id] = t
	x.addAdj(id, t)
	x.undos = append(x.undos, undo{kind: undoSpawn, id: id, tri: t})
	x.born[id] = true
	x.spawned = append(x.spawned, id)
	return id, true, nil
}

// ---- insert --------------------------------------------------------------

// Insert adds one new point to the mesh and returns the change set. The
// coordinate is in the caller's (external) frame. Duplicate rejection is
// the session layer's responsibility.
func (m *Mesh) Insert(p geom.Point) (MeshChange, *geom.Error) {
	x := m.begin()

	q := geom.Point{X: p.X - m.origin.X, Y: p.Y - m.origin.Y}
	fx, fy := jitterFractions(m.nReal) // external index of the new point
	qj := geom.Point{X: q.X + m.jitter*fx, Y: q.Y + m.jitter*fy}

	oldBounds := bounds{m.minX, m.minY, m.maxX, m.maxY}
	if !m.frameContains(qj) {
		if gerr := m.ensureFrame(x, qj); gerr != nil {
			x.rollback()
			return MeshChange{}, gerr
		}
	}
	x.undos = append(x.undos, undo{kind: undoBounds, b: oldBounds})

	slot := len(m.coords)
	m.coords = append(m.coords, qj)
	x.undos = append(x.undos, undo{kind: undoAppend, id: slot})
	m.originals = append(m.originals, p)
	m.nReal++
	m.includeInBounds(qj)

	if _, gerr := m.insertCavityFanTx(x, slot); gerr != nil {
		x.rollback()
		m.nReal--
		m.originals = m.originals[:m.nReal]
		return MeshChange{}, gerr
	}
	if gerr := m.recomputeHull(); gerr != nil {
		x.rollback()
		m.nReal--
		m.originals = m.originals[:m.nReal]
		return MeshChange{}, gerr
	}
	return x.commit(), nil
}

// Move relocates existing real point i (external index) to p and returns
// the change set. It is "collapse the point's local star, retriangulate
// the hole, insert at the new spot" in one atomic operation; the
// reconnection after removal and the reinsertion both reuse the same
// local dig-and-fan logic.
func (m *Mesh) Move(i int, p geom.Point) (MeshChange, *geom.Error) {
	if i < 0 || i >= m.nReal {
		return MeshChange{}, &geom.Error{
			Code:    geom.ErrPointNotFound,
			Message: "mesh: cannot move non-existent point",
		}
	}
	x := m.begin()
	slot := coordSlot(i)

	q := geom.Point{X: p.X - m.origin.X, Y: p.Y - m.origin.Y}
	fxj, fyj := jitterFractions(i)
	qj := geom.Point{X: q.X + m.jitter*fxj, Y: q.Y + m.jitter*fyj}

	oldBounds := bounds{m.minX, m.minY, m.maxX, m.maxY}
	if !m.frameContains(qj) {
		if gerr := m.ensureFrame(x, qj); gerr != nil {
			x.rollback()
			return MeshChange{}, gerr
		}
	}
	x.undos = append(x.undos, undo{kind: undoBounds, b: oldBounds})

	// 1. Close the star of the vertex: link edges (star edges not
	// incident to it) form the boundary of the hole; super vertices are
	// ordinary link members for hull vertices.
	starIDs := append([]int(nil), m.vertAdj[slot]...)
	link := m.linkRing(slot, starIDs)
	if link == nil {
		x.rollback()
		return MeshChange{}, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "mesh: link ring of moved vertex is not a closed loop",
		}
	}

	// 2. Kill the star.
	for _, id := range starIDs {
		x.kill(id)
	}

	// 3. Ear-clip the link polygon so the mesh (point set without i) is
	// conforming again before reinsertion.
	ring := m.orientRingCCW(link)
	filler, gerr := m.earClipFill(x, ring)
	if gerr != nil {
		x.rollback()
		return MeshChange{}, gerr
	}

	// 4. Lawson flips restore Delaunay-ness of the temporary mesh.
	if gerr := m.legalize(x, filler); gerr != nil {
		x.rollback()
		return MeshChange{}, gerr
	}

	// 5. Relocate the point and run the identical cavity/fan insertion.
	x.undos = append(x.undos, undo{
		kind: undoCoord, slot: slot, ext: i,
		p: m.coords[slot], op: m.originals[i],
	})
	m.coords[slot] = qj
	m.originals[i] = p
	m.includeInBounds(qj)

	if _, gerr := m.insertCavityFanTx(x, slot); gerr != nil {
		x.rollback()
		return MeshChange{}, gerr
	}
	if gerr := m.recomputeHull(); gerr != nil {
		x.rollback()
		return MeshChange{}, gerr
	}
	return x.commit(), nil
}

// ---- the shared cavity/fan kernel ----------------------------------------

// insertCavityFanTx performs one Bowyer–Watson insertion of coordinate
// slot p against the current mesh: find every triangle whose
// circumdisk contains p (edge-connected BFS from spatial-index seeds),
// delete the cavity, fan p over its boundary. This is the exact
// "dig a hole, reconnect" step used by the one-shot builder, localized
// and reused verbatim by Insert and Move.
func (m *Mesh) insertCavityFanTx(x *txn, p int) ([]int, *geom.Error) {
	bad, gerr := m.collectCavity(p)
	if gerr != nil {
		return nil, gerr
	}

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
		x.kill(id)
	}

	var born []int
	for _, e := range boundary {
		id, ok, ferr := x.spawnCCW(p, e.u, e.v)
		if ferr != nil {
			return nil, ferr
		}
		if ok {
			born = append(born, id)
		}
	}
	if len(born) == 0 {
		return nil, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "insertion produced no triangles; point set numerically degenerate",
		}
	}
	return born, nil
}

// collectCavity mirrors the one-shot builder's cavity walk on live state.
func (m *Mesh) collectCavity(p int) (map[int]bool, *geom.Error) {
	q := m.coords[p]
	bad := make(map[int]bool, 8)
	var queue []int
	for _, id := range m.idx.candidates(q) {
		t := m.store[id]
		if t.alive && !bad[id] &&
			geom.InCircleSign(m.coords[t.a], m.coords[t.b], m.coords[t.c], q) > 0 {
			bad[id] = true
			queue = append(queue, id)
		}
	}
	if len(bad) == 0 {
		return nil, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "insertion cavity empty; point set numerically degenerate",
		}
	}

	for len(queue) > 0 {
		id := queue[0]
		queue = queue[1:]
		t := m.store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			var other int = -1
			for _, z := range m.edgeAdj[e] {
				if z != id && m.store[z].alive {
					other = z
					break
				}
			}
			if other < 0 || bad[other] {
				continue
			}
			ot := m.store[other]
			if geom.InCircleSign(m.coords[ot.a], m.coords[ot.b], m.coords[ot.c], q) > 0 {
				bad[other] = true
				queue = append(queue, other)
				continue
			}
			// Keep the cavity star-shaped w.r.t. p. The visibility test
			// is oriented by the CAVITY-side triangle id: its third
			// vertex marks the edge's interior side.
			if !m.boundaryEdgeVisible(id, e, q) {
				bad[other] = true
				queue = append(queue, other)
			}
		}
	}
	return bad, nil
}

func (m *Mesh) boundaryEdgeVisible(cavityID int, e edge, p geom.Point) bool {
	u, v := e.u, e.v
	ct := m.store[cavityID]
	w := thirdVertex(ct, u, v)
	if geom.OrientSign(m.coords[u], m.coords[v], m.coords[w]) < 0 {
		u, v = v, u
	}
	return geom.OrientSign(m.coords[u], m.coords[v], p) >= 0
}

// ---- move support: link ring, ear clipping, legalization ----------------

// linkRing closes the star of vertex v: it returns the cycle of
// neighbour vertices where consecutive vertices bound one star
// triangle. Ghost vertices are ordinary cycle members for hull vertices.
func (m *Mesh) linkRing(v int, starIDs []int) []int {
	if len(starIDs) == 0 {
		return nil
	}
	// For each spoke v->u record the next spoke v->w reached by rotating
	// inside the CCW star triangle (v,u,w).
	next := make(map[int]int, len(starIDs))
	for _, id := range starIDs {
		t := m.store[id]
		var u, w int
		switch v {
		case t.a:
			u, w = t.b, t.c
		case t.b:
			u, w = t.c, t.a
		default:
			u, w = t.a, t.b
		}
		if _, dup := next[u]; dup {
			return nil // non-manifold star
		}
		next[u] = w
	}
	var start int
	for u := range next {
		start = u
		break
	}
	ring := make([]int, 0, len(next))
	cur := start
	for {
		ring = append(ring, cur)
		nxt, ok := next[cur]
		if !ok {
			return nil
		}
		cur = nxt
		if cur == start {
			break
		}
		if len(ring) > len(next) {
			return nil
		}
	}
	return ring
}

// orientRingCCW returns a copy of ring oriented CCW (positive signed
// area) in the internal coordinate frame.
func (m *Mesh) orientRingCCW(ring []int) []int {
	var s2 float64
	for i := 0; i < len(ring); i++ {
		a := m.coords[ring[i]]
		b := m.coords[ring[(i+1)%len(ring)]]
		s2 += a.X*b.Y - b.X*a.Y
	}
	out := append([]int(nil), ring...)
	if s2 < 0 {
		for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
			out[i], out[j] = out[j], out[i]
		}
	}
	return out
}

// earClipFill triangulates the simple CCW polygon ring (ghost vertices
// allowed) by ear clipping, spawning the filler triangles. The star of
// the removed vertex was already killed, so the hole contains no
// elements.
func (m *Mesh) earClipFill(x *txn, ring []int) ([]int, *geom.Error) {
	var born []int
	type node struct {
		v, prev, next int
		alive         bool
	}
	nodes := make([]node, len(ring))
	for i, v := range ring {
		nodes[i] = node{v: v,
			prev: (i - 1 + len(ring)) % len(ring),
			next: (i + 1) % len(ring), alive: true}
	}
	count := len(ring)

	// blocked reports whether another surviving polygon vertex lies in
	// the closed ear triangle (a,b,c). On-edge blockers count: a diagonal
	// through another vertex would not be a valid fan edge.
	blocked := func(ai, bi, ci int) bool {
		a, b, c := m.coords[nodes[ai].v], m.coords[nodes[bi].v], m.coords[nodes[ci].v]
		for k := range nodes {
			if !nodes[k].alive || k == ai || k == bi || k == ci {
				continue
			}
			p := m.coords[nodes[k].v]
			if geom.OrientSign(a, b, p) >= 0 &&
				geom.OrientSign(b, c, p) >= 0 &&
				geom.OrientSign(c, a, p) >= 0 {
				return true
			}
		}
		return false
	}

	spawnEar := func(ai, bi, ci int) *geom.Error {
		id, ok, gerr := x.spawnCCW(nodes[ai].v, nodes[bi].v, nodes[ci].v)
		if gerr != nil {
			return gerr
		}
		if !ok {
			return &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh: zero-area ear while retriangulating cavity",
			}
		}
		born = append(born, id)
		return nil
	}

	for count > 3 {
		found := false
		for k := range nodes {
			if !nodes[k].alive {
				continue
			}
			pr, nx := nodes[k].prev, nodes[k].next
			if pr == nx {
				return nil, m.earDegenerate()
			}
			// Convex, positive-area ear tip with an empty ear triangle.
			if geom.OrientSign(m.coords[nodes[pr].v], m.coords[nodes[k].v],
				m.coords[nodes[nx].v]) <= 0 {
				continue
			}
			if blocked(pr, k, nx) {
				continue
			}
			if gerr := spawnEar(pr, k, nx); gerr != nil {
				return nil, gerr
			}
			nodes[pr].next = nx
			nodes[nx].prev = pr
			nodes[k].alive = false
			count--
			found = true
			break
		}
		if !found {
			return nil, &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh: no clippable ear while retriangulating cavity",
			}
		}
	}
	var rem []int
	for k := range nodes {
		if nodes[k].alive {
			rem = append(rem, k)
		}
	}
	if len(rem) != 3 {
		return nil, m.earDegenerate()
	}
	if gerr := spawnEar(rem[0], rem[1], rem[2]); gerr != nil {
		return nil, gerr
	}
	return born, nil
}

func (m *Mesh) earDegenerate() *geom.Error {
	return &geom.Error{
		Code:    geom.ErrDegenerate,
		Message: "mesh: degenerate cavity polygon while retriangulating",
	}
}

// legalize repeatedly flips interior edges that violate the local
// Delaunay condition, seeded from the freshly spawned triangle set.
// Lawson's flip theorem guarantees convergence to a Delaunay
// triangulation from any conforming start mesh; edges whose quad is not
// strictly convex are never flipped (a flip there would fold the mesh),
// which also rules out every exterior edge.
func (m *Mesh) legalize(x *txn, seed []int) *geom.Error {
	var queue []edge
	seen := make(map[edge]bool)
	enqueueTri := func(id int) {
		t := m.store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			if !seen[e] {
				seen[e] = true
				queue = append(queue, e)
			}
		}
	}
	for _, id := range seed {
		if m.store[id].alive {
			enqueueTri(id)
		}
	}

	for len(queue) > 0 {
		e := queue[0]
		queue = queue[1:]
		delete(seen, e)

		owners := m.edgeAdj[e]
		if len(owners) != 2 {
			continue
		}
		t1id, t2id := owners[0], owners[1]
		t1, t2 := m.store[t1id], m.store[t2id]
		opp1 := thirdVertex(t1, e.u, e.v)
		opp2 := thirdVertex(t2, e.u, e.v)

		if geom.OrientSign(m.coords[e.u], m.coords[e.v], m.coords[opp1])*
			geom.OrientSign(m.coords[e.u], m.coords[e.v], m.coords[opp2]) >= 0 {
			continue // non-convex or degenerate quad
		}
		if geom.InCircleSign(m.coords[t1.a], m.coords[t1.b], m.coords[t1.c],
			m.coords[opp2]) <= 0 {
			continue // locally Delaunay (on-circle is legal)
		}

		x.kill(t1id)
		x.kill(t2id)
		id1, ok1, gerr := x.spawnCCW(opp1, e.u, opp2)
		if gerr != nil {
			return gerr
		}
		id2, ok2, gerr := x.spawnCCW(opp2, e.v, opp1)
		if gerr != nil {
			return gerr
		}
		if !ok1 || !ok2 {
			return &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh: degenerate quad during Delaunay legalization",
			}
		}
		enqueueTri(id1)
		enqueueTri(id2)
	}
	return nil
}

// ---- frame management ----------------------------------------------------

func (m *Mesh) frameContains(q geom.Point) bool {
	a, b, c := m.coords[slotS0], m.coords[slotS1], m.coords[slotS2]
	return geom.OrientSign(a, b, q) > 0 &&
		geom.OrientSign(b, c, q) > 0 &&
		geom.OrientSign(c, a, q) > 0
}

func (m *Mesh) includeInBounds(q geom.Point) {
	m.minX = mathMin(m.minX, q.X)
	m.maxX = mathMax(m.maxX, q.X)
	m.minY = mathMin(m.minY, q.Y)
	m.maxY = mathMax(m.maxY, q.Y)
}

// ensureFrame enlarges the virtual super frame so it strictly contains
// the current point set plus q, rebuilding only the ghost triangulation
// between the hull and the new super triangle. Interior triangles are
// never touched. The super vertices keep their slots (0,1,2); only their
// coordinates change. The rebuild is one journaled compound undo.
func (m *Mesh) ensureFrame(x *txn, q geom.Point) *geom.Error {
	f := &frameUndo{
		s0:        m.coords[slotS0],
		s1:        m.coords[slotS1],
		s2:        m.coords[slotS2],
		idx:       m.idx,
		b:         bounds{m.minX, m.minY, m.maxX, m.maxY},
		coordsLen: len(m.coords),
	}

	newMinX, newMinY := mathMin(m.minX, q.X), mathMin(m.minY, q.Y)
	newMaxX, newMaxY := mathMax(m.maxX, q.X), mathMax(m.maxY, q.Y)

	// Snapshot and remove the living old ghosts; the compound frame undo
	// revives them as a whole.
	oldGhostIDs := map[int]bool{}
	for _, v := range []int{slotS0, slotS1, slotS2} {
		for _, id := range m.vertAdj[v] {
			oldGhostIDs[id] = true
		}
	}
	for id := range oldGhostIDs {
		t := m.store[id]
		f.killed = append(f.killed, killedGhost{id: id, tri: t})
		x.removeAdj(id, t)
		m.idx.removeIn(id, t)
		t.alive = false
		m.store[id] = t
	}

	// Move the super frame: overwrite the fixed slots in place.
	s0, s1, s2 := superTriangleFromBounds(newMinX, newMinY, newMaxX, newMaxY)
	m.coords[slotS0], m.coords[slotS1], m.coords[slotS2] = s0, s1, s2

	// Rebuild the spatial index over all surviving (interior) triangles.
	nidx := newSpatialIndex(newMinX, newMinY, newMaxX-newMinX, newMaxY-newMinY, m.nReal)
	for id := range m.store {
		t := m.store[id]
		if t.alive {
			t = nidx.insertTriangle(id, t, m.coords)
			m.store[id] = t
		}
	}

	ghosts, gerr := m.ghostTriangulation(m.hullSlots(),
		newMinX, newMinY, newMaxX, newMaxY)
	if gerr != nil {
		// Restore super coords and rewind via the compound undo.
		m.coords[slotS0], m.coords[slotS1], m.coords[slotS2] = f.s0, f.s1, f.s2
		x.undos = append(x.undos, undo{kind: undoFrame, frame: f})
		x.rollback()
		return gerr
	}
	m.idx = nidx

	for _, gt := range ghosts {
		id := len(m.store)
		m.store = append(m.store, workTri{})
		nt := workTri{a: gt[0], b: gt[1], c: gt[2], alive: true}
		nt = m.idx.insertTriangle(id, nt, m.coords)
		m.store[id] = nt
		x.addAdj(id, nt)
		f.newGhostIDs = append(f.newGhostIDs, id)
	}

	m.minX, m.minY, m.maxX, m.maxY = newMinX, newMinY, newMaxX, newMaxY
	x.undos = append(x.undos, undo{kind: undoFrame, frame: f})

	hullSlots, herr := m.boundaryRingFromStore()
	if herr != nil {
		x.rollback()
		return herr
	}
	m.hull = externalSlots(hullSlots)
	return nil
}

// hullSlots returns the current hull in coordinate slots.
func (m *Mesh) hullSlots() []int {
	out := make([]int, len(m.hull))
	for i, e := range m.hull {
		out[i] = coordSlot(e)
	}
	return out
}

func externalSlots(slots []int) []int {
	out := make([]int, len(slots))
	for i, s := range slots {
		out[i] = extID(s)
	}
	return out
}

// ghostTriangulation computes the ghost triangles of the hull point set
// (given in coordinate slots) against the freshly installed super
// triangle without disturbing the live mesh: a throwaway Bowyer–Watson
// build over the hull points plus the super vertices, using coordinate
// values identical to the live frame. Its ghost fan is exactly the ghost
// fan a from-scratch build would produce for the full point set (hull
// edges are unaffected by interior points), so adopting it keeps every
// remaining triangle Delaunay.
func (m *Mesh) ghostTriangulation(hullSlots []int,
	minX, minY, maxX, maxY float64) ([][3]int, *geom.Error) {

	h := len(hullSlots)
	// Local indices: [0,h) hull points, h..h+2 the super vertices.
	coords := make([]geom.Point, 0, h+superN)
	for _, si := range hullSlots {
		coords = append(coords, m.coords[si])
	}
	coords = append(coords, m.coords[slotS0], m.coords[slotS1], m.coords[slotS2])

	store := []workTri{{a: h, b: h + 1, c: h + 2, alive: true}}
	edgeAdj := make(map[edge][]int)
	edgeAdj[newEdge(h, h+1)] = []int{0}
	edgeAdj[newEdge(h+1, h+2)] = []int{0}
	edgeAdj[newEdge(h+2, h)] = []int{0}
	tidx := newSpatialIndex(minX, minY, maxX-minX, maxY-minY, h)
	store[0] = tidx.insertTriangle(0, store[0], coords)

	removeAdjStatic := func(id int, t workTri) {
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			ns := edgeAdj[e][:0]
			for _, z := range edgeAdj[e] {
				if z != id {
					ns = append(ns, z)
				}
			}
			edgeAdj[e] = ns
		}
	}

	for p := 0; p < h; p++ {
		bad, gerr := collectCavityStatic(p, coords, store, tidx, edgeAdj)
		if gerr != nil {
			return nil, gerr
		}
		var boundary []edge
		bset := map[edge]bool{}
		for id := range bad {
			t := store[id]
			for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
				if bset[e] {
					continue
				}
				ns := edgeAdj[e]
				switch len(ns) {
				case 1:
					bset[e] = true
					boundary = append(boundary, e)
				case 2:
					other := ns[0]
					if other == id {
						other = ns[1]
					}
					if !bad[other] {
						bset[e] = true
						boundary = append(boundary, e)
					}
				}
			}
		}
		sortEdges(boundary)
		for id := range bad {
			t := store[id]
			removeAdjStatic(id, t)
			tidx.removeIn(id, t)
			t.alive = false
			store[id] = t
		}
		for _, e := range boundary {
			nt := workTri{a: p, b: e.u, c: e.v, alive: true}
			switch geom.OrientSign(coords[p], coords[e.u], coords[e.v]) {
			case -1:
				nt.b, nt.c = e.v, e.u
			case 0:
				continue
			}
			id := len(store)
			store = append(store, workTri{})
			nt = tidx.insertTriangle(id, nt, coords)
			store[id] = nt
			edgeAdj[newEdge(nt.a, nt.b)] = append(edgeAdj[newEdge(nt.a, nt.b)], id)
			edgeAdj[newEdge(nt.b, nt.c)] = append(edgeAdj[newEdge(nt.b, nt.c)], id)
			edgeAdj[newEdge(nt.c, nt.a)] = append(edgeAdj[newEdge(nt.c, nt.a)], id)
		}
	}

	var ghosts [][3]int
	for _, t := range store {
		if !t.alive || (t.a < h && t.b < h && t.c < h) {
			continue
		}
		remap := func(li int) int {
			if li < h {
				return hullSlots[li]
			}
			return (li - h) // super local h,h+1,h+2 -> slots 0,1,2
		}
		ghosts = append(ghosts, [3]int{remap(t.a), remap(t.b), remap(t.c)})
	}
	return ghosts, nil
}

// collectCavityStatic is the free-function cavity walker for the
// throwaway hull-only ghost build (identical logic, with the same
// star-shape absorption as the verified one-shot builder).
func collectCavityStatic(p int, coords []geom.Point,
	store []workTri, idx *spatialIndex, edgeAdj map[edge][]int) (map[int]bool, *geom.Error) {

	q := coords[p]
	bad := make(map[int]bool, 8)
	var queue []int
	for _, id := range idx.candidates(q) {
		t := store[id]
		if t.alive && !bad[id] &&
			geom.InCircleSign(coords[t.a], coords[t.b], coords[t.c], q) > 0 {
			bad[id] = true
			queue = append(queue, id)
		}
	}
	if len(bad) == 0 {
		return nil, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "ghost frame rebuild: insertion cavity empty",
		}
	}
	for len(queue) > 0 {
		id := queue[0]
		queue = queue[1:]
		t := store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			var other int = -1
			for _, z := range edgeAdj[e] {
				if z != id && store[z].alive {
					other = z
					break
				}
			}
			if other < 0 || bad[other] {
				continue
			}
			ot := store[other]
			if geom.InCircleSign(coords[ot.a], coords[ot.b], coords[ot.c], q) > 0 {
				bad[other] = true
				queue = append(queue, other)
				continue
			}
			u, v := e.u, e.v
			w := thirdVertex(t, u, v)
			if geom.OrientSign(coords[u], coords[v], coords[w]) < 0 {
				u, v = v, u
			}
			if geom.OrientSign(coords[u], coords[v], q) < 0 {
				bad[other] = true
				queue = append(queue, other)
			}
		}
	}
	return bad, nil
}

// ---- hull maintenance ----------------------------------------------------

func (m *Mesh) recomputeHull() *geom.Error {
	hullSlots, herr := m.boundaryRingFromStore()
	if herr != nil {
		return herr
	}
	m.hull = externalSlots(hullSlots)
	return nil
}

// boundaryRingFromStore chains the one-owner edges of the living real
// triangle soup into the CCW hull ring (coordinate slots).
func (m *Mesh) boundaryRingFromStore() ([]int, *geom.Error) {
	has := make(map[[2]int]bool, 3*m.nReal+4)
	for _, t := range m.store {
		if !t.alive || isGhost(t) {
			continue
		}
		for _, e := range [3][2]int{{t.a, t.b}, {t.b, t.c}, {t.c, t.a}} {
			has[e] = true
		}
	}
	next := make(map[int]int, len(has)/2)
	for e := range has {
		if !has[[2]int{e[1], e[0]}] {
			if other, collision := next[e[0]]; collision && other != e[1] {
				return nil, &geom.Error{
					Code:    geom.ErrDegenerate,
					Message: "mesh boundary vertex has two outgoing boundary edges",
				}
			}
			next[e[0]] = e[1]
		}
	}
	if len(next) == 0 {
		return nil, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "mesh has no boundary edges",
		}
	}
	var start int
	for u := range next {
		start = u
		break
	}
	ring := make([]int, 0, len(next))
	cur := start
	for {
		ring = append(ring, cur)
		nxt, ok := next[cur]
		if !ok {
			return nil, &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh boundary is not a closed ring",
			}
		}
		cur = nxt
		if cur == start {
			break
		}
		if len(ring) > len(next) {
			return nil, &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh boundary walk did not close",
			}
		}
	}
	return ring, nil
}
