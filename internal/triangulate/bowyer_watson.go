package triangulate

import (
	"delaunaysvc/internal/geom"
)

// edge is an undirected edge, always stored with the smaller index first
// so that shared edges between triangles get identical map keys.
type edge struct {
	u, v int
}

func newEdge(i, j int) edge {
	if i < j {
		return edge{i, j}
	}
	return edge{j, i}
}

// sortEdges orders edges deterministically so triangle creation order is
// independent of Go's randomized map iteration.
func sortEdges(es []edge) {
	for i := 1; i < len(es); i++ {
		for j := i; j > 0 && (es[j].u < es[j-1].u || (es[j].u == es[j-1].u && es[j].v < es[j-1].v)); j-- {
			es[j], es[j-1] = es[j-1], es[j]
		}
	}
}

// Build runs Bowyer-Watson over an already validated point set and
// returns the triangulation in the original coordinates.
func Build(points []geom.Point) (*Result, *geom.Error) {
	n := len(points)

	// Work in coordinates translated by -points[0]. The predicate sign
	// is translation invariant; doing the arithmetic near the origin
	// additionally keeps magnitudes comparable so that, e.g., translating
	// the set by (1e8, -1e8) cannot change the topology through round-off.
	origin := points[0]
	local := make([]geom.Point, n)
	for i, p := range points {
		local[i] = geom.Point{X: p.X - origin.X, Y: p.Y - origin.Y}
	}
	// Deterministic, index-dependent symbolic perturbation. Exactly
	// co-circular or nearly co-circular configurations make the strict
	// in-circle cavity wrap an existing vertex; nudging every point by a
	// distinct, scale-free infinitesimal (applied in the translated
	// frame, after translation is removed) puts the set in general
	// position. The perturbation is a pure function of the index, so the
	// same request always yields the same triangulation (no flip
	// oscillation) and translating the request leaves the perturbation
	// and hence the topology unchanged.
	jitterToGeneralPosition(local)

	s0, s1, s2 := superTriangle(local)
	coords := make([]geom.Point, 0, n+3)
	coords = append(coords, local...)
	coords = append(coords, s0, s1, s2)
	i0, i1, i2 := n, n+1, n+2

	// The spatial index is initialized in the translated coordinate
	// frame: every triangle coordinate it ever sees is local, so its
	// origin and cell keys must be local too. Mixing the two frames
	// silently shifts every bucket key.
	lMinX, lMinY := local[0].X, local[0].Y
	lMaxX, lMaxY := lMinX, lMinY
	for _, p := range local[1:] {
		lMinX = mathMin(lMinX, p.X)
		lMaxX = mathMax(lMaxX, p.X)
		lMinY = mathMin(lMinY, p.Y)
		lMaxY = mathMax(lMaxY, p.Y)
	}
	idx := newSpatialIndex(lMinX, lMinY, lMaxX-lMinX, lMaxY-lMinY, n)

	// Append-only triangle store with stable IDs; dead triangles keep
	// their slot with alive == false.
	store := make([]workTri, 0, 2*n+4)
	store = append(store, workTri{a: i0, b: i1, c: i2, alive: true})
	store[0] = idx.insertTriangle(0, store[0], coords)

	// Incremental undirected-edge adjacency over living triangles.
	edgeAdj := make(map[edge][]int, 6*n+4)
	addTriangleAdj := func(id int, t workTri) {
		edgeAdj[newEdge(t.a, t.b)] = append(edgeAdj[newEdge(t.a, t.b)], id)
		edgeAdj[newEdge(t.b, t.c)] = append(edgeAdj[newEdge(t.b, t.c)], id)
		edgeAdj[newEdge(t.c, t.a)] = append(edgeAdj[newEdge(t.c, t.a)], id)
	}
	removeTriangleAdj := func(id int, t workTri) {
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			ns := edgeAdj[e][:0]
			for _, x := range edgeAdj[e] {
				if x != id {
					ns = append(ns, x)
				}
			}
			edgeAdj[e] = ns
		}
	}
	addTriangleAdj(0, store[0])

	// Insert points in input order. The input order is fixed by the
	// request, so the result is deterministic for a given request.
	for p := 0; p < n; p++ {
		bad, gerr := collectCavity(p, coords, store, idx, edgeAdj)
		if gerr != nil {
			return nil, gerr
		}

		// Boundary edges of the cavity, fan-connected below.
		var boundary []edge
		boundarySet := make(map[edge]bool, 8)
		for id := range bad {
			t := store[id]
			for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
				if boundarySet[e] {
					continue
				}
				ns := edgeAdj[e]
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
		// Stable creation order keeps triangle identities (and hence all
		// indices, adjacency and the wire response) independent of map
		// iteration order.
		sortEdges(boundary)

		// Delete the cavity from adjacency and the spatial index.
		for id := range bad {
			t := store[id]
			removeTriangleAdj(id, t)
			idx.removeIn(id, t)
			t.alive = false
			store[id] = t
		}

		// Fan p over every cavity boundary edge, each new triangle
		// oriented strictly CCW. If p lies exactly on a boundary edge
		// (possible when a later input point lands on a hull edge), that
		// edge produces no triangle; the boundary-ring walk keeps the
		// mesh closed and p becomes a boundary vertex.
		q := coords[p]
		for _, e := range boundary {
			u, v := e.u, e.v
			nt := workTri{a: p, b: u, c: v, alive: true}
			switch geom.OrientSign(q, coords[u], coords[v]) {
			case -1:
				nt.b, nt.c = v, u
			case 0:
				continue
			}
			id := len(store)
			store = append(store, workTri{})
			nt = idx.insertTriangle(id, nt, coords)
			store[id] = nt
			addTriangleAdj(id, nt)
		}
	}

	// Shave off every ghost triangle: anything touching one of the
	// three virtual vertices must never reach a caller, or hull
	// statistics (area, edge count) are polluted.
	final := make([]geom.Triangle, 0, 2*n)
	for _, t := range store {
		if !t.alive || t.a >= n || t.b >= n || t.c >= n {
			continue
		}
		final = append(final, geom.Triangle{A: t.a, B: t.b, C: t.c})
	}
	canonicalize(final)

	// Derive the hull from the finished mesh boundary. Doing this here
	// (instead of an independent sweep that can round near-collinear
	// boundary points differently) keeps the reported hull in exact
	// agreement with the triangle soup's one-owner edges.
	hull, herr := boundaryRing(final)
	if herr != nil {
		return nil, herr
	}

	res := &Result{
		Points:    points,
		work:      coords[:n:n],
		Triangles: final,
		Hull:      hull,
		Origin:    origin,
	}
	return res, nil
}

// collectCavity returns the IDs of every triangle to delete when p is
// inserted.
//
// The spatial index supplies the triangles filed in p's grid cell plus
// the always-tested oversize triangles; the triangle containing p is
// always among them. The set of triangles whose circumdisk contains p
// (the on-circle case included) is edge-connected in a valid Delaunay
// mesh, so a breadth-first walk over living-triangle adjacency visits
// the whole cavity doing only local work.
func collectCavity(p int, coords []geom.Point,
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
			Message: "insertion cavity empty; point set numerically degenerate",
		}
	}

	for len(queue) > 0 {
		id := queue[0]
		queue = queue[1:]
		t := store[id]
		for _, e := range [3]edge{newEdge(t.a, t.b), newEdge(t.b, t.c), newEdge(t.c, t.a)} {
			var other int = -1
			for _, x := range edgeAdj[e] {
				if x != id && store[x].alive {
					other = x
					break
				}
			}
			if other < 0 || bad[other] {
				continue
			}
			ot := store[other]
			// Strict circumdisk containment...
			if geom.InCircleSign(coords[ot.a], coords[ot.b], coords[ot.c], q) > 0 {
				bad[other] = true
				queue = append(queue, other)
				continue
			}
			// ...or pull in a non-bad neighbour across an edge that is
			// not visible from p. With exact predicates the strict
			// in-circle cavity can still be non-star-shaped through a
			// super-triangle / co-circular configuration near a hull
			// insertion; iterating this to a fixpoint makes the union
			// star-shaped w.r.t. p, which is exactly what the fan step
			// needs. The absorbed real triangles are restored by the new
			// fan triangles and the result remains Delaunay.
			if !boundaryEdgeVisible(id, e, q, coords, store) {
				bad[other] = true
				queue = append(queue, other)
			}
		}
	}
	return bad, nil
}

// boundaryEdgeVisible reports whether the boundary edge e of cavity
// triangle cavityID is visible from p, i.e. p lies on the cavity
// (interior) side of the oriented edge. Collinearity is treated as
// visible; the fan step handles the on-edge case.
func boundaryEdgeVisible(cavityID int, e edge, p geom.Point, coords []geom.Point, store []workTri) bool {
	u, v := e.u, e.v
	ct := store[cavityID]
	w := thirdVertex(ct, u, v)
	if geom.OrientSign(coords[u], coords[v], coords[w]) < 0 {
		u, v = v, u
	}
	return geom.OrientSign(coords[u], coords[v], p) >= 0
}

// thirdVertex returns the vertex of t different from u and v.
func thirdVertex(t workTri, u, v int) int {
	switch {
	case t.a != u && t.a != v:
		return t.a
	case t.b != u && t.b != v:
		return t.b
	default:
		return t.c
	}
}

// superTriangle returns a strictly CCW triangle that contains every
// point of ps in its interior with a wide margin, so that no input
// point can escape it even under the exact-predicate filter.
func superTriangle(ps []geom.Point) (geom.Point, geom.Point, geom.Point) {
	minX, minY := ps[0].X, ps[0].Y
	maxX, maxY := minX, minY
	for _, p := range ps[1:] {
		minX = mathMin(minX, p.X)
		maxX = mathMax(maxX, p.X)
		minY = mathMin(minY, p.Y)
		maxY = mathMax(maxY, p.Y)
	}
	span := mathMax(maxX-minX, maxY-minY)
	if span == 0 {
		span = 1
	}
	d := 100 * span
	mx := (minX + maxX) * 0.5
	my := (minY + maxY) * 0.5

	bottom := geom.Point{X: mx, Y: my - d}
	right := geom.Point{X: mx + d, Y: my + d}
	left := geom.Point{X: mx - d, Y: my + d}
	// (bottom, right, left) is CCW: the apex edge runs right-to-left.
	return bottom, right, left
}

func mathMin(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func mathMax(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}
