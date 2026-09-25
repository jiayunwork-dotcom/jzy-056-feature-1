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
//
// It is a thin, stateless façade over the long-lived stateful Mesh: the
// mesh is constructed with the exact same local cavity routine the
// incremental session layer later reuses for its edits, so the one-shot
// path and the stateful path cannot drift apart. A fresh mesh, one
// snapshot, then it is discarded.
func Build(points []geom.Point) (*Result, *geom.Error) {
	m := &Mesh{}
	// Order-robust construction: the natural order virtually always
	// succeeds; at an order-dependent near-collinear sliver an alternate
	// deterministic insertion order is used so the returned triangulation
	// still passes the empty-circle and hull-coverage guarantees.
	if gerr := m.rebuildRobust(points, points[0]); gerr != nil {
		return nil, gerr
	}
	res, serr := m.Snapshot()
	if serr != nil {
		return nil, serr
	}
	return res, nil
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

// superOfBounds returns a strictly CCW super-triangle that contains the
// box (minX,minY)-(maxX,maxY) in its interior with a wide 100x margin,
// so no real point can escape it even under the exact-predicate filter.
// Both the initial mesh construction and the local super-triangle
// expansion of an editing session derive the virtual frame from here,
// guaranteeing identical geometry.
func superOfBounds(minX, minY, maxX, maxY float64) (geom.Point, geom.Point, geom.Point) {
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
