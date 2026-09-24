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
// The construction is the stateful Mesh run once over the full point
// set: opening the super triangle, per-point "dig the cavity / fan the
// boundary" insertion (see mesh.go) and shaving the ghost triangles are
// exactly the same code path incremental sessions use — the one-shot
// interface simply never performs a second operation on the mesh.
func Build(points []geom.Point) (*Result, *geom.Error) {
	m, gerr := NewMesh(points)
	if gerr != nil {
		return nil, gerr
	}
	return m.Snapshot(), nil
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

// superTriangleFromBounds returns a strictly CCW triangle that strictly
// contains the axis-aligned box of the given bounds with a wide margin,
// so that no real point can escape it even under the exact-predicate
// filter.
func superTriangleFromBounds(minX, minY, maxX, maxY float64) (geom.Point, geom.Point, geom.Point) {
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
