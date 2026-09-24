package triangulate

import (
	"delaunaysvc/internal/geom"
)

// GlobalCheck validates a triangulation against the acceptance criteria
// used for incrementally maintained meshes:
//
//  1. GLOBAL empty-circumcircle: no point of the set lies strictly inside
//     the circumcircle of any triangle. This is stronger than the local
//     across-edge check IsDelaunay performs and is exact (it does not
//     assume the mesh is Delaunay to localize the test).
//  2. Conforming coverage: every triangle is strictly CCW, every interior
//     edge has exactly two owners and every hull edge exactly one, the
//     one-owner edges are precisely the hull ring, the Euler relations
//     hold, and the triangle area sum equals the hull area — i.e. the
//     triangles tile the current convex hull without overlap or gaps.
//
// It returns an empty string when the mesh is legal, otherwise a
// description of the first failure found.
func GlobalCheck(r *Result) string {
	wp := r.work
	n := len(r.Points)

	// All indices valid and triangles strictly CCW in the internal frame.
	for _, tr := range r.Triangles {
		for _, v := range []int{tr.A, tr.B, tr.C} {
			if v < 0 || v >= n {
				return "triangle references an out-of-range vertex"
			}
		}
		if geom.OrientSign(wp[tr.A], wp[tr.B], wp[tr.C]) <= 0 {
			return "triangle is not strictly counter-clockwise"
		}
	}

	// Global empty circumcircle.
	for _, tr := range r.Triangles {
		for p := 0; p < n; p++ {
			if p == tr.A || p == tr.B || p == tr.C {
				continue
			}
			if geom.InCircleSign(wp[tr.A], wp[tr.B], wp[tr.C], wp[p]) > 0 {
				return "point lies strictly inside a triangle circumcircle"
			}
		}
	}

	// Edge ownership: interior edges exactly two, hull edges exactly one.
	owners := make(map[edge]int, 3*len(r.Triangles))
	for _, tr := range r.Triangles {
		for _, e := range [3]edge{
			newEdge(tr.A, tr.B), newEdge(tr.B, tr.C), newEdge(tr.C, tr.A),
		} {
			owners[e]++
			if owners[e] > 2 {
				return "an edge is shared by more than two triangles"
			}
		}
	}

	hullEdges := make(map[edge]bool, len(r.Hull))
	for i := 0; i < len(r.Hull); i++ {
		hullEdges[newEdge(r.Hull[i], r.Hull[(i+1)%len(r.Hull)])] = true
	}
	boundaryCount := 0
	for e, c := range owners {
		if c == 1 {
			boundaryCount++
			if !hullEdges[e] {
				return "a boundary edge is not part of the hull ring"
			}
		}
	}
	if boundaryCount != len(r.Hull) {
		return "number of boundary edges does not match the hull ring"
	}
	for e := range hullEdges {
		if owners[e] != 1 {
			return "a hull edge is not a single-owner boundary edge"
		}
	}

	// Euler: T - E + N = 1, and T = 2N - H - 2.
	T, E, N, H := len(r.Triangles), len(owners), n, len(r.Hull)
	if T-E+N != 1 {
		return "Euler relation T-E+N=1 violated"
	}
	if T != 2*N-H-2 {
		return "Euler relation T=2N-H-2 violated"
	}

	// Coverage by area.
	sum, area := r.AreaSum(), r.HullArea()
	if !relCloseSum(sum, area) {
		return "triangle area sum does not equal hull area"
	}
	return ""
}

func relCloseSum(a, b float64) bool {
	den := a
	if b > den {
		den = b
	}
	if den == 0 {
		return true
	}
	d := a - b
	if d < 0 {
		d = -d
	}
	return d/den <= 1e-9
}
