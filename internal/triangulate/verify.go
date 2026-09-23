package triangulate

import "delaunaysvc/internal/geom"

// Violation describes one failure of the local Delaunay condition: the
// opposite vertex of the triangle across an interior edge lies strictly
// inside the reported triangle's circumcircle.
type Violation struct {
	TriangleIndex int `json:"triangle_index"`
	PointIndex    int `json:"point_index"`
	EdgeU         int `json:"edge_u"`
	EdgeV         int `json:"edge_v"`
}

// IsDelaunay verifies the empty-circumcircle property in linear time.
//
// For a conforming triangulation the global condition ("no input point is
// strictly inside any triangle circumcircle") is equivalent to the local
// condition across every interior edge: the vertex opposite that edge in
// the adjacent triangle must not lie strictly inside the triangle's
// circumdisk (and symmetrically). Hull edges have no neighbour and need
// no check. Each interior edge therefore needs one in-circle test.
func (r *Result) IsDelaunay() (bool, []Violation) {
	type owner struct {
		t1, t2 int
		has2   bool
	}
	edgeOwners := make(map[edge]*owner, 3*len(r.Triangles))
	for ti, t := range r.Triangles {
		for _, e := range [3]edge{newEdge(t.A, t.B), newEdge(t.B, t.C), newEdge(t.C, t.A)} {
			o := edgeOwners[e]
			if o == nil {
				o = &owner{t1: ti}
				edgeOwners[e] = o
			} else {
				o.t2, o.has2 = ti, true
			}
		}
	}

	var violations []Violation
	for e, o := range edgeOwners {
		if !o.has2 {
			continue // hull edge: no opposite triangle
		}
		t1 := r.Triangles[o.t1]
		t2 := r.Triangles[o.t2]
		opp2 := otherVertex(t2, e.u, e.v)
		opp1 := otherVertex(t1, e.u, e.v)

		if geom.InCircleSign(r.work[t1.A], r.work[t1.B], r.work[t1.C], r.work[opp2]) > 0 {
			violations = append(violations, Violation{
				TriangleIndex: o.t1, PointIndex: opp2, EdgeU: e.u, EdgeV: e.v,
			})
		}
		if geom.InCircleSign(r.work[t2.A], r.work[t2.B], r.work[t2.C], r.work[opp1]) > 0 {
			violations = append(violations, Violation{
				TriangleIndex: o.t2, PointIndex: opp1, EdgeU: e.u, EdgeV: e.v,
			})
		}
	}
	return len(violations) == 0, violations
}

// otherVertex returns the vertex of t that is neither u nor v; u and v
// are assumed to be two vertices of t.
func otherVertex(t geom.Triangle, u, v int) int {
	switch {
	case t.A != u && t.A != v:
		return t.A
	case t.B != u && t.B != v:
		return t.B
	default:
		return t.C
	}
}
