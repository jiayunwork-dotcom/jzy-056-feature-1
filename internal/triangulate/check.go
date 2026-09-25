package triangulate

import (
	"math"

	"delaunaysvc/internal/geom"
)

// locallyConsistent is the cheap, mesh-internal legality gate run after
// every incremental edit before the result is accepted. It verifies the
// exact postconditions the hard requirement pins:
//
//   - every living real triangle is strictly CCW with no ghost vertex;
//   - each interior edge has exactly two living owners and each hull
//     edge exactly one (a manifold, gap- and overlap-free soup);
//   - the local Delaunay condition across every interior edge — the
//     opposite vertex is not strictly inside the circumcircle;
//   - the living triangles cover the convex hull: the sum of triangle
//     areas equals the independently computed convex-hull area.
//
// All predicate work uses the exact internal coordinates. The global
// empty-circle condition is equivalent (for a conforming triangulation)
// to the local across-edge condition used here, so this is linear in the
// current mesh size. Returning false forces the edit onto its safe
// whole-mesh rebuild path, which always yields a legal net.
func (m *Mesh) locallyConsistent() bool {
	res, gerr := m.Snapshot()
	if gerr != nil || len(res.Triangles) == 0 {
		return false
	}
	pp := res.Points
	wp := res.WorkPoints()
	n := len(m.Points)

	owners := make(map[edge][]int, 3*len(res.Triangles))
	for ti, t := range res.Triangles {
		for _, v := range [3]int{t.A, t.B, t.C} {
			if v < 0 || v >= n {
				return false
			}
		}
		if geom.OrientSign(wp[t.A], wp[t.B], wp[t.C]) <= 0 {
			return false
		}
		for _, e := range [3]edge{newEdge(t.A, t.B), newEdge(t.B, t.C), newEdge(t.C, t.A)} {
			owners[e] = append(owners[e], ti)
		}
	}

	var sum float64
	for _, t := range res.Triangles {
		sum += geom.SignedArea(pp[t.A], pp[t.B], pp[t.C])
	}
	for _, o := range owners {
		if len(o) < 1 || len(o) > 2 {
			return false
		}
	}

	// Local empty-circumcircle check across every interior edge.
	for e, o := range owners {
		if len(o) != 2 {
			continue
		}
		t1 := res.Triangles[o[0]]
		t2 := res.Triangles[o[1]]
		opp1 := otherVertex(t1, e.u, e.v)
		opp2 := otherVertex(t2, e.u, e.v)
		if geom.InCircleSign(wp[t1.A], wp[t1.B], wp[t1.C], wp[opp2]) > 0 {
			return false
		}
		if geom.InCircleSign(wp[t2.A], wp[t2.B], wp[t2.C], wp[opp1]) > 0 {
			return false
		}
	}

	// Hull coverage in the caller coordinates (the jittered work frame
	// may move an on-edge micro-slice point by ~1e-12*span; topology and
	// client-visible coverage are judged on the original points).
	hull := geom.ConvexHull(pp)
	hullArea := geom.PolygonArea(pp, hull)
	if hullArea <= 0 || sum <= 0 {
		return false
	}
	if math.Abs(sum-hullArea)/math.Max(sum, hullArea) > 1e-9 {
		return false
	}

	// The one-owner edges must be exactly the hull edges.
	hullSet := make(map[edge]bool, len(hull))
	for i := range hull {
		hullSet[newEdge(hull[i], hull[(i+1)%len(hull)])] = true
	}
	for e, o := range owners {
		if (len(o) == 1) != hullSet[e] {
			return false
		}
	}
	return true
}
