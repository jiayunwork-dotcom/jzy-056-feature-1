// Package triangulate implements the incremental Bowyer-Watson
// Delaunay triangulation of a validated 2D point set.
//
// Construction runs in coordinates translated by -Origin so that the
// geometry kernel operates near the origin regardless of where the
// caller's points live; the translation is rigid, so topology is
// independent of it. Three virtual super-triangle vertices are appended
// during construction and every triangle touching them is removed before
// the result is returned.
package triangulate

import (
	"sort"

	"delaunaysvc/internal/geom"
)

// Result is a finished Delaunay triangulation. Triangles are given as
// CCW index triples shared by both coordinate frames:
//
//   - Points are the caller's original coordinates, used for the wire
//     response;
//   - work are the internal coordinates (rigidly translated and, when
//     needed to break exact co-circularity, deterministically nudged
//     into general position). All geometric predicates, the empty-circle
//     verification, area accounting and Voronoi circumcenters operate on
//     work, where the triangulation is exactly Delaunay.
type Result struct {
	// Points is the canonical, order-preserving input point set.
	Points []geom.Point
	// work are the coordinates the kernel actually triangulated.
	work []geom.Point
	// Triangles are CCW triangles; none of their indices refers to a
	// virtual super-triangle vertex.
	Triangles []geom.Triangle
	// Hull are the boundary vertex indices of the mesh in CCW order,
	// including input points lying (numerically) on a hull edge. It is
	// extracted from the finished mesh rather than computed
	// independently, so topology statistics can never disagree about
	// near-collinear boundary points.
	Hull []int
	// Origin is the vector subtracted from every point during
	// construction (always Points[0]). Voronoi consumers add it back to
	// centers computed in internal coordinates.
	Origin geom.Point
}

// WorkPoints returns the internal coordinate slice on which the
// triangulation is exactly Delaunay. Downstream geometry (the
// independent empty-circle check and the Voronoi dual) must use these.
func (r *Result) WorkPoints() []geom.Point { return r.work }

// TriangleCount returns the number of triangles.
func (r *Result) TriangleCount() int { return len(r.Triangles) }

// HullEdgeCount returns the number of edges on the convex hull, counting
// collinear hull vertices as edge endpoints.
func (r *Result) HullEdgeCount() int { return len(r.Hull) }

// AreaSum returns the sum of the triangle areas. Property: for a
// conforming triangulation this equals the hull area.
func (r *Result) AreaSum() float64 {
	var sum float64
	for _, t := range r.Triangles {
		sum += geom.Area2(r.work[t.A], r.work[t.B], r.work[t.C]) * 0.5
	}
	return sum
}

// HullArea returns the area enclosed by the mesh boundary (== the
// convex-hull area).
func (r *Result) HullArea() float64 {
	return geom.PolygonArea(r.work, r.Hull)
}

// IsCCW reports whether every stored triangle is counter-clockwise.
func (r *Result) IsCCW() bool {
	for _, t := range r.Triangles {
		if geom.OrientSign(r.work[t.A], r.work[t.B], r.work[t.C]) <= 0 {
			return false
		}
	}
	return true
}

// canonicalize sorts triangles lexicographically for deterministic
// output and stable diffs; each triangle is already CCW.
func canonicalize(ts []geom.Triangle) {
	sort.Slice(ts, func(i, j int) bool {
		if ts[i].A != ts[j].A {
			return ts[i].A < ts[j].A
		}
		if ts[i].B != ts[j].B {
			return ts[i].B < ts[j].B
		}
		return ts[i].C < ts[j].C
	})
}
