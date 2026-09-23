// Package voronoi derives the Voronoi dual of a Delaunay triangulation:
//
//   - every Delaunay triangle becomes a Voronoi vertex at the triangle's
//     exact circumcenter;
//   - every interior edge shared by two triangles becomes a finite
//     Voronoi edge connecting the two circumcenters;
//   - every convex-hull edge, which has only one adjacent triangle,
//     becomes an unbounded Voronoi ray starting at that circumcenter and
//     pointing outward from the hull.
package voronoi

import (
	"math"
	"sort"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
)

// Vertex is a Voronoi vertex: the circumcenter of one Delaunay triangle.
type Vertex struct {
	// Index is the index of the generating triangle.
	Index int
	Point geom.Point
}

// FiniteEdge is a Voronoi edge shared by two Delaunay triangles.
type FiniteEdge struct {
	// U, V are Voronoi vertex indices (= Delaunay triangle indices),
	// stored in ascending order.
	U int
	V int
}

// Ray is an unbounded Voronoi edge dual to a convex-hull edge.
type Ray struct {
	// Origin is the Voronoi vertex index (= the sole adjacent
	// Delaunay triangle index).
	Origin int
	// Direction is the unit outward direction of the ray.
	Direction geom.Point
	// HullU, HullV are the hull edge endpoints (input point indices),
	// oriented CCW around the hull.
	HullU int
	HullV int
}

// Diagram is the full Voronoi dual.
type Diagram struct {
	Vertices []Vertex
	Edges    []FiniteEdge
	Rays     []Ray
}

type edgeKey struct{ u, v int }

// Build constructs the Voronoi dual of a finished triangulation.
func Build(t *triangulate.Result) *Diagram {
	// All dual geometry is computed in the triangulation's internal
	// coordinate frame, where the mesh is exactly Delaunay; circumcenters
	// are then shifted back by +Origin to the caller's frame. Because
	// Circumcircle solves from edge vectors relative to one vertex, that
	// shift is exactly the inverse of the internal translation.
	wp := t.WorkPoints()
	verts := make([]Vertex, len(t.Triangles))
	for i, tr := range t.Triangles {
		lc := geom.Circumcenter(wp[tr.A], wp[tr.B], wp[tr.C])
		verts[i] = Vertex{
			Index: i,
			Point: geom.Point{X: lc.X + t.Origin.X, Y: lc.Y + t.Origin.Y},
		}
	}

	// Map every edge to the (one or two) adjacent triangles. A hull edge
	// has exactly one owner; an interior edge has two.
	type adjacency struct{ t1, t2 int }
	edgeTris := make(map[edgeKey]*adjacency, 3*len(t.Triangles))
	addEdge := func(u, v, ti int) {
		// Normalize: the adjacent triangle stores the shared edge in the
		// opposite direction, so both directions must map to one key.
		if u > v {
			u, v = v, u
		}
		k := edgeKey{u, v}
		a := edgeTris[k]
		if a == nil {
			a = &adjacency{t1: ti}
			edgeTris[k] = a
		} else {
			a.t2 = ti
		}
	}
	for i, tr := range t.Triangles {
		addEdge(tr.A, tr.B, i)
		addEdge(tr.B, tr.C, i)
		addEdge(tr.C, tr.A, i)
	}

	// Convex-hull edges keyed by the normalized undirected edge (so the
	// key matches the adjacency map); each value keeps the edge's CCW
	// orientation around the hull ring.
	hullEdges := make(map[edgeKey][2]int)
	h := t.Hull
	for i := 0; i < len(h); i++ {
		u, v := h[i], h[(i+1)%len(h)]
		k := edgeKey{u, v}
		if k.u > k.v {
			k = edgeKey{v, u}
		}
		hullEdges[k] = [2]int{u, v}
	}

	var finite []FiniteEdge
	var rays []Ray
	for k, a := range edgeTris {
		if hv, isHull := hullEdges[k]; isHull {
			// Unbounded dual edge. hv -> next is CCW, so the outward
			// normal of hu -> hv is (dy, -dx) with d = hv - hu.
			pu := t.Points[hv[0]]
			pv := t.Points[hv[1]]
			dx := pv.X - pu.X
			dy := pv.Y - pu.Y
			dir := geom.Point{X: dy, Y: -dx}
			l := math.Hypot(dir.X, dir.Y)
			if l > 0 {
				dir.X /= l
				dir.Y /= l
			}
			rays = append(rays, Ray{
				Origin:    a.t1,
				Direction: dir,
				HullU:     hv[0],
				HullV:     hv[1],
			})
			continue
		}
		u, v := a.t1, a.t2
		if u > v {
			u, v = v, u
		}
		finite = append(finite, FiniteEdge{U: u, V: v})
	}

	sort.Slice(finite, func(i, j int) bool {
		if finite[i].U != finite[j].U {
			return finite[i].U < finite[j].U
		}
		return finite[i].V < finite[j].V
	})
	sort.Slice(rays, func(i, j int) bool {
		if rays[i].Origin != rays[j].Origin {
			return rays[i].Origin < rays[j].Origin
		}
		if rays[i].HullU != rays[j].HullU {
			return rays[i].HullU < rays[j].HullU
		}
		return rays[i].HullV < rays[j].HullV
	})

	return &Diagram{Vertices: verts, Edges: finite, Rays: rays}
}
