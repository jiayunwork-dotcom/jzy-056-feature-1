// Package sample provides the service's built-in, always-available
// reference case: a regular 4x4 grid (16 points) whose interior points
// carry fixed, deterministic perturbations. It is used both as a demo
// payload and as the fixture whose triangle count is pinned by the
// Euler relation.
package sample

import (
	"math"

	"delaunaysvc/internal/geom"
)

// GridN is the side length of the square sample grid.
const GridN = 4

// Size is the number of sample points.
const Size = GridN * GridN // 16

// hullEdgeCount is the number of convex-hull edges of the unperturbed
// square grid; the boundary points are kept fixed by Grid so that the
// perturbations do not move the hull.
const hullEdgeCount = 4 * (GridN - 1) // 12

// ExpectedTriangles is the triangle count dictated by the planar
// Euler relation for a triangulation of Size points whose hull has
// hullEdgeCount edges:
//
//	T = 2N - H - 2 = 32 - 12 - 2 = 18.
const ExpectedTriangles = 2*Size - hullEdgeCount - 2 // 18

// ExpectedHullEdges is the expected convex-hull edge count.
const ExpectedHullEdges = hullEdgeCount

// Grid returns the perturbed regular grid. Boundary points stay exactly
// on the square; interior points are nudged by fixed fractions of the
// cell size (a deterministic, seed-free perturbation), so the payload
// is bit-for-bit reproducible across runs.
func Grid() []geom.Point {
	ps := make([]geom.Point, 0, Size)
	for j := 0; j < GridN; j++ {
		for i := 0; i < GridN; i++ {
			x, y := float64(i), float64(j)
			if i > 0 && i < GridN-1 && j > 0 && j < GridN-1 {
				x += 0.03 * math.Sin(float64(i)*1.7+float64(j))
				y += 0.03 * math.Cos(float64(j)*1.3+float64(i))
			}
			ps = append(ps, geom.Point{X: x, Y: y})
		}
	}
	return ps
}
