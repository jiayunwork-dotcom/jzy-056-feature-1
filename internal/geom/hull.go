package geom

import (
	"sort"
)

// ConvexHull returns the convex hull of ps as indices into ps in
// counter-clockwise order. Collinear points lying on a hull edge are
// retained: they are real input points and therefore real mesh vertices
// (the triangulation's hull-edge count, used in Euler checks, must count
// them). Every non-hull point lies strictly inside.
//
// This is Andrew's monotone-chain sweep with the strict pop condition
// (pop while the turn is clockwise); zero-turn edges are kept, which is
// exactly what retains the collinear boundary points without duplicating
// the chain endpoints.
func ConvexHull(ps []Point) []int {
	idx := make([]int, len(ps))
	for i := range idx {
		idx[i] = i
	}
	sort.Slice(idx, func(i, j int) bool {
		if ps[idx[i]].X != ps[idx[j]].X {
			return ps[idx[i]].X < ps[idx[j]].X
		}
		return ps[idx[i]].Y < ps[idx[j]].Y
	})

	// Lower hull: pop while the last turn is clockwise.
	lower := make([]int, 0, len(idx))
	for _, k := range idx {
		for len(lower) >= 2 &&
			OrientSign(ps[lower[len(lower)-2]], ps[lower[len(lower)-1]], ps[k]) < 0 {
			lower = lower[:len(lower)-1]
		}
		lower = append(lower, k)
	}

	// Upper hull, scanning in reverse.
	upper := make([]int, 0, len(idx))
	for i := len(idx) - 1; i >= 0; i-- {
		k := idx[i]
		for len(upper) >= 2 &&
			OrientSign(ps[upper[len(upper)-2]], ps[upper[len(upper)-1]], ps[k]) < 0 {
			upper = upper[:len(upper)-1]
		}
		upper = append(upper, k)
	}

	// The last element of each chain equals the first element of the
	// other; drop both copies.
	return append(lower[:len(lower)-1], upper[:len(upper)-1]...)
}

// PolygonArea returns the unsigned area of the polygon given by the
// provided point indices (shoelace formula).
func PolygonArea(ps []Point, ring []int) float64 {
	var s2 float64
	n := len(ring)
	for i := 0; i < n; i++ {
		p, q := ps[ring[i]], ps[ring[(i+1)%n]]
		s2 += p.X*q.Y - q.X*p.Y
	}
	if s2 < 0 {
		s2 = -s2
	}
	return s2 * 0.5
}
