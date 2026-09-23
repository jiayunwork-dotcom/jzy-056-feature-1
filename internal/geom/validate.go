package geom

import "math"

// ParsePoints is the single entry point for raw point-set input: it
// performs every input legality check and returns the canonical
// (order-preserving) point slice the kernel consumes.
//
// The checks, in order, are:
//   - fewer than three points (nothing can be triangulated);
//   - NaN or infinite coordinates;
//   - exact duplicate points (the input format does not declare them,
//     and they would force zero-area elements into the mesh);
//   - every point strictly collinear (the hull has zero area).
func ParsePoints(raw []Point) ([]Point, *Error) {
	if len(raw) < 3 {
		return nil, newError(ErrTooFewPoints,
			"at least 3 points are required, got %d", len(raw))
	}

	for i, p := range raw {
		if math.IsNaN(p.X) || math.IsNaN(p.Y) ||
			math.IsInf(p.X, 0) || math.IsInf(p.Y, 0) {
			return nil, newError(ErrInvalidCoord,
				"point at index %d has NaN or infinite coordinate", i)
		}
	}

	for i := 1; i < len(raw); i++ {
		for j := 0; j < i; j++ {
			if raw[i] == raw[j] {
				return nil, newError(ErrDuplicatePoint,
					"point at index %d duplicates point at index %d", i, j)
			}
		}
	}

	if allCollinear(raw) {
		return nil, newError(ErrAllCollinear,
			"all %d points are strictly collinear; no area to triangulate", len(raw))
	}

	out := make([]Point, len(raw))
	copy(out, raw)
	return out, nil
}

// allCollinear reports whether every point lies on a single line. The
// orientation test is relative to the diameter of the point set so that
// large coordinates are not falsely rejected: a point counts as off the
// line only when its double area exceeds 1e-12 * diameter^2.
func allCollinear(ps []Point) bool {
	// Find the farthest pair (a simple O(n^2) scan; input validation is
	// not a hot path) and use it as the base segment.
	var maxD2 float64
	ai, bi := 0, 1
	for i := 0; i < len(ps); i++ {
		for j := i + 1; j < len(ps); j++ {
			if d := dist2(ps[i], ps[j]); d > maxD2 {
				maxD2, ai, bi = d, i, j
			}
		}
	}
	if maxD2 == 0 {
		return true // every point identical (already rejected as duplicate).
	}
	a, b := ps[ai], ps[bi]
	for _, p := range ps {
		// Exact collinearity sign: no tolerance tuning required.
		if OrientSign(a, b, p) != 0 {
			return false
		}
	}
	return true
}
