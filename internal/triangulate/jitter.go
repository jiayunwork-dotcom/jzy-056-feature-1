package triangulate

import "delaunaysvc/internal/geom"

// generalPositionScale is the relative perturbation magnitude used to
// move a point set into general position. It is many orders of magnitude
// below any structurally meaningful separation but comfortably above
// binary64 round-off for the determinant predicates (the in-circle
// determinant is degree four in the coordinates, so a 1e-12 coordinate
// nudge yields an effectively unambiguous sign).
const generalPositionScale = 1e-12

// jitterToGeneralPosition applies a deterministic, index-dependent,
// pairwise-distinct perturbation to the (already origin-translated)
// coordinates. The nudge in each axis is proportional to the set's
// coordinate span, so it is scale-free, and is derived only from the
// point's index via a deterministic integer hash, so repeated builds of
// the same request produce byte-identical coordinates and therefore an
// identical triangulation — eliminating predicate flip-flop on exactly
// or nearly co-circular input.
//
// The perturbation is purely geometric bookkeeping: output vertex
// coordinates reported to callers are the original ones; only the
// internal predicate frame is perturbed.
func jitterToGeneralPosition(local []geom.Point) {
	if len(local) == 0 {
		return
	}
	minX, minY := local[0].X, local[0].Y
	maxX, maxY := minX, minY
	for _, p := range local[1:] {
		minX = mathMin(minX, p.X)
		maxX = mathMax(maxX, p.X)
		minY = mathMin(minY, p.Y)
		maxY = mathMax(maxY, p.Y)
	}
	span := mathMax(maxX-minX, maxY-minY)
	if span == 0 {
		span = 1
	}
	hx := generalPositionScale * span
	hy := generalPositionScale * span
	for i := range local {
		// Distinct fractions in (-0.5, 0.5) per axis, deterministic in i.
		fx := hashFraction(uint64(i), 0xA24BAED4963EE407) - 0.5
		fy := hashFraction(uint64(i), 0x9FB21C651E98DF25) - 0.5
		local[i] = geom.Point{
			X: local[i].X + hx*fx,
			Y: local[i].Y + hy*fy,
		}
	}
}

// hashFraction maps (seed, salt) to a deterministic value in [0, 1)
// using a splitmix64-style finalizer; different salts decorrelate the x
// and y nudges.
func hashFraction(seed uint64, salt uint64) float64 {
	z := seed ^ salt
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ^ (z >> 27)) * 0x94D049BB133111EB
	z ^= z >> 31
	// Map to [0,1) with 53-bit mantissa resolution.
	return float64(z>>11) * (1.0 / 9007199254740992.0) // 2^53
}
