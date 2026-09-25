package triangulate

import "delaunaysvc/internal/geom"

// generalPositionScale is the relative perturbation magnitude used to
// move a point set into general position. It is many orders of magnitude
// below any structurally meaningful separation but comfortably above
// binary64 round-off for the determinant predicates (the in-circle
// determinant is degree four in the coordinates, so a 1e-12 coordinate
// nudge yields an effectively unambiguous sign).
const generalPositionScale = 1e-12

// jitterPoint applies the canonical deterministic, index-dependent,
// pairwise-distinct perturbation (salt 0) the one-shot build applies to
// every point: a splitmix64-derived fraction of the set's coordinate
// span, distinct per axis and per stable point id. The stateful session
// reuses it verbatim so topology never depends on insertion timing.
//
// The perturbation is purely geometric bookkeeping: coordinates reported
// to callers are always the originals.
func jitterPoint(p geom.Point, id int, h float64) geom.Point {
	return jitterPointSalt(p, id, h, 0)
}

// jitterPointSalt applies the general-position perturbation selected by
// salt. Salt 0 reproduces the canonical scheme byte-for-byte (so all
// normal-path topologies and the translation-invariance guarantees are
// unchanged). Non-zero salts are used only by the order-robust fallback
// to step off a sliver that the canonical perturbation happens to land
// on; every salt still gives a pairwise-distinct, scale-free nudge.
func jitterPointSalt(p geom.Point, id int, h float64, salt uint64) geom.Point {
	var fx, fy float64
	if salt == 0 {
		fx = hashFraction2(uint64(id), 0xA24BAED4963EE407) - 0.5
		fy = hashFraction2(uint64(id), 0x9FB21C651E98DF25) - 0.5
	} else {
		fx = hashFraction3(uint64(id), salt, 0) - 0.5
		fy = hashFraction3(uint64(id), salt, 1) - 0.5
	}
	return geom.Point{X: p.X + h*fx, Y: p.Y + h*fy}
}

// hashFraction2 is the canonical two-word splitmix64 fraction.
func hashFraction2(seed, salt uint64) float64 {
	z := seed ^ salt
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ^ (z >> 27)) * 0x94D049BB133111EB
	z ^= z >> 31
	return float64(z>>11) * (1.0 / 9007199254740992.0) // 2^53
}

// hashFraction3 is the three-word variant used by non-canonical salts.
func hashFraction3(seed, salt, axis uint64) float64 {
	z := seed ^ salt ^ (axis*0x9E3779B97F4A7C15 + 0xD1B54A32D192ED03)
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ^ (z >> 27)) * 0x94D049BB133111EB
	z ^= z >> 31
	return float64(z>>11) * (1.0 / 9007199254740992.0) // 2^53
}
