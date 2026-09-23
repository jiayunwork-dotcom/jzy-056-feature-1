package geom

import "math"

// Robust sign predicates used throughout the kernel.
//
// Every sign decision (orientation, in-circle) is made in two stages:
//
//  1. a fast binary64 determinant; if its magnitude comfortably exceeds
//     a proven forward round-off bound, its sign is returned directly;
//  2. otherwise (co-circular / nearly degenerate configuration) the
//     sign is recomputed in high precision with math/big (see
//     robust.go).
//
// The returned sign is therefore the exact sign of the real determinant
// of the binary64 inputs. Four nearly co-circular points or nearly
// coincident vertices never flip a sign from float jitter, and the
// common case costs only a handful of FP multiplies. There is no
// hand-tuned absolute epsilon on a scale-dependent determinant.

// eps is 2^-52, the binary64 unit round-off.
const eps = 2.2204460492503131e-16

// orientBound is Shewchuk's ccwerrboundA, the proven round-off bound of
// the non-static 2x2 orientation filter.
var orientBound = (3.0 + 16.0*eps) * eps

// incircleFilterConst is the constant in the strict forward-error bound
// of the degree-four in-circle determinant. With M the largest absolute
// translated coordinate difference, a first-order round-off analysis of
// the six products, the three cross-determinants, the three squared
// norms and their final products and sums bounds the error below
// 84*eps*M^4; 256 leaves a wide margin for higher-order terms. The bound
// only decides when exact arithmetic is unnecessary: an over-large
// constant merely routes more calls to the exact path, never a wrong
// sign. The O(eps*M^4) threshold is negligible against an O(M^4)
// determinant, so non-degenerate calls take the fast path.
const incircleFilterConst = 256.0

// OrientSign returns the exact sign of the (p, q, r) orientation:
// +1 counter-clockwise, -1 clockwise, 0 exactly collinear.
func OrientSign(p, q, r Point) int {
	acx := p.X - r.X
	bcx := q.X - r.X
	acy := p.Y - r.Y
	bcy := q.Y - r.Y
	detLeft := acx * bcy
	detRight := acy * bcx
	det := detLeft - detRight

	// Fast filter.
	if det > orientBound*(math.Abs(detLeft)+math.Abs(detRight)) {
		return 1
	}
	if det < -orientBound*(math.Abs(detLeft)+math.Abs(detRight)) {
		return -1
	}
	return orient2dExact(p.X, p.Y, q.X, q.Y, r.X, r.Y)
}

// InCircleSign classifies d against the circumcircle of CCW triangle
// (a, b, c): +1 strictly inside, 0 exactly co-circular, -1 outside.
func InCircleSign(a, b, c, d Point) int {
	adx := a.X - d.X
	bdx := b.X - d.X
	cdx := c.X - d.X
	ady := a.Y - d.Y
	bdy := b.Y - d.Y
	cdy := c.Y - d.Y

	bdxc := adx * bdy
	bdyc := ady * bdx
	cdxc := bdx * cdy
	cdyc := bdy * cdx
	adxc := cdx * ady
	adyc := adx * cdy

	abDet := bdxc - bdyc
	bcDet := cdxc - cdyc
	caDet := adxc - adyc

	alift := adx*adx + ady*ady
	blift := bdx*bdx + bdy*bdy
	clift := cdx*cdx + cdy*cdy

	det := alift*bcDet + blift*caDet + clift*abDet

	// Strictly conservative M^4 error bound.
	m := math.Max(math.Max(math.Abs(adx), math.Abs(ady)),
		math.Max(math.Max(math.Abs(bdx), math.Abs(bdy)),
			math.Max(math.Abs(cdx), math.Abs(cdy))))
	m4 := m * m * m * m
	tol := incircleFilterConst * eps * m4
	if det > tol {
		return 1
	}
	if det < -tol {
		return -1
	}
	return incircleExactVals(a.X, a.Y, b.X, b.Y, c.X, c.Y, d.X, d.Y)
}

// Circumcircle returns the center and squared radius of the circumcircle
// of triangle (a, b, c). The triangle must be non-degenerate.
//
// The center is computed from edge vectors measured relative to a
// (rather than the global origin); doing the arithmetic in local
// coordinates makes the result covariant under rigid translation:
// translating every input point translates the center by exactly the
// same vector (up to the same round-off).
func Circumcircle(a, b, c Point) (center Point, r2 float64, ok bool) {
	ux := b.X - a.X
	uy := b.Y - a.Y
	vx := c.X - a.X
	vy := c.Y - a.Y
	d := 2 * (ux*vy - uy*vx)
	if math.Abs(d) < 1e-30 {
		return Point{}, 0, false
	}
	uu := ux*ux + uy*uy
	vv := vx*vx + vy*vy
	cx := a.X + (vy*uu-uy*vv)/d
	cy := a.Y + (ux*vv-vx*uu)/d
	return Point{X: cx, Y: cy}, dist2(Point{X: cx, Y: cy}, a), true
}

// Circumcenter returns the circumcenter of triangle (a, b, c). It
// panics for a degenerate triangle; the kernel only ever produces
// strictly CCW, positive-area triangles.
func Circumcenter(a, b, c Point) Point {
	center, _, ok := Circumcircle(a, b, c)
	if !ok {
		panic("geom: circumcenter of degenerate triangle")
	}
	return center
}
