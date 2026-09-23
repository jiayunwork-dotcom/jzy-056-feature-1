package geom

// Exact sign fallback used by the adaptive predicates in predicates.go.
//
// The fast binary64 determinant decides the overwhelming majority of
// calls; only when its magnitude is below its proven round-off bound do
// we evaluate the same determinant in high precision with math/big. The
// inputs are binary64 values translated near the origin, so a fixed 512
// bit precision evaluates the (at most degree-four) determinant with an
// enormous safety margin and yields its exact sign.

import "math/big"

// exactPrec is the big.Float precision used for sign resolution.
const exactPrec = 512

func bf(x float64) *big.Float { return new(big.Float).SetPrec(exactPrec).SetFloat64(x) }

// orient2dExact returns the exact sign of the 2x2 orientation
// determinant: +1 CCW, -1 CW, 0 exactly collinear.
func orient2dExact(ax, ay, bx, by, cx, cy float64) int {
	acx := bf(ax - cx)
	bcx := bf(bx - cx)
	acy := bf(ay - cy)
	bcy := bf(by - cy)
	detLeft := new(big.Float).Mul(acx, bcy)
	detRight := new(big.Float).Mul(acy, bcx)
	return new(big.Float).Sub(detLeft, detRight).Sign()
}

// incircleExactVals returns the exact in-circle sign for CCW triangle
// (a,b,c) and query d: +1 inside, -1 outside, 0 co-circular.
func incircleExactVals(ax, ay, bx, by, cx, cy, dx, dy float64) int {
	adx := bf(ax - dx)
	bdx := bf(bx - dx)
	cdx := bf(cx - dx)
	ady := bf(ay - dy)
	bdy := bf(by - dy)
	cdy := bf(cy - dy)

	mul := func(x, y *big.Float) *big.Float { return new(big.Float).Mul(x, y) }
	sub := func(x, y *big.Float) *big.Float { return new(big.Float).Sub(x, y) }
	add := func(x, y *big.Float) *big.Float { return new(big.Float).Add(x, y) }

	abDet := sub(mul(adx, bdy), mul(ady, bdx))
	bcDet := sub(mul(bdx, cdy), mul(bdy, cdx))
	caDet := sub(mul(cdx, ady), mul(cdy, adx))

	alift := add(mul(adx, adx), mul(ady, ady))
	blift := add(mul(bdx, bdx), mul(bdy, bdy))
	clift := add(mul(cdx, cdx), mul(cdy, cdy))

	det := add(add(mul(alift, bcDet), mul(blift, caDet)), mul(clift, abDet))
	return det.Sign()
}
