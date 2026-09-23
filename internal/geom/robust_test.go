package geom

import (
	"math"
	"math/big"
	"math/rand"
	"testing"
)

func bigSign3(ax, ay, bx, by, cx, cy float64, prec uint) int {
	F := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	d := new(big.Float).Sub(
		new(big.Float).Mul(F(ax-cx), F(by-cy)),
		new(big.Float).Mul(F(ay-cy), F(bx-cx)))
	return d.Sign()
}

func bigSignIncircle(a, b, c, d [2]float64, prec uint) int {
	F := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	mk := func(p [2]float64) (x, y *big.Float) { return F(p[0]), F(p[1]) }
	ax, ay := mk(a)
	bx, by := mk(b)
	cx, cy := mk(c)
	dx, dy := mk(d)
	sub := func(x, y *big.Float) *big.Float { return new(big.Float).Sub(x, y) }
	mul := func(x, y *big.Float) *big.Float { return new(big.Float).Mul(x, y) }
	add := func(x, y *big.Float) *big.Float { return new(big.Float).Add(x, y) }

	adx, ady := sub(ax, dx), sub(ay, dy)
	bdx, bdy := sub(bx, dx), sub(by, dy)
	cdx, cdy := sub(cx, dx), sub(cy, dy)
	ab := sub(mul(adx, bdy), mul(ady, bdx))
	bc := sub(mul(bdx, cdy), mul(bdy, cdx))
	ca := sub(mul(cdx, ady), mul(cdy, adx))
	alift := add(mul(adx, adx), mul(ady, ady))
	blift := add(mul(bdx, bdx), mul(bdy, bdy))
	clift := add(mul(cdx, cdx), mul(cdy, cdy))
	det := add(add(mul(alift, bc), mul(blift, ca)), mul(clift, ab))
	return det.Sign()
}

func TestOrient2DExact_RandomVsBig(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	for i := 0; i < 20000; i++ {
		// Mix scales and near-degenerate triples.
		s := math.Pow(10, float64(rng.Intn(12)-6))
		ax := (rng.Float64()*2 - 1) * s
		ay := (rng.Float64()*2 - 1) * s
		bx := ax + rng.NormFloat64()*s
		by := ay + rng.NormFloat64()*s
		cx := bx + rng.NormFloat64()*s*1e-3
		cy := by + rng.NormFloat64()*s*1e-3
		got := orient2dExact(ax, ay, bx, by, cx, cy)
		want := bigSign3(ax, ay, bx, by, cx, cy, 256)
		if got != want {
			t.Fatalf("orient mismatch (%g,%g)(%g,%g)(%g,%g): got %d want %d",
				ax, ay, bx, by, cx, cy, got, want)
		}
	}
}

func TestInCircleExact_RandomVsBig(t *testing.T) {
	rng := rand.New(rand.NewSource(11))
	for i := 0; i < 20000; i++ {
		s := math.Pow(10, float64(rng.Intn(10)-3))
		a := [2]float64{(rng.Float64()*2 - 1) * s, (rng.Float64()*2 - 1) * s}
		b := [2]float64{a[0] + rng.Float64()*s, a[1] + rng.NormFloat64()*s}
		c := [2]float64{a[0] + rng.NormFloat64()*s, a[1] + rng.Float64()*s}
		// ensure CCW
		if orient2dExact(a[0], a[1], b[0], b[1], c[0], c[1]) <= 0 {
			b, c = c, b
		}
		d := [2]float64{a[0] + rng.NormFloat64()*s, a[1] + rng.NormFloat64()*s}
		got := incircleExactVals(a[0], a[1], b[0], b[1], c[0], c[1], d[0], d[1])
		want := bigSignIncircle(a, b, c, d, 256)
		if got != want {
			t.Fatalf("incircle mismatch a%v b%v c%v d%v: got %d want %d", a, b, c, d, got, want)
		}
	}
}

func TestInCircleExact_KnownCases(t *testing.T) {
	a := [2]float64{0, 0}
	b := [2]float64{2, 0}
	c := [2]float64{0, 2}
	if incircleExactVals(a[0], a[1], b[0], b[1], c[0], c[1], 1, 1) <= 0 {
		t.Fatal("center must be inside (>0)")
	}
	if incircleExactVals(a[0], a[1], b[0], b[1], c[0], c[1], 10, 10) >= 0 {
		t.Fatal("far point must be outside (<0)")
	}
	// Exact co-circular: (2,2) on circle center (1,1) r sqrt2.
	if incircleExactVals(a[0], a[1], b[0], b[1], c[0], c[1], 2, 2) != 0 {
		t.Fatal("(2,2) must be exactly on the circle")
	}
}

func TestInCircleExact_NearCoincidentVertices(t *testing.T) {
	// Reproduce the failure mode: a vertex within 1e-1 of another,
	// query on the circle region; exact sign must agree with big.Float.
	rng := rand.New(rand.NewSource(99))
	for i := 0; i < 5000; i++ {
		s := 1000.0
		a := [2]float64{rng.Float64() * s, rng.Float64() * s}
		// b very near a
		b := [2]float64{a[0] + rng.Float64()*0.1, a[1] + rng.Float64()*0.1}
		c := [2]float64{a[0] + rng.Float64()*5, a[1] + rng.Float64()*5}
		if orient2dExact(a[0], a[1], b[0], b[1], c[0], c[1]) <= 0 {
			b, c = c, b
		}
		d := [2]float64{a[0] + rng.NormFloat64()*3, a[1] + rng.NormFloat64()*3}
		got := incircleExactVals(a[0], a[1], b[0], b[1], c[0], c[1], d[0], d[1])
		want := bigSignIncircle(a, b, c, d, 256)
		if got != want {
			t.Fatalf("near-coincident mismatch: got %d want %d a%v b%v c%v d%v", got, want, a, b, c, d)
		}
	}
}
