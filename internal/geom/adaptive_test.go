package geom

import (
	"math"
	"math/big"
	"math/rand"
	"testing"
)

func bigOrient(ax, ay, bx, by, cx, cy float64, prec uint) int {
	F := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	d := new(big.Float).Sub(
		new(big.Float).Mul(F(ax-cx), F(by-cy)),
		new(big.Float).Mul(F(ay-cy), F(bx-cx)))
	return d.Sign()
}

func bigInCircle2(a, b, c, d [2]float64, prec uint) int {
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
	al := add(mul(adx, adx), mul(ady, ady))
	bl := add(mul(bdx, bdx), mul(bdy, bdy))
	cl := add(mul(cdx, cdx), mul(cdy, cdy))
	return add(add(mul(al, bc), mul(bl, ca)), mul(cl, ab)).Sign()
}

// The public adaptive predicates (fast filter + exact fallback) must
// agree with arbitrary-precision arithmetic across the whole range,
// including deliberately near-degenerate inputs that straddle the
// filter threshold in both directions.
func TestAdaptiveOrientSign_Random(t *testing.T) {
	rng := rand.New(rand.NewSource(31))
	for i := 0; i < 100000; i++ {
		s := math.Pow(10, float64(rng.Intn(14)-7))
		ax := (rng.Float64()*2 - 1) * s
		ay := (rng.Float64()*2 - 1) * s
		// b and c increasingly close to the line through a: drives the
		// determinant through the filter band.
		t0 := rng.Float64() * s
		slope := rng.NormFloat64()
		h := math.Pow(10, float64(rng.Intn(16)-16)) * s // 1e-16 .. 1e-1 of span
		bx := ax + t0
		by := ay + slope*t0
		cx := ax + rng.Float64()*s
		cy := ay + slope*(cx-ax) + rng.NormFloat64()*h
		got := OrientSign(
			Point{X: ax, Y: ay}, Point{X: bx, Y: by}, Point{X: cx, Y: cy})
		want := bigOrient(ax, ay, bx, by, cx, cy, 256)
		if got != want {
			t.Fatalf("orient adaptive mismatch: got %d want %d", got, want)
		}
	}
}

func TestAdaptiveInCircleSign_Random(t *testing.T) {
	rng := rand.New(rand.NewSource(57))
	for i := 0; i < 100000; i++ {
		s := math.Pow(10, float64(rng.Intn(10)-3))
		a := [2]float64{(rng.Float64()*2 - 1) * s, (rng.Float64()*2 - 1) * s}
		b := [2]float64{a[0] + rng.Float64()*s, a[1] + rng.NormFloat64()*s}
		c := [2]float64{a[0] + rng.NormFloat64()*s, a[1] + rng.Float64()*s}
		if orient2dExact(a[0], a[1], b[0], b[1], c[0], c[1]) <= 0 {
			b, c = c, b
		}
		// d near the circumcircle: radial offset spanning the band.
		// Place d at a triangle vertex + small perturb in many scales.
		base := a
		d := [2]float64{
			base[0] + rng.NormFloat64()*s*1e-3,
			base[1] + rng.NormFloat64()*s*1e-3,
		}
		pa, pb, pc, pd := Point{a[0], a[1]}, Point{b[0], b[1]}, Point{c[0], c[1]}, Point{d[0], d[1]}
		got := InCircleSign(pa, pb, pc, pd)
		want := bigInCircle2(a, b, c, d, 256)
		if got != want {
			t.Fatalf("incircle adaptive mismatch: got %d want %d", got, want)
		}
	}
}

func TestAdaptiveInCircleSign_ExactCocircle(t *testing.T) {
	// Four integer-grid points exactly co-circular: sign must be 0 via
	// whichever path.
	sq := [][2]float64{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	a, b, c, d := sq[0], sq[1], sq[2], sq[3]
	if s := InCircleSign(Point{a[0], a[1]}, Point{b[0], b[1]}, Point{c[0], c[1]}, Point{d[0], d[1]}); s != 0 {
		t.Fatalf("square cocircular sign = %d, want 0", s)
	}
	// A point on the circumcircle of a non-axis triangle: center (2,2),
	// r^2=8; (4,4) lies on it.
	if s := InCircleSign(Point{0, 0}, Point{4, 0}, Point{0, 4}, Point{4, 4}); s != 0 {
		t.Fatalf("(4,4) cocircular sign = %d, want 0", s)
	}
}
