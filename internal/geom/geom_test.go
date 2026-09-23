package geom_test

import (
	"math"
	"testing"

	"delaunaysvc/internal/geom"
)

func TestParsePoints_RejectsTooFew(t *testing.T) {
	cases := [][]geom.Point{
		nil,
		{},
		{{X: 0, Y: 0}},
		{{X: 0, Y: 0}, {X: 1, Y: 0}},
	}
	for i, ps := range cases {
		_, err := geom.ParsePoints(ps)
		if err == nil {
			t.Fatalf("case %d: expected error, got nil", i)
		}
		if err.Code != geom.ErrTooFewPoints {
			t.Fatalf("case %d: code = %q, want %q", i, err.Code, geom.ErrTooFewPoints)
		}
	}
}

func TestParsePoints_RejectsInvalidCoordinates(t *testing.T) {
	base := []geom.Point{{0, 0}, {1, 0}, {0, 1}}
	bads := []geom.Point{
		{X: math.NaN(), Y: 0},
		{X: 0, Y: math.NaN()},
		{X: math.Inf(1), Y: 0},
		{X: 0, Y: math.Inf(-1)},
	}
	for i, bad := range bads {
		ps := append([]geom.Point{}, base...)
		ps = append(ps, bad)
		_, err := geom.ParsePoints(ps)
		if err == nil || err.Code != geom.ErrInvalidCoord {
			t.Fatalf("case %d: got %v, want code %s", i, err, geom.ErrInvalidCoord)
		}
	}
}

func TestParsePoints_RejectsDuplicates(t *testing.T) {
	ps := []geom.Point{{0, 0}, {1, 0}, {0, 1}, {1, 0}}
	_, err := geom.ParsePoints(ps)
	if err == nil || err.Code != geom.ErrDuplicatePoint {
		t.Fatalf("got %v, want code %s", err, geom.ErrDuplicatePoint)
	}
}

func TestParsePoints_RejectsCollinear(t *testing.T) {
	ps := []geom.Point{{0, 0}, {1, 0}, {2, 0}, {3, 0}}
	_, err := geom.ParsePoints(ps)
	if err == nil || err.Code != geom.ErrAllCollinear {
		t.Fatalf("got %v, want code %s", err, geom.ErrAllCollinear)
	}

	// Collinear on a slanted line far from the origin must also be
	// detected (the collinearity tolerance is relative).
	slanted := []geom.Point{{1e8 + 0, 1e8 + 0}, {1e8 + 1, 1e8 + 2}, {1e8 + 2, 1e8 + 4}}
	_, err = geom.ParsePoints(slanted)
	if err == nil || err.Code != geom.ErrAllCollinear {
		t.Fatalf("slanted: got %v, want %s", err, geom.ErrAllCollinear)
	}

	// But a genuinely off-line point at that scale must pass.
	off := []geom.Point{{1e8, 1e8}, {1e8 + 1, 1e8 + 2}, {1e8 + 2, 1e8 + 4}, {1e8 + 1, 1e8 + 3}}
	if _, err := geom.ParsePoints(off); err != nil {
		t.Fatalf("off-line set rejected: %v", err)
	}
}

func TestParsePoints_AcceptsValidAndPreservesOrder(t *testing.T) {
	ps := []geom.Point{{2, 1}, {0, 0}, {1, 0}, {0, 1}}
	out, err := geom.ParsePoints(ps)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(out) != len(ps) {
		t.Fatalf("len = %d, want %d", len(out), len(ps))
	}
	for i := range ps {
		if out[i] != ps[i] {
			t.Fatalf("index %d mutated: got %v want %v", i, out[i], ps[i])
		}
	}
	// The returned slice must not alias the input.
	out[0] = geom.Point{}
	if ps[0].X != 2 {
		t.Fatal("ParsePoints returned an aliasing slice")
	}
}

func TestInCircle_UnitRightTriangle(t *testing.T) {
	a := geom.Point{0, 0}
	b := geom.Point{2, 0}
	c := geom.Point{0, 2}
	// Circumcenter (1,1), radius sqrt(2).
	mid := geom.Point{1, 1}
	if geom.InCircleSign(a, b, c, mid) <= 0 {
		t.Fatal("circumcenter must be inside the circumcircle")
	}
	far := geom.Point{10, 10}
	if geom.InCircleSign(a, b, c, far) >= 0 {
		t.Fatal("(10,10) must be outside")
	}
	// Point on the circle: (2,2) is exactly distance sqrt(2) from (1,1).
	on := geom.Point{2, 2}
	if got := geom.InCircleSign(a, b, c, on); got != 0 {
		t.Fatalf("(2,2): sign = %d, want 0", got)
	}
}

func TestInCircle_OrientationSign(t *testing.T) {
	// Flipping triangle orientation flips the determinant sign.
	a := geom.Point{0, 0}
	b := geom.Point{2, 0}
	c := geom.Point{0, 2}
	d := geom.Point{1, 1}
	if geom.InCircleSign(a, b, c, d) <= 0 {
		t.Fatal("CCW triangle: expected inside")
	}
	if geom.InCircleSign(a, c, b, d) >= 0 {
		t.Fatal("CW triangle: expected outside")
	}
}

func TestCircumcircle_KnownCenter(t *testing.T) {
	a := geom.Point{0, 0}
	b := geom.Point{4, 0}
	c := geom.Point{0, 6}
	// Center (2,3), r2 = 13.
	center, r2, ok := geom.Circumcircle(a, b, c)
	if !ok {
		t.Fatal("degenerate ok=false")
	}
	if center.X != 2 || center.Y != 3 {
		t.Fatalf("center = %v, want (2,3)", center)
	}
	if math.Abs(r2-13) > 1e-12 {
		t.Fatalf("r2 = %v, want 13", r2)
	}
}

func TestCircumcircle_TranslationCovariance(t *testing.T) {
	a := geom.Point{-3, -2}
	b := geom.Point{5, 1}
	c := geom.Point{2, 7}
	shift := geom.Point{X: 1e7 - 4, Y: -3e7 + 9}
	center0, _, ok := geom.Circumcircle(a, b, c)
	if !ok {
		t.Fatal("degenerate")
	}
	center1, _, ok := geom.Circumcircle(
		geom.Point{a.X + shift.X, a.Y + shift.Y},
		geom.Point{b.X + shift.X, b.Y + shift.Y},
		geom.Point{c.X + shift.X, c.Y + shift.Y},
	)
	if !ok {
		t.Fatal("degenerate shifted")
	}
	if d := math.Abs(center1.X - (center0.X + shift.X)); d > 1e-5 {
		t.Fatalf("center.X not covariant: diff %g", d)
	}
	if d := math.Abs(center1.Y - (center0.Y + shift.Y)); d > 1e-5 {
		t.Fatalf("center.Y not covariant: diff %g", d)
	}
}

func TestConvexHull_SquareWithInteriorPoint(t *testing.T) {
	ps := []geom.Point{{0, 0}, {2, 0}, {2, 2}, {0, 2}, {1, 1}}
	hull := geom.ConvexHull(ps)
	want := []int{0, 1, 2, 3}
	if len(hull) != len(want) {
		t.Fatalf("hull = %v, want %v", hull, want)
	}
	for i, idx := range want {
		if hull[i] != idx {
			t.Fatalf("hull = %v, want %v", hull, want)
		}
	}
	// CCW check
	for i := 0; i < len(hull); i++ {
		p := ps[hull[i]]
		q := ps[hull[(i+1)%len(hull)]]
		r := ps[hull[(i+2)%len(hull)]]
		if geom.Orient2D(p, q, r) < 0 {
			t.Fatalf("hull not CCW at edge %d", i)
		}
	}
}

func TestConvexHull_KeepsCollinearBoundaryPoints(t *testing.T) {
	// A midpoint on a hull edge is a real input vertex and must be
	// retained (it participates in a hull edge of the triangulation).
	ps := []geom.Point{{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}}
	hull := geom.ConvexHull(ps)
	if len(hull) != 5 {
		t.Fatalf("hull = %v, want 5 vertices", hull)
	}
	if a := geom.PolygonArea(ps, hull); math.Abs(a-4) > 1e-12 {
		t.Fatalf("hull area = %v, want 4", a)
	}
}
