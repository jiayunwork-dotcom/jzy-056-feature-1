package triangulate_test

import (
	"math"
	"math/rand"
	"sort"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/sample"
	"delaunaysvc/internal/triangulate"
)

// ---- mesh-wide structural checks ---------------------------------------

func edgesOf(t geom.Triangle) [3][2]int {
	return [3][2]int{{t.A, t.B}, {t.B, t.C}, {t.C, t.A}}
}

func normEdge(a, b int) [2]int {
	if a > b {
		a, b = b, a
	}
	return [2]int{a, b}
}

// assertValidMesh checks the structural properties every returned
// triangulation must satisfy regardless of the input:
//
//   - every triangle is CCW with positive area;
//   - no vertex index refers to a virtual super-triangle vertex;
//   - every interior edge is shared by exactly two triangles, every hull
//     edge by exactly one;
//   - no two non-incident edges properly cross;
//   - Euler relation T - E + N = 1 and T = 2N - H - 2 hold;
//   - triangle area sum equals the hull area.
func assertValidMesh(t *testing.T, res *triangulate.Result) {
	t.Helper()
	n := len(res.Points)

	for _, tr := range res.Triangles {
		for _, v := range []int{tr.A, tr.B, tr.C} {
			if v < 0 || v >= n {
				t.Fatalf("triangle %+v references non-input index %d (ghost leaked?)", tr, v)
			}
		}
		a2 := geom.Area2(res.Points[tr.A], res.Points[tr.B], res.Points[tr.C])
		if a2 <= 0 {
			t.Fatalf("triangle %+v not strictly CCW/positive area: %v", tr, a2)
		}
	}

	edgeOwners := map[[2]int][]int{}
	for i, tr := range res.Triangles {
		for _, e := range edgesOf(tr) {
			k := normEdge(e[0], e[1])
			edgeOwners[k] = append(edgeOwners[k], i)
		}
	}
	for e, owners := range edgeOwners {
		if len(owners) > 2 {
			t.Fatalf("edge %v incident to %d triangles", e, len(owners))
		}
	}

	// Hull edges must be exactly the one-owner edges.
	hullEdgeSet := map[[2]int]bool{}
	h := res.Hull
	for i := 0; i < len(h); i++ {
		hullEdgeSet[normEdge(h[i], h[(i+1)%len(h)])] = true
	}
	boundaryEdges := 0
	for e, owners := range edgeOwners {
		if len(owners) == 1 {
			boundaryEdges++
			if !hullEdgeSet[e] {
				t.Fatalf("boundary edge %v is not a hull edge", e)
			}
		}
	}
	if boundaryEdges != len(h) {
		t.Fatalf("boundary edges = %d, hull vertices = %d", boundaryEdges, len(h))
	}

	if c := edgeCrossings(res); c != 0 {
		t.Fatalf("found %d crossing edge pairs", c)
	}

	T := len(res.Triangles)
	E := len(edgeOwners)
	N := n
	if got := T - E + N; got != 1 {
		t.Fatalf("Euler T-E+N = %d, want 1 (T=%d E=%d N=%d)", got, T, E, N)
	}
	if want := 2*N - len(h) - 2; T != want {
		t.Fatalf("T = %d, want 2N-H-2 = %d", T, want)
	}

	sum, hullArea := res.AreaSum(), res.HullArea()
	if !relClose(sum, hullArea, 1e-9) {
		t.Fatalf("area sum %.15g != hull area %.15g (abs diff %g)", sum, hullArea, math.Abs(sum-hullArea))
	}
}

func relClose(a, b, rel float64) bool {
	den := math.Max(math.Abs(a), math.Abs(b))
	if den == 0 {
		return true
	}
	return math.Abs(a-b)/den <= rel
}

func orient(p, q, r geom.Point) float64 {
	return geom.Orient2D(p, q, r)
}

// edgeCrossings counts pairs of properly intersecting, non-incident
// edges in the triangle soup.
func edgeCrossings(res *triangulate.Result) int {
	type seg struct {
		a, b int
	}
	var segs []seg
	seen := map[[2]int]bool{}
	for _, tr := range res.Triangles {
		for _, e := range edgesOf(tr) {
			k := normEdge(e[0], e[1])
			if !seen[k] {
				seen[k] = true
				segs = append(segs, seg{k[0], k[1]})
			}
		}
	}
	count := 0
	for i := 0; i < len(segs); i++ {
		for j := i + 1; j < len(segs); j++ {
			s1, s2 := segs[i], segs[j]
			if s1.a == s2.a || s1.a == s2.b || s1.b == s2.a || s1.b == s2.b {
				continue
			}
			p1, p2 := res.Points[s1.a], res.Points[s1.b]
			p3, p4 := res.Points[s2.a], res.Points[s2.b]
			d1 := orient(p3, p4, p1)
			d2 := orient(p3, p4, p2)
			d3 := orient(p1, p2, p3)
			d4 := orient(p1, p2, p4)
			if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
				((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
				count++
			}
		}
	}
	return count
}

func sortedTriangles(ts []geom.Triangle) []geom.Triangle {
	out := append([]geom.Triangle(nil), ts...)
	sort.Slice(out, func(i, j int) bool {
		if out[i].A != out[j].A {
			return out[i].A < out[j].A
		}
		if out[i].B != out[j].B {
			return out[i].B < out[j].B
		}
		return out[i].C < out[j].C
	})
	return out
}

// ---- property 1: empty circumcircle ------------------------------------

func TestEmptyCircumcircle(t *testing.T) {
	ps := []geom.Point{{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 4, Y: 4}, {X: 0, Y: 4}, {X: 2, Y: 2}, {X: 1.5, Y: 3.1}, {X: 3.2, Y: 1.2}}
	res, gerr := triangulate.Build(mustParse(t, ps))
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertValidMesh(t, res)
	ok, viols := res.IsDelaunay()
	if !ok {
		t.Fatalf("empty-circle violations: %+v", viols)
	}
}

func TestEmptyCircumcircle_Randomized(t *testing.T) {
	rng := rand.New(rand.NewSource(20260921))
	for iter := 0; iter < 40; iter++ {
		n := 4 + rng.Intn(120)
		ps := make([]geom.Point, n)
		for i := range ps {
			ps[i] = geom.Point{X: rng.Float64() * 100, Y: rng.Float64() * 100}
		}
		res, gerr := triangulate.Build(mustParse(t, ps))
		if gerr != nil {
			t.Fatal(gerr)
		}
		assertValidMesh(t, res)
		if ok, viols := res.IsDelaunay(); !ok {
			t.Fatalf("iter %d: %d violations, first %+v", iter, len(viols), viols[0])
		}
	}
}

// ---- property 2: hull area conservation --------------------------------

func TestHullAreaConservation(t *testing.T) {
	ps := []geom.Point{{X: -2, Y: -1}, {X: 3, Y: -2}, {X: 4, Y: 2}, {X: 1, Y: 4}, {X: -3, Y: 2}, {X: 0, Y: 0}, {X: 2, Y: 1}}
	res, gerr := triangulate.Build(mustParse(t, ps))
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertValidMesh(t, res)
	if !relClose(res.AreaSum(), res.HullArea(), 1e-12) {
		t.Fatalf("sum %.12f hull %.12f", res.AreaSum(), res.HullArea())
	}
}

// ---- property 3: translation invariance --------------------------------

func TestTranslationInvariance(t *testing.T) {
	ps := []geom.Point{{X: 0, Y: 0}, {X: 5, Y: 0}, {X: 5, Y: 3}, {X: 0, Y: 3}, {X: 2, Y: 1}, {X: 3.5, Y: 2}, {X: 1, Y: 2.5}}
	res0, gerr := triangulate.Build(mustParse(t, ps))
	if gerr != nil {
		t.Fatal(gerr)
	}

	shifts := []geom.Point{
		{X: 1e6, Y: -1e6},
		{X: -12345.6789, Y: 98765.4321},
		{X: 0.0001, Y: 0.0003},
	}
	for _, s := range shifts {
		moved := make([]geom.Point, len(ps))
		for i, p := range ps {
			moved[i] = geom.Point{X: p.X + s.X, Y: p.Y + s.Y}
		}
		res1, gerr := triangulate.Build(mustParse(t, moved))
		if gerr != nil {
			t.Fatal(gerr)
		}
		a := sortedTriangles(res0.Triangles)
		b := sortedTriangles(res1.Triangles)
		if len(a) != len(b) {
			t.Fatalf("shift %v: triangle count %d != %d", s, len(b), len(a))
		}
		for i := range a {
			if a[i] != b[i] {
				t.Fatalf("shift %v: topology differs at %d: %+v vs %+v", s, i, a[i], b[i])
			}
		}
		if len(res0.Hull) != len(res1.Hull) {
			t.Fatalf("shift %v: hull ring differs: %v vs %v", s, res1.Hull, res0.Hull)
		}
	}
}

// ---- property 4: inserting an interior point adds exactly two ----------

func TestInteriorPointAddsTwoTriangles(t *testing.T) {
	outer := []geom.Point{{X: 0, Y: 0}, {X: 6, Y: 0}, {X: 6, Y: 6}, {X: 0, Y: 6}, {X: 3, Y: 3}}
	// First triangulate without the center.
	before, gerr := triangulate.Build(mustParse(t, outer[:4]))
	if gerr != nil {
		t.Fatal(gerr)
	}
	after, gerr := triangulate.Build(mustParse(t, outer))
	if gerr != nil {
		t.Fatal(gerr)
	}
	if d := len(after.Triangles) - len(before.Triangles); d != 2 {
		t.Fatalf("triangle count delta = %d, want 2 (before=%d after=%d)",
			d, len(before.Triangles), len(after.Triangles))
	}
	assertValidMesh(t, before)
	assertValidMesh(t, after)

	// The new point is strictly inside the original hull: the fan around
	// it must cover exactly the whole hull area.
	hullArea := before.HullArea()
	var fanArea float64
	for _, tr := range after.Triangles {
		if tr.A == 4 || tr.B == 4 || tr.C == 4 {
			fanArea += geom.SignedArea(after.Points[tr.A], after.Points[tr.B], after.Points[tr.C])
		}
	}
	if !relClose(fanArea, hullArea, 1e-12) {
		t.Fatalf("fan around inserted point covers %.12f of hull %.12f", fanArea, hullArea)
	}
}

// ---- property 5: exactly co-circular stability -------------------------

func TestCocircularSquare_NoFlipNoCross(t *testing.T) {
	// Four points exactly on the unit circle: (1,0),(0,1),(-1,0),(0,-1).
	sq := []geom.Point{{X: 1, Y: 0}, {X: 0, Y: 1}, {X: -1, Y: 0}, {X: 0, Y: -1}}

	// Same geometric configuration under every cyclic input ordering:
	// the result must always be two CCW, non-crossing triangles (the
	// chosen diagonal may follow the order, but it must never
	// oscillate into an invalid mesh).
	for shift := 0; shift < 4; shift++ {
		ps := make([]geom.Point, 4)
		for i := 0; i < 4; i++ {
			ps[i] = sq[(i+shift)%4]
		}
		res, gerr := triangulate.Build(mustParse(t, ps))
		if gerr != nil {
			t.Fatal(gerr)
		}
		assertValidMesh(t, res)
		if len(res.Triangles) != 2 {
			t.Fatalf("shift %d: %d triangles, want 2", shift, len(res.Triangles))
		}
	}

	// Axis-aligned unit square, where the co-circularity is not exact
	// trig rounding: still exactly co-circular in float arithmetic.
	card := []geom.Point{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}, {X: 0, Y: 1}}
	res, gerr := triangulate.Build(mustParse(t, card))
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertValidMesh(t, res)
	if ok, viols := res.IsDelaunay(); !ok {
		t.Fatalf("co-circular square reports violations: %+v", viols)
	}
}

func TestNearCocircularStability(t *testing.T) {
	// Points hovering around the co-circular configuration at a sweep
	// of perturbation magnitudes: every result must be a valid mesh
	// (no crossings, positive area, exact area conservation), which is
	// what the tolerance band is there to guarantee.
	for _, eps := range []float64{0, 1e-15, 1e-13, 1e-11, 1e-9, 1e-7, 1e-5} {
		ps := []geom.Point{
			{X: 0, Y: 0},
			{X: 1 + eps, Y: 0},
			{X: 1 - eps, Y: 1},
			{X: 0, Y: 1 + 2*eps},
		}
		res, gerr := triangulate.Build(mustParse(t, ps))
		if gerr != nil {
			t.Fatalf("eps %g: %v", eps, gerr)
		}
		assertValidMesh(t, res)
	}
}

func TestRegularPolygonsAreStable(t *testing.T) {
	// All vertices of a regular polygon are exactly co-circular in the
	// ideal geometry; float construction makes them near-co-circular.
	for m := 5; m <= 16; m++ {
		ps := make([]geom.Point, m)
		for i := 0; i < m; i++ {
			a := 2 * math.Pi * float64(i) / float64(m)
			ps[i] = geom.Point{X: math.Cos(a), Y: math.Sin(a)}
		}
		res, gerr := triangulate.Build(mustParse(t, ps))
		if gerr != nil {
			t.Fatalf("m=%d: %v", m, gerr)
		}
		assertValidMesh(t, res)
		if ok, viols := res.IsDelaunay(); !ok {
			t.Fatalf("m=%d violations: %+v", m, viols[:min(3, len(viols))])
		}
		if len(res.Triangles) != m-2 {
			t.Fatalf("m=%d: T=%d want %d", m, len(res.Triangles), m-2)
		}
	}
}

// ---- built-in sample: Euler count pinned -------------------------------

func TestSampleGridTriangleCountPinned(t *testing.T) {
	ps := sample.Grid()
	if len(ps) != sample.Size {
		t.Fatalf("sample size = %d, want %d", len(ps), sample.Size)
	}
	res, gerr := triangulate.Build(mustParse(t, ps))
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertValidMesh(t, res)
	if len(res.Triangles) != sample.ExpectedTriangles {
		t.Fatalf("sample triangle count = %d, pinned value %d",
			len(res.Triangles), sample.ExpectedTriangles)
	}
	if res.HullEdgeCount() != sample.ExpectedHullEdges {
		t.Fatalf("sample hull edges = %d, want %d",
			res.HullEdgeCount(), sample.ExpectedHullEdges)
	}
	if ok, _ := res.IsDelaunay(); !ok {
		t.Fatal("sample grid is not Delaunay")
	}
}

// ---- helpers ------------------------------------------------------------

func mustParse(t *testing.T, ps []geom.Point) []geom.Point {
	t.Helper()
	out, err := geom.ParsePoints(ps)
	if err != nil {
		t.Fatal(err)
	}
	return out
}
