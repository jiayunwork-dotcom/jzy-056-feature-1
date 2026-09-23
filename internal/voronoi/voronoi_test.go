package voronoi_test

import (
	"math"
	"math/rand"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
	"delaunaysvc/internal/voronoi"
)

func buildDiagram(t *testing.T, ps []geom.Point) (*triangulate.Result, *voronoi.Diagram) {
	t.Helper()
	valid, err := geom.ParsePoints(ps)
	if err != nil {
		t.Fatal(err)
	}
	res, gerr := triangulate.Build(valid)
	if gerr != nil {
		t.Fatal(gerr)
	}
	return res, voronoi.Build(res)
}

// Every Voronoi vertex must be the circumcenter of exactly its
// generating triangle, and equidistant (up to round-off) from that
// triangle's three vertices.
func TestVoronoiVerticesAreCircumcenters(t *testing.T) {
	ps := []geom.Point{{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 4, Y: 4}, {X: 0, Y: 4}, {X: 2, Y: 2}, {X: 1, Y: 3}, {X: 3.2, Y: 1.1}}
	res, d := buildDiagram(t, ps)

	if len(d.Vertices) != len(res.Triangles) {
		t.Fatalf("vertex count %d != triangle count %d", len(d.Vertices), len(res.Triangles))
	}
	for i, v := range d.Vertices {
		if v.Index != i {
			t.Fatalf("vertex index %d at position %d", v.Index, i)
		}
		tr := res.Triangles[i]
		want := geom.Circumcenter(res.Points[tr.A], res.Points[tr.B], res.Points[tr.C])
		if math.Abs(v.Point.X-want.X) > 1e-9 || math.Abs(v.Point.Y-want.Y) > 1e-9 {
			t.Fatalf("vertex %d = %v, want circumcenter %v", i, v.Point, want)
		}
		da := math.Hypot(v.Point.X-ps[tr.A].X, v.Point.Y-ps[tr.A].Y)
		db := math.Hypot(v.Point.X-ps[tr.B].X, v.Point.Y-ps[tr.B].Y)
		dc := math.Hypot(v.Point.X-ps[tr.C].X, v.Point.Y-ps[tr.C].Y)
		if math.Abs(da-db) > 1e-8*math.Max(1, da) || math.Abs(da-dc) > 1e-8*math.Max(1, da) {
			t.Fatalf("vertex %d not equidistant: %v %v %v", i, da, db, dc)
		}
	}
}

// Every finite Voronoi edge joins the circumcenters of the two triangles
// sharing a Delaunay edge and lies on that edge's perpendicular
// bisector. The finite edges must form the exact set of triangle pairs
// over interior edges.
func TestVoronoiEdgesDualToInteriorEdges(t *testing.T) {
	ps := []geom.Point{{X: 0, Y: 0}, {X: 5, Y: 0}, {X: 6, Y: 3}, {X: 4, Y: 6}, {X: 1, Y: 5}, {X: -1, Y: 2}, {X: 2, Y: 2}, {X: 4, Y: 2}}
	res, d := buildDiagram(t, ps)

	// Rebuild expected pairs from edge -> triangles adjacency.
	owners := map[[2]int][]int{}
	for ti, tr := range res.Triangles {
		for _, e := range [3][2]int{{tr.A, tr.B}, {tr.B, tr.C}, {tr.C, tr.A}} {
			if e[0] > e[1] {
				e[0], e[1] = e[1], e[0]
			}
			owners[e] = append(owners[e], ti)
		}
	}
	want := map[[2]int]bool{}
	for _, o := range owners {
		if len(o) == 2 {
			u, v := o[0], o[1]
			if u > v {
				u, v = v, u
			}
			want[[2]int{u, v}] = true
		}
	}
	got := map[[2]int]bool{}
	for _, e := range d.Edges {
		got[[2]int{e.U, e.V}] = true
	}
	if len(got) != len(want) {
		t.Fatalf("finite edge count %d, want %d", len(got), len(want))
	}
	for k := range want {
		if !got[k] {
			t.Fatalf("missing dual edge between triangles %v", k)
		}
	}

	// For each finite edge, find the shared Delaunay edge (common
	// vertices of the two triangles) and check perpendicular bisector.
	for _, e := range d.Edges {
		t1, t2 := res.Triangles[e.U], res.Triangles[e.V]
		shared := sharedVertices(t1, t2)
		if len(shared) != 2 {
			t.Fatalf("triangles %v,%v share %d vertices", t1, t2, len(shared))
		}
		p, q := res.Points[shared[0]], res.Points[shared[1]]
		c1 := d.Vertices[e.U].Point
		c2 := d.Vertices[e.V].Point

		// c1 and c2 must be equidistant from p and q.
		d1p := math.Hypot(c1.X-p.X, c1.Y-p.Y)
		d1q := math.Hypot(c1.X-q.X, c1.Y-q.Y)
		if math.Abs(d1p-d1q) > 1e-7*math.Max(1, d1p) {
			t.Fatalf("center %v not on bisector of %v-%v: %v vs %v", c1, p, q, d1p, d1q)
		}
		// Voronoi edge vector perpendicular to Delaunay edge vector.
		vx, vy := c2.X-c1.X, c2.Y-c1.Y
		ex, ey := q.X-p.X, q.Y-p.Y
		dot := vx*ex + vy*ey
		scale := math.Hypot(vx, vy) * math.Hypot(ex, ey)
		if scale > 0 && math.Abs(dot)/scale > 1e-9 {
			t.Fatalf("voronoi edge not perpendicular to delaunay edge: sin=%g", dot/scale)
		}
	}

	// One ray per hull edge.
	if len(d.Rays) != len(res.Hull) {
		t.Fatalf("rays %d, hull edges %d", len(d.Rays), len(res.Hull))
	}
}

func sharedVertices(a, b geom.Triangle) []int {
	set := map[int]bool{a.A: true, a.B: true, a.C: true}
	var out []int
	for _, v := range []int{b.A, b.B, b.C} {
		if set[v] {
			out = append(out, v)
		}
	}
	return out
}

// Translating all input points must translate every Voronoi vertex by
// the same vector while leaving the dual topology untouched.
func TestVoronoiTranslationCovariance(t *testing.T) {
	ps := []geom.Point{{X: 0, Y: 0}, {X: 5, Y: 0}, {X: 5, Y: 4}, {X: 0, Y: 4}, {X: 2.5, Y: 2}, {X: 1, Y: 3}}
	res0, d0 := buildDiagram(t, ps)
	shift := geom.Point{X: 3.7e6, Y: -8.2e6}
	moved := make([]geom.Point, len(ps))
	for i, p := range ps {
		moved[i] = geom.Point{X: p.X + shift.X, Y: p.Y + shift.Y}
	}
	res1, d1 := buildDiagram(t, moved)

	if len(d0.Vertices) != len(d1.Vertices) || len(d0.Edges) != len(d1.Edges) || len(d0.Rays) != len(d1.Rays) {
		t.Fatal("dual topology changed under translation")
	}
	for i := range d0.Vertices {
		if d0.Vertices[i].Index != d1.Vertices[i].Index {
			t.Fatalf("vertex %d index mismatch", i)
		}
		gx := d1.Vertices[i].Point.X - d0.Vertices[i].Point.X
		gy := d1.Vertices[i].Point.Y - d0.Vertices[i].Point.Y
		tol := 1e-6
		if math.Abs(gx-shift.X) > tol || math.Abs(gy-shift.Y) > tol {
			t.Fatalf("vertex %d shifted (%v,%v), want (%v,%v)", i, gx, gy, shift.X, shift.Y)
		}
	}
	for i := range d0.Edges {
		if d0.Edges[i] != d1.Edges[i] {
			t.Fatalf("edge %d topology differs: %+v vs %+v", i, d0.Edges[i], d1.Edges[i])
		}
	}
	_ = res0
	_ = res1
}

func TestVoronoiRandomized(t *testing.T) {
	rng := rand.New(rand.NewSource(777))
	for iter := 0; iter < 20; iter++ {
		n := 5 + rng.Intn(50)
		ps := make([]geom.Point, n)
		for i := range ps {
			ps[i] = geom.Point{X: rng.Float64()*20 - 10, Y: rng.Float64()*20 - 10}
		}
		res, d := buildDiagram(t, ps)
		for i, v := range d.Vertices {
			tr := res.Triangles[i]
			da := math.Hypot(v.Point.X-ps[tr.A].X, v.Point.Y-ps[tr.A].Y)
			db := math.Hypot(v.Point.X-ps[tr.B].X, v.Point.Y-ps[tr.B].Y)
			dc := math.Hypot(v.Point.X-ps[tr.C].X, v.Point.Y-ps[tr.C].Y)
			tol := 1e-8 * math.Max(1, da)
			if math.Abs(da-db) > tol || math.Abs(da-dc) > tol {
				t.Fatalf("iter %d vertex %d not equidistant", iter, i)
			}
		}
		if len(d.Rays) != len(res.Hull) {
			t.Fatalf("iter %d rays %d hull %d", iter, len(d.Rays), len(res.Hull))
		}
	}
}
