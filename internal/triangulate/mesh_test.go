package triangulate_test

import (
	"math"
	"math/rand"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
)

// ---- stateful mesh: global Delaunay legality after every local edit ---

// assertGlobalDelaunay checks the hard requirement directly: no point of
// the current set lies strictly inside any triangle's circumcircle. It
// is the global O(T*N) check in the exact-predicate work frame — not a
// comparison against a from-scratch rebuild (which the degeneracy note
// explicitly forbids asserting on).
func assertGlobalDelaunay(t *testing.T, m *triangulate.Mesh, label string) {
	t.Helper()
	res, gerr := m.Snapshot()
	if gerr != nil {
		t.Fatalf("%s: snapshot: %v", label, gerr)
	}
	wp := res.WorkPoints()
	for ti, tr := range res.Triangles {
		for p := 0; p < m.PointCount(); p++ {
			if p == tr.A || p == tr.B || p == tr.C {
				continue
			}
			if geom.InCircleSign(wp[tr.A], wp[tr.B], wp[tr.C], wp[p]) > 0 {
				t.Fatalf("%s: triangle %d %+v circumcircle contains point %d",
					label, ti, tr, p)
			}
		}
	}
}

// assertHullCoverage checks the triangles exactly cover the current
// convex hull, without overlap or gaps: manifold one-owner boundary,
// Euler counts, and total triangle area equal to the independently
// computed convex-hull area.
//
// Geometry bookkeeping here deliberately runs in the *caller's*
// coordinates. The internal work frame is symbolically perturbed by
// ~1e-12*span; a point inserted microscopically inside a hull edge can
// be nudged microscopically outside it, and then the work-frame hull
// legitimately runs through that point where the unperturbed hull uses
// the edge. Both frames still carry a valid Delaunay net (the global
// empty-circle check above runs in the work frame), so structural
// coverage is asserted on the original coordinates — exactly what the
// client sees — while predicate-level legality uses the work frame.
func assertHullCoverage(t *testing.T, m *triangulate.Mesh, label string) {
	t.Helper()
	res, gerr := m.Snapshot()
	if gerr != nil {
		t.Fatalf("%s: snapshot: %v", label, gerr)
	}
	wp := res.WorkPoints()
	n := m.PointCount()
	pp := res.Points

	owners := map[[2]int]int{}
	sum := 0.0
	for _, tr := range res.Triangles {
		if geom.OrientSign(wp[tr.A], wp[tr.B], wp[tr.C]) <= 0 {
			t.Fatalf("%s: non-CCW triangle %+v", label, tr)
		}
		sum += geom.SignedArea(pp[tr.A], pp[tr.B], pp[tr.C])
		for _, e := range edgesOf(tr) {
			k := normEdge(e[0], e[1])
			owners[k]++
			if owners[k] > 2 {
				t.Fatalf("%s: edge %v owned by >2 triangles", label, k)
			}
		}
	}
	var bnd [][2]int
	for e, c := range owners {
		if c == 1 {
			bnd = append(bnd, e)
		}
	}
	ringSet := map[[2]int]bool{}
	h := res.Hull
	for i := 0; i < len(h); i++ {
		ringSet[normEdge(h[i], h[(i+1)%len(h)])] = true
	}
	if len(bnd) != len(h) {
		t.Fatalf("%s: %d boundary edges but hull ring has %d", label, len(bnd), len(h))
	}
	for _, e := range bnd {
		if !ringSet[e] {
			t.Fatalf("%s: boundary edge %v missing from hull ring", label, e)
		}
	}

	// Independent convex hull of the caller coordinates: its area must be
	// exactly the area covered by the triangles.
	ind := geom.ConvexHull(pp[:n])
	hullArea := geom.PolygonArea(pp[:n], ind)
	if !relClose(sum, hullArea, 1e-9) {
		t.Fatalf("%s: triangle area sum %.15g != convex-hull area %.15g",
			label, sum, hullArea)
	}
	// The boundary of the triangle soup must be the same closed curve as
	// the independent convex hull, compared as edge sets (collinear
	// vertices on a hull edge are real vertices in both).
	indSet := map[[2]int]bool{}
	for i := 0; i < len(ind); i++ {
		indSet[normEdge(ind[i], ind[(i+1)%len(ind)])] = true
	}
	if len(indSet) != len(ringSet) {
		t.Fatalf("%s: mesh hull has %d edges, independent hull %d",
			label, len(ringSet), len(indSet))
	}
	for e := range ringSet {
		if !indSet[e] {
			t.Fatalf("%s: mesh hull edge %v not on the independent convex hull", label, e)
		}
	}

	T, E, N := len(res.Triangles), len(owners), n
	if got := T - E + N; got != 1 {
		t.Fatalf("%s: Euler T-E+N = %d (T=%d E=%d N=%d)", label, got, T, E, N)
	}
	if T != 2*N-len(h)-2 {
		t.Fatalf("%s: T=%d want 2N-H-2=%d", label, T, 2*N-len(h)-2)
	}
}

// assertMesh is the full per-step legality gate.
func assertMesh(t *testing.T, m *triangulate.Mesh, label string) {
	t.Helper()
	assertGlobalDelaunay(t, m, label)
	assertHullCoverage(t, m, label)
}

func keyTri(tr geom.Triangle) [3]int {
	s := [3]int{tr.A, tr.B, tr.C}
	for i := 1; i < 3; i++ {
		for j := i; j > 0 && s[j] < s[j-1]; j-- {
			s[j], s[j-1] = s[j-1], s[j]
		}
	}
	return s
}

// TestStatefulMesh_InsertKeepsDelaunay builds a session mesh point by
// point and checks legality after every single insertion.
func TestStatefulMesh_InsertKeepsDelaunay(t *testing.T) {
	rng := rand.New(rand.NewSource(101))
	seed := []geom.Point{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 10, Y: 10}, {X: 0, Y: 10}, {X: 3, Y: 4}}
	m, gerr := triangulate.NewMesh(seed)
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertMesh(t, m, "seed")

	live := map[[3]int]bool{}
	res, _ := m.Snapshot()
	for _, tr := range res.Triangles {
		live[keyTri(tr)] = true
	}

	for step := 0; step < 200; step++ {
		p := geom.Point{X: rng.Float64() * 12, Y: rng.Float64() * 12}
		id, cs, ierr := m.Insert(p)
		if ierr != nil {
			t.Fatalf("insert %d: %v", step, ierr)
		}
		for _, tr := range cs.Removed {
			k := keyTri(tr)
			if !live[k] {
				t.Fatalf("step %d: change set removes triangle %v not in the old mesh", step, k)
			}
			delete(live, k)
		}
		for _, tr := range cs.Added {
			k := keyTri(tr)
			if live[k] {
				t.Fatalf("step %d: change set adds triangle %v already in the mesh", step, k)
			}
			if tr.A != id && tr.B != id && tr.C != id {
				t.Fatalf("step %d: added triangle %+v not incident to new point %d", step, tr, id)
			}
			live[k] = true
		}
		snap, _ := m.Snapshot()
		want := map[[3]int]bool{}
		for _, tr := range snap.Triangles {
			want[keyTri(tr)] = true
		}
		if len(want) != len(live) {
			t.Fatalf("step %d: applying the change set gives %d triangles, mesh has %d",
				step, len(live), len(want))
		}
		for k := range want {
			if !live[k] {
				t.Fatalf("step %d: applied change set is missing triangle %v", step, k)
			}
		}
		assertMesh(t, m, "insert "+itoa(step))
	}
}

// TestStatefulMesh_MoveKeepsDelaunay exercises the move path with a long
// random walk: every move must leave a globally legal Delaunay net and
// the returned change set must reproduce exactly the new mesh.
func TestStatefulMesh_MoveKeepsDelaunay(t *testing.T) {
	rng := rand.New(rand.NewSource(202))
	ps := []geom.Point{
		{X: 0, Y: 0}, {X: 8, Y: 0}, {X: 8, Y: 8}, {X: 0, Y: 8},
		{X: 2, Y: 2}, {X: 6, Y: 2}, {X: 6, Y: 6}, {X: 2, Y: 6},
	}
	m, gerr := triangulate.NewMesh(ps)
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertMesh(t, m, "seed")

	live := map[[3]int]bool{}
	snap0, _ := m.Snapshot()
	for _, tr := range snap0.Triangles {
		live[keyTri(tr)] = true
	}

	for step := 0; step < 400; step++ {
		id := rng.Intn(m.PointCount())
		p := geom.Point{X: rng.Float64() * 10, Y: rng.Float64() * 10}
		// Skip a move that would coincide with another live point; that
		// rejection belongs to the session layer, not the kernel test.
		bad := false
		for j := 0; j < m.PointCount(); j++ {
			q, _ := m.PointAt(j)
			if j != id && q == p {
				bad = true
			}
		}
		if bad {
			continue
		}
		cs, merr := m.Move(id, p)
		if merr != nil {
			t.Fatalf("move %d: %v", step, merr)
		}
		for _, tr := range cs.Removed {
			k := keyTri(tr)
			if !live[k] {
				t.Fatalf("step %d: removed %v not in old mesh", step, k)
			}
			delete(live, k)
		}
		for _, tr := range cs.Added {
			k := keyTri(tr)
			if live[k] {
				t.Fatalf("step %d: added %v already present", step, k)
			}
			live[k] = true
		}
		snap, _ := m.Snapshot()
		want := map[[3]int]bool{}
		for _, tr := range snap.Triangles {
			want[keyTri(tr)] = true
		}
		if len(want) != len(live) {
			t.Fatalf("step %d: applied change set (%d) disagrees with mesh (%d)",
				step, len(live), len(want))
		}
		for k := range want {
			if !live[k] {
				t.Fatalf("step %d: change set application loses %v", step, k)
			}
		}
		assertMesh(t, m, "move "+itoa(step))
	}
}

// TestStatefulMesh_MixedAndFarRanging mixes insertion and movement while
// roaming far outside the initial box (forcing local super-triangle
// widening), including hull-vertex moves that shrink, grow and reshape
// the convex hull.
func TestStatefulMesh_MixedAndFarRanging(t *testing.T) {
	rng := rand.New(rand.NewSource(303))
	ps := []geom.Point{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 1}}
	m, gerr := triangulate.NewMesh(ps)
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertMesh(t, m, "seed-triangle")

	for step := 0; step < 300; step++ {
		if m.PointCount() < 40 && rng.Intn(2) == 0 {
			scale := math.Pow(1.5, float64(rng.Intn(30)))
			p := geom.Point{
				X: (rng.Float64()*2 - 1) * scale,
				Y: (rng.Float64()*2 - 1) * scale,
			}
			dup := false
			for j := 0; j < m.PointCount(); j++ {
				q, _ := m.PointAt(j)
				if q == p {
					dup = true
				}
			}
			if dup {
				continue
			}
			if _, _, ierr := m.Insert(p); ierr != nil {
				t.Fatalf("insert %d (p=%v): %v", step, p, ierr)
			}
		} else {
			id := rng.Intn(m.PointCount())
			scale := math.Pow(1.4, float64(rng.Intn(25)))
			p := geom.Point{
				X: (rng.Float64()*2 - 1) * scale,
				Y: (rng.Float64()*2 - 1) * scale,
			}
			dup := false
			for j := 0; j < m.PointCount(); j++ {
				q, _ := m.PointAt(j)
				if j != id && q == p {
					dup = true
				}
			}
			if dup {
				continue
			}
			if _, merr := m.Move(id, p); merr != nil {
				t.Fatalf("move %d (id=%d p=%v): %v", step, id, p, merr)
			}
		}
		assertMesh(t, m, "mixed "+itoa(step))
	}
}

// TestStatefulMesh_CocircularSquareMoves pins the degeneracy rule: after
// moving a vertex around the co-circular configuration, every mesh is
// itself legal; the chosen diagonal is never asserted against a rebuild.
func TestStatefulMesh_CocircularSquareMoves(t *testing.T) {
	ps := []geom.Point{{X: 1, Y: 0}, {X: 0, Y: 1}, {X: -1, Y: 0}, {X: 0, Y: -1}}
	m, gerr := triangulate.NewMesh(ps)
	if gerr != nil {
		t.Fatal(gerr)
	}
	assertMesh(t, m, "cocircular-seed")
	for i := 0; i < 40; i++ {
		a := 2 * math.Pi * float64(i) / 40
		if _, merr := m.Move(0, geom.Point{X: math.Cos(a), Y: math.Sin(a)}); merr != nil {
			t.Fatalf("move %d: %v", i, merr)
		}
		assertMesh(t, m, "circle-walk "+itoa(i))
	}
}

// TestStatefulMesh_MoveNoop returns an empty change set and an unchanged
// mesh when the point is set to its own coordinate.
func TestStatefulMesh_MoveNoop(t *testing.T) {
	ps := []geom.Point{{X: 0, Y: 0}, {X: 5, Y: 0}, {X: 2, Y: 4}, {X: 5, Y: 5}, {X: 0, Y: 5}}
	m, gerr := triangulate.NewMesh(ps)
	if gerr != nil {
		t.Fatal(gerr)
	}
	before, _ := m.Snapshot()
	cs, merr := m.Move(2, ps[2])
	if merr != nil {
		t.Fatal(merr)
	}
	if len(cs.Removed) != 0 || len(cs.Added) != 0 {
		t.Fatalf("same-coordinate move produced a change set: %+v", cs)
	}
	after, _ := m.Snapshot()
	if len(after.Triangles) != len(before.Triangles) {
		t.Fatalf("noop move changed triangle count")
	}
	assertMesh(t, m, "noop")
}

func itoa(i int) string {
	if i == 0 {
		return "0"
	}
	var b [20]byte
	pos := len(b)
	for i > 0 {
		pos--
		b[pos] = byte('0' + i%10)
		i /= 10
	}
	return string(b[pos:])
}
