package triangulate_test

import (
	"math"
	"math/rand"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
)

// globalEmptyCircle checks the GLOBAL empty-circumcircle property against
// every point (not just edge-neighbours), plus coverage of the convex
// hull. It is the strict acceptance criterion for the incremental mesh.
func globalEmptyCircle(t *testing.T, res *triangulate.Result) {
	t.Helper()
	wp := res.WorkPoints()
	// Global condition: no point strictly inside any circumcircle.
	for ti, tr := range res.Triangles {
		for p := 0; p < len(wp); p++ {
			if p == tr.A || p == tr.B || p == tr.C {
				continue
			}
			if geom.InCircleSign(wp[tr.A], wp[tr.B], wp[tr.C], wp[p]) > 0 {
				t.Fatalf("triangle %d %+v: point %d strictly inside circumcircle",
					ti, tr, p)
			}
		}
	}
	// Local condition as a cross-check.
	if ok, viols := res.IsDelaunay(); !ok {
		t.Fatalf("local Delaunay violations: %+v", viols[:min(3, len(viols))])
	}
}

func TestMesh_InsertAndMove_Randomized(t *testing.T) {
	rng := rand.New(rand.NewSource(424242))
	for iter := 0; iter < 60; iter++ {
		// Start from 4-10 random points.
		n0 := 4 + rng.Intn(7)
		ps := make([]geom.Point, n0)
		for i := range ps {
			ps[i] = geom.Point{X: rng.Float64()*20 - 10, Y: rng.Float64()*20 - 10}
		}
		pts, verr := geom.ParsePoints(ps)
		if verr != nil {
			t.Fatalf("iter %d: %v", iter, verr)
		}
		m, gerr := triangulate.NewMesh(pts)
		if gerr != nil {
			t.Fatalf("iter %d: %v", iter, gerr)
		}
		globalEmptyCircle(t, m.Snapshot())

		used := map[geom.Point]bool{}
		for _, p := range pts {
			used[p] = true
		}

		for step := 0; step < 80; step++ {
			switch {
			case rng.Intn(2) == 0:
				// Insert a fresh distinct point.
				before := m.Snapshot()
				var np geom.Point
				for tries := 0; ; tries++ {
					np = geom.Point{
						X: rng.Float64()*20 - 10 + rng.NormFloat64()*2,
						Y: rng.Float64()*20 - 10 + rng.NormFloat64()*2,
					}
					if !used[np] {
						break
					}
					if tries > 100 {
						np = geom.Point{X: rng.Float64() * 100, Y: rng.Float64() * 100}
						if !used[np] {
							break
						}
					}
				}
				ch, gerr := m.Insert(np)
				if gerr != nil {
					t.Fatalf("iter %d step %d insert %v: %v", iter, step, np, gerr)
				}
				used[np] = true
				checkChangeSet(t, before, m.Snapshot(), ch)
			default:
				// Move a random existing point to a fresh spot.
				before := m.Snapshot()
				i := rng.Intn(m.PointCount())
				var np geom.Point
				for tries := 0; ; tries++ {
					np = geom.Point{
						X: rng.Float64()*20 - 10 + rng.NormFloat64()*2,
						Y: rng.Float64()*20 - 10 + rng.NormFloat64()*2,
					}
					dup := false
					for j := 0; j < m.PointCount(); j++ {
						if j != i && m.OriginalPoint(j) == np {
							dup = true
							break
						}
					}
					if !dup {
						break
					}
					if tries > 100 {
						np = geom.Point{X: rng.Float64()*100 - 50, Y: rng.Float64()*100 - 50}
						dup = false
						for j := 0; j < m.PointCount(); j++ {
							if j != i && m.OriginalPoint(j) == np {
								dup = true
								break
							}
						}
						if !dup {
							break
						}
					}
				}
				old := m.OriginalPoint(i)
				ch, gerr := m.Move(i, np)
				if gerr != nil {
					t.Fatalf("iter %d step %d move %d->%v: %v", iter, step, i, np, gerr)
				}
				delete(used, old)
				used[np] = true
				checkChangeSet(t, before, m.Snapshot(), ch)
			}
			snap := m.Snapshot()
			globalEmptyCircle(t, snap)
			assertIncrementalMeshShape(t, snap)
		}
	}
}

// triKey renders a triangle as a vertex-set key (CCW cyclic order is
// preserved by Snapshot, but the change set and snapshots both come from
// canonicalization, so compare on the sorted vertex triple).
func triKey(tr geom.Triangle) [3]int {
	a := []int{tr.A, tr.B, tr.C}
	for i := 1; i < 3; i++ {
		for j := i; j > 0 && a[j] < a[j-1]; j-- {
			a[j], a[j-1] = a[j-1], a[j]
		}
	}
	return [3]int{a[0], a[1], a[2]}
}

// checkChangeSet verifies: (old \ Removed) ∪ Added == new, exactly.
func checkChangeSet(t *testing.T, before, after *triangulate.Result, ch triangulate.MeshChange) {
	t.Helper()
	oldSet := map[[3]int]bool{}
	for _, tr := range before.Triangles {
		oldSet[triKey(tr)] = true
	}
	newSet := map[[3]int]bool{}
	for _, tr := range after.Triangles {
		newSet[triKey(tr)] = true
	}
	got := map[[3]int]bool{}
	for k := range oldSet {
		got[k] = true
	}
	for _, tr := range ch.Removed {
		k := triKey(tr)
		if !got[k] {
			t.Fatalf("change set removes triangle not present before: %v", k)
		}
		if newSet[k] {
			t.Fatalf("removed triangle %v is present after", k)
		}
		delete(got, k)
	}
	for _, tr := range ch.Added {
		k := triKey(tr)
		if got[k] {
			t.Fatalf("change set adds triangle already present: %v", k)
		}
		if !newSet[k] {
			t.Fatalf("added triangle %v not present after", k)
		}
		got[k] = true
	}
	if len(got) != len(newSet) {
		t.Fatalf("applied change set has %d triangles, new mesh has %d", len(got), len(newSet))
	}
	for k := range newSet {
		if !got[k] {
			t.Fatalf("new-mesh triangle %v not produced by applying change set", k)
		}
	}
}

// assertIncrementalMeshShape checks the structural conforming-mesh
// properties: CCW triangles, manifold edges, hull consistency, Euler and
// area conservation.
func assertIncrementalMeshShape(t *testing.T, res *triangulate.Result) {
	t.Helper()
	n := len(res.Points)
	edgeOwners := map[[2]int]int{}
	for _, tr := range res.Triangles {
		if geom.OrientSign(res.WorkPoints()[tr.A], res.WorkPoints()[tr.B],
			res.WorkPoints()[tr.C]) <= 0 {
			t.Fatalf("triangle %+v not strictly CCW", tr)
		}
		for _, e := range [3][2]int{{tr.A, tr.B}, {tr.B, tr.C}, {tr.C, tr.A}} {
			k := normEdge(e[0], e[1])
			edgeOwners[k]++
			if edgeOwners[k] > 2 {
				t.Fatalf("edge %v has %d owners", k, edgeOwners[k])
			}
		}
	}
	h := len(res.Hull)
	boundary := 0
	for _, c := range edgeOwners {
		if c == 1 {
			boundary++
		}
	}
	if boundary != h {
		t.Fatalf("boundary edges %d != hull ring %d", boundary, h)
	}
	T, E := len(res.Triangles), len(edgeOwners)
	if T-E+n != 1 {
		t.Fatalf("Euler T-E+N=%d", T-E+n)
	}
	if T != 2*n-h-2 {
		t.Fatalf("T=%d want %d", T, 2*n-h-2)
	}
	if !relClose(res.AreaSum(), res.HullArea(), 1e-9) {
		t.Fatalf("area sum %g != hull %g", res.AreaSum(), res.HullArea())
	}
}

func TestMesh_MoveFarTriggersFrameRebuild(t *testing.T) {
	ps, verr := geom.ParsePoints([]geom.Point{
		{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}, {X: 0, Y: 1}, {X: 0.5, Y: 0.5},
	})
	if verr != nil {
		t.Fatal(verr)
	}
	m, gerr := triangulate.NewMesh(ps)
	if gerr != nil {
		t.Fatal(gerr)
	}
	// Move a point far outside the initial super triangle (span 1,
	// margin ~100).
	if _, gerr := m.Move(0, geom.Point{X: 5000, Y: -3000}); gerr != nil {
		t.Fatal(gerr)
	}
	snap := m.Snapshot()
	globalEmptyCircle(t, snap)
	assertIncrementalMeshShape(t, snap)

	// Insert far out too.
	if _, gerr := m.Insert(geom.Point{X: -4000, Y: 4000}); gerr != nil {
		t.Fatal(gerr)
	}
	snap = m.Snapshot()
	globalEmptyCircle(t, snap)
	assertIncrementalMeshShape(t, snap)
}

// Co-circular configurations must remain a legal Delaunay mesh after
// every incremental step.
func TestMesh_CoCircularMoves(t *testing.T) {
	ps, verr := geom.ParsePoints([]geom.Point{
		{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}, {X: 0, Y: 1},
	})
	if verr != nil {
		t.Fatal(verr)
	}
	m, gerr := triangulate.NewMesh(ps)
	if gerr != nil {
		t.Fatal(gerr)
	}
	moves := []geom.Point{
		{X: 0.5, Y: 0.5},
		{X: 2, Y: 2},
		{X: -5, Y: 0.2},
		{X: 0.1, Y: 0.1},
		{X: 3, Y: -3},
	}
	for _, np := range moves {
		if _, gerr := m.Move(2, np); gerr != nil {
			t.Fatalf("move to %v: %v", np, gerr)
		}
		snap := m.Snapshot()
		globalEmptyCircle(t, snap)
		assertIncrementalMeshShape(t, snap)
	}
	if math.IsNaN(m.Snapshot().HullArea()) {
		t.Fatal("nan hull area")
	}
}
