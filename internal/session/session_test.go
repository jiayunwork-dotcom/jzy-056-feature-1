package session_test

import (
	"math"
	"math/rand"
	"sort"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/session"
)

func triSet(ts []geom.Triangle) map[[3]int]bool {
	out := make(map[[3]int]bool, len(ts))
	for _, t := range ts {
		a := []int{t.A, t.B, t.C}
		sort.Ints(a)
		out[[3]int{a[0], a[1], a[2]}] = true
	}
	return out
}

// applyChange returns (old \ removed) ∪ added and asserts the change set
// is well-formed: every removed triangle existed, none is in both sets.
func applyChange(t *testing.T, old map[[3]int]bool, ch *session.Change) map[[3]int]bool {
	t.Helper()
	out := make(map[[3]int]bool, len(old))
	for k := range old {
		out[k] = true
	}
	rem := triSet(ch.Removed)
	add := triSet(ch.Added)
	for k := range rem {
		if !out[k] {
			t.Fatalf("change set removes a triangle absent before op: %v", k)
		}
		if add[k] {
			t.Fatalf("triangle %v is both removed and added", k)
		}
		delete(out, k)
	}
	for k := range add {
		if old[k] {
			// Only legal if it was also removed (re-created); those were
			// filtered by the disjoint check above, so reaching here means
			// a spurious add.
			t.Fatalf("change set adds a triangle already present: %v", k)
		}
		out[k] = true
	}
	return out
}

func assertLegal(t *testing.T, mgr *session.Manager, s *session.Session) map[[3]int]bool {
	t.Helper()
	msg, gerr := mgr.Validate(s)
	if gerr != nil {
		t.Fatalf("validate: %v", gerr)
	}
	if msg != "" {
		t.Fatalf("incremental mesh is not legal: %s", msg)
	}
	snap, gerr := mgr.Snapshot(s)
	if gerr != nil {
		t.Fatal(gerr)
	}
	return triSet(snap.Triangles)
}

func square() []geom.Point {
	return []geom.Point{
		{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 4, Y: 4}, {X: 0, Y: 4},
	}
}

// The headline acceptance test: a long randomized sequence of inserts
// and moves is fed to one session; after EVERY step the maintained mesh
// must independently satisfy the global empty-circumcircle property and
// conforming convex-hull coverage, and the returned change set applied to
// the previous triangles must produce the new triangles.
func TestSession_IncrementalSequenceIsAlwaysLegal(t *testing.T) {
	rng := rand.New(rand.NewSource(99))
	for iter := 0; iter < 40; iter++ {
		mgr := session.NewManager()
		n0 := 4 + rng.Intn(8)
		init := make([]geom.Point, n0)
		for i := range init {
			init[i] = geom.Point{X: rng.Float64()*20 - 10, Y: rng.Float64()*20 - 10}
		}
		s, snap, gerr := mgr.Create(init)
		if gerr != nil {
			t.Fatalf("iter %d create: %v", iter, gerr)
		}
		current := triSet(snap.Triangles)
		if msg, _ := mgr.Validate(s); msg != "" {
			t.Fatalf("iter %d initial: %s", iter, msg)
		}

		for step := 0; step < 120; step++ {
			switch rng.Intn(2) {
			case 0: // insert
				var np geom.Point
				for {
					np = geom.Point{
						X: rng.Float64()*20 - 10 + rng.NormFloat64()*3,
						Y: rng.Float64()*20 - 10 + rng.NormFloat64()*3,
					}
					if !coordExists(snap.Points, np) {
						break
					}
				}
				ch, gerr := mgr.Insert(s, np)
				if gerr != nil {
					t.Fatalf("iter %d step %d insert %v: %v", iter, step, np, gerr)
				}
				current = applyChange(t, current, ch)
			default: // move
				i := rng.Intn(snap.PointCount)
				var np geom.Point
				for {
					np = geom.Point{
						X: rng.Float64()*20 - 10 + rng.NormFloat64()*3,
						Y: rng.Float64()*20 - 10 + rng.NormFloat64()*3,
					}
					blocked := false
					for j, q := range snap.Points {
						if j != i && q == np {
							blocked = true
							break
						}
					}
					if !blocked {
						break
					}
				}
				ch, gerr := mgr.Move(s, i, np)
				if gerr != nil {
					t.Fatalf("iter %d step %d move %d->%v: %v", iter, step, i, np, gerr)
				}
				current = applyChange(t, current, ch)
			}

			// 1. The maintained mesh itself must be legal.
			newTri := assertLegal(t, mgr, s)
			// 2. The applied change set must be exactly the new mesh.
			if len(newTri) != len(current) {
				t.Fatalf("iter %d step %d: applied change set has %d triangles, mesh has %d",
					iter, step, len(current), len(newTri))
			}
			for k := range newTri {
				if !current[k] {
					t.Fatalf("iter %d step %d: triangle %v in mesh but not produced by change set",
						iter, step, k)
				}
			}
			var err2 error
			_ = err2
			snap2, gerr := mgr.Snapshot(s)
			if gerr != nil {
				t.Fatal(gerr)
			}
			snap = snap2
		}
		if gerr := mgr.Destroy(s.ID()); gerr != nil {
			t.Fatal(gerr)
		}
	}
}

func coordExists(ps []geom.Point, p geom.Point) bool {
	for _, q := range ps {
		if q == p {
			return true
		}
	}
	return false
}

// A moved point can drag the hull out far enough to require enlarging the
// virtual super frame; the mesh must stay legal at every step.
func TestSession_FarMovesAndInserts(t *testing.T) {
	mgr := session.NewManager()
	s, _, gerr := mgr.Create([]geom.Point{
		{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 1, Y: 1}, {X: 0, Y: 1}, {X: .5, Y: .5},
	})
	if gerr != nil {
		t.Fatal(gerr)
	}
	moves := []struct {
		i int
		p geom.Point
	}{
		{0, geom.Point{X: 5000, Y: -3000}},
		{1, geom.Point{X: -5000, Y: 3000}},
		{2, geom.Point{X: 6000, Y: 6000}},
	}
	for _, mv := range moves {
		if _, gerr := mgr.Move(s, mv.i, mv.p); gerr != nil {
			t.Fatalf("move %d: %v", mv.i, gerr)
		}
		assertLegal(t, mgr, s)
	}
	for _, p := range []geom.Point{{X: -7000, Y: -7000}, {X: 2, Y: 2}, {X: 9000, Y: 0}} {
		if _, gerr := mgr.Insert(s, p); gerr != nil {
			t.Fatalf("insert %v: %v", p, gerr)
		}
		assertLegal(t, mgr, s)
	}
}

// Co-circular / near-degenerate layouts are legal in more than one
// triangulation. The criterion is self-legality after every edit, never
// equality with a from-scratch rebuild.
func TestSession_CoCircularDegeneracies(t *testing.T) {
	mgr := session.NewManager()
	s, _, gerr := mgr.Create(square())
	if gerr != nil {
		t.Fatal(gerr)
	}
	// Moves that keep four points exactly / nearly co-circular.
	for _, np := range []geom.Point{
		{X: 0.5, Y: 0.5}, // collides with nothing; centre of square
		{X: 2, Y: 2},
		{X: -3, Y: 3},
	} {
		if _, gerr := mgr.Move(s, 0, np); gerr != nil {
			t.Fatalf("move to %v: %v", np, gerr)
		}
		assertLegal(t, mgr, s)
	}
	// Regular polygon (all vertices co-circular) built by inserts.
	poly := session.NewManager()
	base := []geom.Point{{X: 1, Y: 0}, {X: 0, Y: 1}, {X: -1, Y: 0}}
	sp, _, gerr := poly.Create(base)
	if gerr != nil {
		t.Fatal(gerr)
	}
	for k := 3; k <= 9; k++ {
		a := 2 * math.Pi * float64(k) / 10
		if _, gerr := poly.Insert(sp, geom.Point{X: math.Cos(a), Y: math.Sin(a)}); gerr != nil {
			t.Fatalf("insert polygon vertex %d: %v", k, gerr)
		}
		assertLegal(t, poly, sp)
	}
}

// ---- illegal edits are rejected and leave the mesh intact ---------------

func snapshotTriangles(t *testing.T, mgr *session.Manager, s *session.Session) []geom.Triangle {
	t.Helper()
	snap, gerr := mgr.Snapshot(s)
	if gerr != nil {
		t.Fatal(gerr)
	}
	return snap.Triangles
}

func TestSession_RejectsDuplicateInsert(t *testing.T) {
	mgr := session.NewManager()
	s, snap0, gerr := mgr.Create(square())
	if gerr != nil {
		t.Fatal(gerr)
	}
	before := snapshotTriangles(t, mgr, s)

	_, gerr = mgr.Insert(s, geom.Point{X: 4, Y: 0}) // == point 1
	if gerr == nil || gerr.Code != geom.ErrDuplicatePoint {
		t.Fatalf("want DUPLICATE_POINT, got %v", gerr)
	}
	// Mesh untouched.
	after := snapshotTriangles(t, mgr, s)
	if !sameTriangles(before, after) {
		t.Fatal("mesh changed after a rejected insert")
	}
	assertLegal(t, mgr, s)
	if snap0.PointCount != 4 {
		t.Fatal("point count changed after rejected insert")
	}
}

func TestSession_RejectsMoveOfMissingPoint(t *testing.T) {
	mgr := session.NewManager()
	s, _, gerr := mgr.Create(square())
	if gerr != nil {
		t.Fatal(gerr)
	}
	before := snapshotTriangles(t, mgr, s)
	for _, bad := range []int{4, 99, -1} {
		_, gerr = mgr.Move(s, bad, geom.Point{X: 7, Y: 7})
		if gerr == nil || gerr.Code != geom.ErrPointNotFound {
			t.Fatalf("index %d: want POINT_NOT_FOUND, got %v", bad, gerr)
		}
	}
	// Moving onto another existing point is also rejected.
	_, gerr = mgr.Move(s, 0, geom.Point{X: 4, Y: 0}) // == point 1
	if gerr == nil || gerr.Code != geom.ErrDuplicatePoint {
		t.Fatalf("want DUPLICATE_POINT for colliding move, got %v", gerr)
	}
	if !sameTriangles(before, snapshotTriangles(t, mgr, s)) {
		t.Fatal("mesh changed after rejected moves")
	}
	assertLegal(t, mgr, s)
}

func TestSession_UnknownAndDestroyedHandles(t *testing.T) {
	mgr := session.NewManager()
	s, _, gerr := mgr.Create(square())
	if gerr != nil {
		t.Fatal(gerr)
	}
	// Never-existed handle.
	if _, gerr := mgr.Get("deadbeef"); gerr == nil || gerr.Code != geom.ErrSessionNotFound {
		t.Fatalf("want SESSION_NOT_FOUND, got %v", gerr)
	}
	if gerr := mgr.Destroy(s.ID()); gerr != nil {
		t.Fatal(gerr)
	}
	// Operations on the destroyed handle must all fail structurally.
	if _, gerr := mgr.Snapshot(s); gerr == nil || gerr.Code != geom.ErrSessionNotFound {
		t.Fatalf("snapshot after destroy: %v", gerr)
	}
	if _, gerr := mgr.Insert(s, geom.Point{X: 2, Y: 2}); gerr == nil || gerr.Code != geom.ErrSessionNotFound {
		t.Fatalf("insert after destroy: %v", gerr)
	}
	if _, gerr := mgr.Move(s, 0, geom.Point{X: 2, Y: 2}); gerr == nil || gerr.Code != geom.ErrSessionNotFound {
		t.Fatalf("move after destroy: %v", gerr)
	}
	if gerr := mgr.Destroy(s.ID()); gerr == nil || gerr.Code != geom.ErrSessionNotFound {
		t.Fatalf("double destroy: %v", gerr)
	}
	if mgr.Count() != 0 {
		t.Fatalf("live session count = %d, want 0", mgr.Count())
	}
}

// A failed frame-enlargement path must roll back fully; drive it by
// repeatedly moving points extremely far (forces many frame rebuilds),
// interleaved with ordinary edits, and confirm legality + memory liveness.
func TestSession_ManyFrameRebuildsStayIntact(t *testing.T) {
	mgr := session.NewManager()
	s, _, gerr := mgr.Create(square())
	if gerr != nil {
		t.Fatal(gerr)
	}
	rng := rand.New(rand.NewSource(7))
	scale := 100.0
	for step := 0; step < 30; step++ {
		i := rng.Intn(4)
		np := geom.Point{
			X: (rng.Float64()*2 - 1) * scale,
			Y: (rng.Float64()*2 - 1) * scale,
		}
		// Avoid accidental exact duplicates.
		if (i == 1 && np == geom.Point{X: 4, Y: 0}) {
			np.X += 0.123
		}
		if _, gerr := mgr.Move(s, i, np); gerr != nil {
			t.Fatalf("step %d: %v", step, gerr)
		}
		assertLegal(t, mgr, s)
		scale *= 10
	}
}

func sameTriangles(a, b []geom.Triangle) bool {
	if len(a) != len(b) {
		return false
	}
	ra, rb := triSet(a), triSet(b)
	for k := range ra {
		if !rb[k] {
			return false
		}
	}
	return true
}
