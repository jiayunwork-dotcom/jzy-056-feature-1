package session_test

import (
	"math"
	"math/rand"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/session"
	"delaunaysvc/internal/triangulate"
)

// keyTri canonicalizes a triangle index triple into a comparable key.
func keyTri(t geom.Triangle) [3]int {
	s := [3]int{t.A, t.B, t.C}
	if s[0] > s[1] {
		s[0], s[1] = s[1], s[0]
	}
	if s[1] > s[2] {
		s[1], s[2] = s[2], s[1]
	}
	if s[0] > s[1] {
		s[0], s[1] = s[1], s[0]
	}
	return s
}

func relClose(a, b float64) bool {
	d := math.Abs(a - b)
	den := math.Max(math.Abs(a), math.Abs(b))
	if den == 0 {
		return true
	}
	return d/den <= 1e-9
}

// assertMesh checks the four hard postconditions on a session snapshot:
// global empty circumcircle, exact hull coverage, manifold edge
// ownership, and the Euler count.
func assertMesh(t *testing.T, label string, res *triangulate.Result) {
	t.Helper()
	n := len(res.Points)

	owners := map[[2]int]int{}
	for _, tr := range res.Triangles {
		for _, v := range [3]int{tr.A, tr.B, tr.C} {
			if v < 0 || v >= n {
				t.Fatalf("%s: triangle references out-of-range index %d", label, v)
			}
		}
		wp := res.WorkPoints()
		if geom.OrientSign(wp[tr.A], wp[tr.B], wp[tr.C]) <= 0 {
			t.Fatalf("%s: non-CCW triangle %+v", label, tr)
		}
		for _, e := range [3][2]int{{tr.A, tr.B}, {tr.B, tr.C}, {tr.C, tr.A}} {
			a, b := e[0], e[1]
			if a > b {
				a, b = b, a
			}
			owners[[2]int{a, b}]++
		}
	}
	for e, c := range owners {
		if c < 1 || c > 2 {
			t.Fatalf("%s: edge %v owned by %d triangles", label, e, c)
		}
	}

	// Global empty circumcircle: O(T*N).
	wp := res.WorkPoints()
	for ti, tr := range res.Triangles {
		a, b, c := wp[tr.A], wp[tr.B], wp[tr.C]
		for p := 0; p < n; p++ {
			if p == tr.A || p == tr.B || p == tr.C {
				continue
			}
			if geom.InCircleSign(a, b, c, wp[p]) > 0 {
				t.Fatalf("%s: triangle %d %+v circumcircle contains point %d",
					label, ti, tr, p)
			}
		}
	}

	// Hull coverage: triangle area sum equals independent convex-hull area.
	sum := 0.0
	for _, tr := range res.Triangles {
		sum += geom.SignedArea(res.Points[tr.A], res.Points[tr.B], res.Points[tr.C])
	}
	hull := geom.ConvexHull(res.Points)
	hullArea := geom.PolygonArea(res.Points, hull)
	if !relClose(sum, hullArea) {
		t.Fatalf("%s: triangle sum %.12g != hull area %.12g", label, sum, hullArea)
	}

	// One-owner edges == hull edges.
	hullSet := map[[2]int]bool{}
	for i := range hull {
		a, b := hull[i], hull[(i+1)%len(hull)]
		if a > b {
			a, b = b, a
		}
		hullSet[[2]int{a, b}] = true
	}
	for e, c := range owners {
		if (c == 1) != hullSet[e] {
			t.Fatalf("%s: boundary edge %v mismatches hull", label, e)
		}
	}

	// Euler: T = 2N - H - 2.
	H := len(hull)
	if want := 2*n - H - 2; len(res.Triangles) != want {
		t.Fatalf("%s: T=%d want %d (N=%d H=%d)", label, len(res.Triangles), want, n, H)
	}
}

func newSession(t *testing.T, pts []geom.Point) (*session.Manager, uint64) {
	t.Helper()
	mng := session.NewManager()
	id, err := mng.Create(pts)
	if err != nil {
		t.Fatalf("create: %v", err)
	}
	return mng, id
}

func TestCreateValidation(t *testing.T) {
	mng := session.NewManager()
	if _, err := mng.Create([]geom.Point{{X: 0, Y: 0}, {X: 1, Y: 0}}); err == nil || err.Code != geom.ErrTooFewPoints {
		t.Fatalf("expected TOO_FEW_POINTS, got %v", err)
	}
	if _, err := mng.Create([]geom.Point{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 0}}); err == nil || err.Code != geom.ErrDuplicatePoint {
		t.Fatalf("expected DUPLICATE_POINT, got %v", err)
	}
	if _, err := mng.Create([]geom.Point{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 2, Y: 0}}); err == nil || err.Code != geom.ErrAllCollinear {
		t.Fatalf("expected ALL_POINTS_COLLINEAR, got %v", err)
	}
}

func TestInsertChainLegal(t *testing.T) {
	rng := rand.New(rand.NewSource(11))
	pts := []geom.Point{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 10, Y: 10}, {X: 0, Y: 10}}
	mng, id := newSession(t, pts)
	prev := triSet(mustSnap(t, mng, id))
	for step := 0; step < 150; step++ {
		p := geom.Point{X: rng.Float64() * 10, Y: rng.Float64() * 10}
		pid, cs, err := mng.Insert(id, p)
		if err != nil {
			t.Fatalf("step %d insert: %v", step, err)
		}
		if pid != 4+step {
			t.Fatalf("step %d: expected id %d got %d", step, 4+step, pid)
		}
		res, _ := mng.Snapshot(id)
		assertChangeSetApplied(t, prev, cs, res)
		assertMesh(t, "insert "+itoa(step), res)
		prev = triSet(res)
	}
}

func TestMoveChainLegal(t *testing.T) {
	rng := rand.New(rand.NewSource(23))
	pts := []geom.Point{{X: 0, Y: 0}, {X: 8, Y: 0}, {X: 8, Y: 8}, {X: 0, Y: 8}, {X: 2, Y: 2}, {X: 6, Y: 2}, {X: 6, Y: 6}, {X: 2, Y: 6}}
	mng, id := newSession(t, pts)
	assertMesh(t, "seed", mustSnap(t, mng, id))
	prev := triSet(mustSnap(t, mng, id))
	for step := 0; step < 300; step++ {
		pid := rng.Intn(len(pts))
		p := geom.Point{X: rng.Float64() * 8, Y: rng.Float64() * 8}
		cs, err := mng.Move(id, pid, p)
		if err != nil {
			t.Fatalf("step %d move: %v", step, err)
		}
		pts[pid] = p
		res, _ := mng.Snapshot(id)
		assertChangeSetApplied(t, prev, cs, res)
		if step%5 == 0 {
			assertMesh(t, "move "+itoa(step), res)
		}
		prev = triSet(res)
	}
	assertMesh(t, "move-final", mustSnap(t, mng, id))
}

func TestMixedRoamingLegal(t *testing.T) {
	rng := rand.New(rand.NewSource(99))
	pts := []geom.Point{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 1}}
	mng, id := newSession(t, pts)
	for step := 0; step < 200; step++ {
		if rng.Intn(2) == 0 {
			scale := math.Pow(1.5, float64(rng.Intn(20)))
			p := geom.Point{X: (rng.Float64()*2 - 1) * scale, Y: (rng.Float64()*2 - 1) * scale}
			dup := false
			res, _ := mng.Snapshot(id)
			for _, q := range res.Points {
				if q == p {
					dup = true
				}
			}
			if dup {
				continue
			}
			if _, _, err := mng.Insert(id, p); err != nil {
				t.Fatalf("step %d insert: %v", step, err)
			}
		} else {
			res, _ := mng.Snapshot(id)
			pid := rng.Intn(len(res.Points))
			scale := math.Pow(1.4, float64(rng.Intn(18)))
			p := geom.Point{X: (rng.Float64()*2 - 1) * scale, Y: (rng.Float64()*2 - 1) * scale}
			if _, err := mng.Move(id, pid, p); err != nil {
				t.Fatalf("step %d move: %v", step, err)
			}
		}
		if step%10 == 0 {
			assertMesh(t, "mixed "+itoa(step), mustSnap(t, mng, id))
		}
	}
	assertMesh(t, "mixed-final", mustSnap(t, mng, id))
}

// triSet returns the set of canonical triangle keys in a snapshot.
func triSet(res *triangulate.Result) map[[3]int]bool {
	s := make(map[[3]int]bool, len(res.Triangles))
	for _, tr := range res.Triangles {
		s[keyTri(tr)] = true
	}
	return s
}

// assertChangeSetApplied verifies (old \\ removed) ∪ added == new and that
// removed triangles were present and added triangles were absent before.
func assertChangeSetApplied(t *testing.T, old map[[3]int]bool, cs triangulate.ChangeSet, now *triangulate.Result) {
	t.Helper()
	got := make(map[[3]int]bool, len(old))
	for k := range old {
		got[k] = true
	}
	for _, tr := range cs.Removed {
		k := keyTri(tr)
		if !old[k] {
			t.Fatalf("change set removes a triangle not in the old mesh: %v", k)
		}
		delete(got, k)
	}
	for _, tr := range cs.Added {
		k := keyTri(tr)
		if old[k] {
			t.Fatalf("change set adds a triangle already in the old mesh: %v", k)
		}
		if got[k] {
			t.Fatalf("change set adds duplicate triangle %v", k)
		}
		got[k] = true
	}
	want := triSet(now)
	if len(got) != len(want) {
		t.Fatalf("applied change set gives %d triangles, mesh has %d", len(got), len(want))
	}
	for k := range want {
		if !got[k] {
			t.Fatalf("mesh triangle %v not produced by applying change set", k)
		}
	}
	for k := range got {
		if !want[k] {
			t.Fatalf("applied change set left triangle %v absent from mesh", k)
		}
	}
}

func mustSnap(t *testing.T, mng *session.Manager, id uint64) *triangulate.Result {
	t.Helper()
	res, err := mng.Snapshot(id)
	if err != nil {
		t.Fatal(err)
	}
	return res
}

// ---- rejection cases ----------------------------------------------------

func TestRejectDuplicateInsert(t *testing.T) {
	pts := []geom.Point{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 0, Y: 2}}
	mng, id := newSession(t, pts)
	res, _ := mng.Snapshot(id)
	before := len(res.Triangles)
	_, _, err := mng.Insert(id, geom.Point{X: 2, Y: 0})
	if err == nil || err.Code != geom.ErrDuplicatePoint {
		t.Fatalf("expected DUPLICATE_POINT, got %v", err)
	}
	res, _ = mng.Snapshot(id)
	if len(res.Triangles) != before {
		t.Fatal("mesh changed after rejected duplicate insert")
	}
	assertMesh(t, "after-dup-insert", res)
}

func TestRejectMoveUnknownPoint(t *testing.T) {
	pts := []geom.Point{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 0, Y: 2}}
	mng, id := newSession(t, pts)
	res, _ := mng.Snapshot(id)
	before := res.Triangles
	_, err := mng.Move(id, 99, geom.Point{X: 5, Y: 5})
	if err == nil || err.Code != geom.ErrPointNotFound {
		t.Fatalf("expected POINT_NOT_FOUND, got %v", err)
	}
	res, _ = mng.Snapshot(id)
	if len(res.Triangles) != len(before) {
		t.Fatal("mesh changed after rejected move of unknown point")
	}
	assertMesh(t, "after-move-unknown", res)
}

func TestRejectMoveOntoAnotherPoint(t *testing.T) {
	pts := []geom.Point{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 0, Y: 2}, {X: 3, Y: 3}}
	mng, id := newSession(t, pts)
	res, _ := mng.Snapshot(id)
	before := len(res.Triangles)
	_, err := mng.Move(id, 3, geom.Point{X: 0, Y: 0})
	if err == nil || err.Code != geom.ErrDuplicatePoint {
		t.Fatalf("expected DUPLICATE_POINT, got %v", err)
	}
	res, _ = mng.Snapshot(id)
	if len(res.Triangles) != before {
		t.Fatal("mesh changed after rejected coincident move")
	}
	if res.Points[3] != (geom.Point{X: 3, Y: 3}) {
		t.Fatalf("rejected move changed point 3 coordinate: %v", res.Points[3])
	}
	assertMesh(t, "after-move-coincident", res)
}

func TestSessionNotFound(t *testing.T) {
	mng := session.NewManager()
	if _, err := mng.Snapshot(12345); err == nil || err.Code != geom.ErrSessionNotFound {
		t.Fatalf("expected SESSION_NOT_FOUND, got %v", err)
	}
	if _, _, err := mng.Insert(12345, geom.Point{X: 0, Y: 0}); err == nil || err.Code != geom.ErrSessionNotFound {
		t.Fatalf("expected SESSION_NOT_FOUND on insert, got %v", err)
	}
	if _, err := mng.Move(12345, 0, geom.Point{X: 1, Y: 1}); err == nil || err.Code != geom.ErrSessionNotFound {
		t.Fatalf("expected SESSION_NOT_FOUND on move, got %v", err)
	}
	if err := mng.Destroy(12345); err == nil || err.Code != geom.ErrSessionNotFound {
		t.Fatalf("expected SESSION_NOT_FOUND on destroy, got %v", err)
	}
}

func TestDestroyReleasesSession(t *testing.T) {
	pts := []geom.Point{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 0, Y: 2}}
	mng, id := newSession(t, pts)
	if err := mng.Destroy(id); err != nil {
		t.Fatal(err)
	}
	if _, err := mng.Snapshot(id); err == nil || err.Code != geom.ErrSessionNotFound {
		t.Fatalf("session still usable after destroy: %v", err)
	}
	// Double destroy is rejected cleanly.
	if err := mng.Destroy(id); err == nil || err.Code != geom.ErrSessionNotFound {
		t.Fatalf("expected SESSION_NOT_FOUND on double destroy, got %v", err)
	}
}

func TestNoOpMoveReturnsEmptyChangeSet(t *testing.T) {
	pts := []geom.Point{{X: 0, Y: 0}, {X: 5, Y: 0}, {X: 5, Y: 5}, {X: 0, Y: 5}, {X: 2, Y: 2}}
	mng, id := newSession(t, pts)
	res, _ := mng.Snapshot(id)
	q := res.Points[2]
	cs, err := mng.Move(id, 2, q)
	if err != nil {
		t.Fatal(err)
	}
	if len(cs.Removed) != 0 || len(cs.Added) != 0 {
		t.Fatalf("same-coordinate move produced a change set: %+v", cs)
	}
	assertMesh(t, "noop-move", mustSnap(t, mng, id))
}

func itoa(i int) string {
	if i == 0 {
		return "0"
	}
	neg := i < 0
	if neg {
		i = -i
	}
	var b []byte
	for i > 0 {
		b = append([]byte{byte('0' + i%10)}, b...)
		i /= 10
	}
	if neg {
		b = append([]byte{'-'}, b...)
	}
	return string(b)
}
