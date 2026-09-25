package session_test

import (
	"sync"
	"testing"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/session"
)

// Distinct sessions are fully independent and are driven concurrently; the
// same handle is additionally hammered serially through its own mutex.
func TestConcurrentSessions(t *testing.T) {
	mng := session.NewManager()
	const nSessions = 8
	ids := make([]uint64, nSessions)
	for i := range ids {
		id, err := mng.Create([]geom.Point{
			{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 4, Y: 4}, {X: 0, Y: 4},
		})
		if err != nil {
			t.Fatal(err)
		}
		ids[i] = id
	}

	var wg sync.WaitGroup
	for s := 0; s < nSessions; s++ {
		wg.Add(1)
		go func(sid uint64, seed int) {
			defer wg.Done()
			for k := 0; k < 40; k++ {
				p := geom.Point{X: float64(seed + k), Y: float64(seed*10 + k)}
				// Insertions with the same handle are serialised by the
				// session; collisions just get rejected.
				_, _, _ = mng.Insert(sid, p)
			}
			res, err := mng.Snapshot(sid)
			if err != nil {
				t.Errorf("snapshot: %v", err)
				return
			}
			assertMesh(t, "concurrent", res)
		}(ids[s], s)
	}
	wg.Wait()

	// Every session still destroys cleanly.
	for _, id := range ids {
		if err := mng.Destroy(id); err != nil {
			t.Fatalf("destroy %d: %v", id, err)
		}
	}
}
