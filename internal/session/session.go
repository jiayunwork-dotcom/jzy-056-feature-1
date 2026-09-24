// Package session adds a stateful, long-lived layer on top of the
// one-shot Delaunay kernel. A session holds one triangulation and
// applies incremental edits — inserting a point or moving an existing
// one — by reworking only the affected locality in the underlying
// triangulate.Mesh. Each edit returns just the triangles that
// disappeared and appeared, so a client can update its render
// incrementally.
//
// Sessions are created through a Manager, which hands out opaque
// handles. Operations on one handle are processed serially in arrival
// order (the caller guarantees no concurrent use of one session); the
// Manager itself is safe for concurrent use across different sessions.
// Sessions must be explicitly Destroyed to release their memory.
package session

import (
	"crypto/rand"
	"encoding/hex"
	"math"
	"strconv"
	"sync"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
)

// Snapshot is the full state of a session at one point in time, in the
// same shape as the one-shot triangulate response.
type Snapshot struct {
	PointCount int
	Triangles  []geom.Triangle
	Hull       []int
	Points     []geom.Point
}

// Change is the incremental result of one edit: the triangles removed
// and the triangles added by that edit. Applying the removals then the
// additions to the previous triangle set yields the new triangle set.
type Change struct {
	Removed    []geom.Triangle
	Added      []geom.Triangle
	PointCount int
}

// Session is one long-lived triangulation.
type Session struct {
	id   string
	mesh *triangulate.Mesh
	// destroyed is consulted only under the Manager lock; the field also
	// lets a caller holding a *Session check liveness directly.
	destroyed bool
}

// Manager owns the live sessions. It is safe for concurrent use by
// multiple goroutines, though operations on any single session are
// expected to be serialized by the caller.
type Manager struct {
	mu       sync.Mutex
	sessions map[string]*Session
}

// NewManager creates an empty session manager.
func NewManager() *Manager {
	return &Manager{sessions: make(map[string]*Session)}
}

// Create validates the initial point set with the existing one-shot
// checks, builds the triangulation through the shared kernel and returns
// a fresh session handle.
func (m *Manager) Create(raw []geom.Point) (*Session, *Snapshot, *geom.Error) {
	points, verr := geom.ParsePoints(raw)
	if verr != nil {
		return nil, nil, verr
	}
	mesh, gerr := triangulate.NewMesh(points)
	if gerr != nil {
		return nil, nil, gerr
	}
	s := &Session{id: newID(), mesh: mesh}

	m.mu.Lock()
	m.sessions[s.id] = s
	m.mu.Unlock()

	return s, s.snapshot(), nil
}

// Get returns the session for handle id, or a SESSION_NOT_FOUND error if
// no such (live) session exists.
func (m *Manager) Get(id string) (*Session, *geom.Error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	s, ok := m.sessions[id]
	if !ok || s.destroyed {
		return nil, notFound(id)
	}
	return s, nil
}

// Destroy releases the session and its mesh. Destroying an unknown oralready-destroyed handle is a SESSION_NOT_FOUND error.
func (m *Manager) Destroy(id string) *geom.Error {
	m.mu.Lock()
	defer m.mu.Unlock()
	s, ok := m.sessions[id]
	if !ok || s.destroyed {
		return notFound(id)
	}
	s.destroyed = true
	delete(m.sessions, id)
	return nil
}

// Count returns the number of live sessions (mostly useful in tests and
// for observability).
func (m *Manager) Count() int {
	m.mu.Lock()
	defer m.mu.Unlock()
	return len(m.sessions)
}

func notFound(id string) *geom.Error {
	return &geom.Error{
		Code:    geom.ErrSessionNotFound,
		Message: "no live session with handle " + id,
	}
}

func duplicatePoint(i int) *geom.Error {
	return &geom.Error{
		Code:    geom.ErrDuplicatePoint,
		Message: "a point with those exact coordinates already exists",
	}
}

// ID returns the opaque handle of the session.
func (s *Session) ID() string { return s.id }

// Snapshot returns the full current state of the session.
func (m *Manager) Snapshot(s *Session) (*Snapshot, *geom.Error) {
	if gerr := m.live(s); gerr != nil {
		return nil, gerr
	}
	return s.snapshot(), nil
}

func (m *Manager) live(s *Session) *geom.Error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if s.destroyed {
		return notFound(s.id)
	}
	return nil
}

func (s *Session) snapshot() *Snapshot {
	r := s.mesh.Snapshot()
	return &Snapshot{
		PointCount: len(r.Points),
		Triangles:  r.Triangles,
		Hull:       r.Hull,
		Points:     r.Points,
	}
}

// Validate returns "" when the session's current triangulation satisfies
// the global empty-circumcircle property and conforming hull coverage,
// otherwise a description of the first violation. It is the same
// independent post-condition the automated tests pin after every edit.
func (m *Manager) Validate(s *Session) (string, *geom.Error) {
	if gerr := m.live(s); gerr != nil {
		return "", gerr
	}
	return triangulate.GlobalCheck(s.mesh.Snapshot()), nil
}

// MeshResult exposes the kernel-level result of a session for in-process
// callers that want the same verification primitives as the one-shot
// path (notably the tests).
func (s *Session) MeshResult() *triangulate.Result { return s.mesh.Snapshot() }

// Insert adds a new point. It rejects a coordinate identical to an
// existing point (DUPLICATE_POINT) without modifying the mesh.
func (m *Manager) Insert(s *Session, p geom.Point) (*Change, *geom.Error) {
	if gerr := validateCoord(p); gerr != nil {
		return nil, gerr
	}
	if gerr := m.live(s); gerr != nil {
		return nil, gerr
	}
	if s.mesh.HasPointAt(p) {
		return nil, duplicatePoint(0)
	}
	ch, gerr := s.mesh.Insert(p)
	if gerr != nil {
		return nil, gerr
	}
	return &Change{
		Removed:    ch.Removed,
		Added:      ch.Added,
		PointCount: s.mesh.PointCount(),
	}, nil
}

// Move relocates existing point index i to p. It rejects a move to a
// coordinate identical to another existing point (DUPLICATE_POINT) and a
// non-existent index (POINT_NOT_FOUND), without modifying the mesh.
func (m *Manager) Move(s *Session, i int, p geom.Point) (*Change, *geom.Error) {
	if gerr := validateCoord(p); gerr != nil {
		return nil, gerr
	}
	if gerr := m.live(s); gerr != nil {
		return nil, gerr
	}
	if i < 0 || i >= s.mesh.PointCount() {
		return nil, &geom.Error{
			Code:    geom.ErrPointNotFound,
			Message: "no point with index " + strconv.Itoa(i),
		}
	}
	// Duplicate against every OTHER point. Comparing to i itself is
	// allowed (a move to the point's own coordinates is a no-op edit and
	// changes nothing).
	for j := 0; j < s.mesh.PointCount(); j++ {
		if j != i && s.mesh.OriginalPoint(j) == p {
			return nil, duplicatePoint(j)
		}
	}
	ch, gerr := s.mesh.Move(i, p)
	if gerr != nil {
		return nil, gerr
	}
	return &Change{
		Removed:    ch.Removed,
		Added:      ch.Added,
		PointCount: s.mesh.PointCount(),
	}, nil
}

func validateCoord(p geom.Point) *geom.Error {
	if math.IsNaN(p.X) || math.IsNaN(p.Y) ||
		math.IsInf(p.X, 0) || math.IsInf(p.Y, 0) {
		return &geom.Error{Code: geom.ErrInvalidCoord, Message: "coordinate is NaN or infinite"}
	}
	return nil
}

// newID returns an opaque, unguessable session handle.
func newID() string {
	var b [16]byte
	_, _ = rand.Read(b[:])
	return hex.EncodeToString(b[:])
}
