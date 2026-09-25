// Package session holds the long-lived incremental Delaunay sessions on
// top of the stateful triangulation kernel. It is deliberately kept
// separate from the one-shot compute path: a session owns a mesh that
// persists across edits and a registry of handles.
//
// Edits on one handle are serialised through that handle's own mutex;
// the registry is independently locked. A session can be explicitly
// destroyed to release its memory.
package session

import (
	"math"
	"sync"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
)

// Manager creates and tracks sessions by opaque handle. The zero value
// is ready to use.
type Manager struct {
	mu       sync.Mutex
	next     uint64
	sessions map[uint64]*Session
}

// NewManager returns an empty session registry.
func NewManager() *Manager {
	return &Manager{sessions: make(map[uint64]*Session)}
}

// Session is one long-lived triangulation.
type Session struct {
	id     uint64
	mu     sync.Mutex
	mesh   *triangulate.Mesh
	closed bool
}

// Create validates the initial point set (reusing the one-shot kernel's
// rules), builds the mesh and returns a handle. Too-few points, duplicates
// and all-collinear inputs are rejected exactly as on the stateless path.
func (mng *Manager) Create(points []geom.Point) (uint64, *geom.Error) {
	pts, verr := geom.ParsePoints(points)
	if verr != nil {
		return 0, verr
	}
	mesh, gerr := triangulate.NewMesh(pts)
	if gerr != nil {
		return 0, gerr
	}
	mng.mu.Lock()
	mng.next++
	id := mng.next
	s := &Session{id: id, mesh: mesh}
	mng.sessions[id] = s
	mng.mu.Unlock()
	return id, nil
}

// exists reports whether handle refers to a live session (registry lock
// held by the caller).
func (mng *Manager) exists(id uint64) bool {
	_, ok := mng.sessions[id]
	return ok
}

// ErrNotFound is returned for an unknown or destroyed handle.
func ErrNotFound() *geom.Error {
	return &geom.Error{Code: geom.ErrSessionNotFound, Message: "session not found"}
}

// Insert adds a point to the session, returning the point's stable id and
// the local change set. A coordinate coincident with an existing point is
// rejected and leaves the mesh untouched.
func (mng *Manager) Insert(handle uint64, p geom.Point) (int, triangulate.ChangeSet, *geom.Error) {
	s, gerr := mng.get(handle)
	if gerr != nil {
		return 0, triangulate.ChangeSet{}, gerr
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if e := validatePoint(s, p); e != nil {
		return 0, triangulate.ChangeSet{}, e
	}
	id, cs, ierr := s.mesh.Insert(p)
	if ierr != nil {
		return 0, triangulate.ChangeSet{}, ierr
	}
	return id, cs, nil
}

// Move relocates an existing point atomically, returning the local change
// set. An unknown point id is rejected; a coincident target is rejected
// and the mesh is left in its pre-move state.
func (mng *Manager) Move(handle uint64, id int, p geom.Point) (triangulate.ChangeSet, *geom.Error) {
	s, gerr := mng.get(handle)
	if gerr != nil {
		return triangulate.ChangeSet{}, gerr
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, ok := s.mesh.PointAt(id); !ok {
		return triangulate.ChangeSet{},
			&geom.Error{Code: geom.ErrPointNotFound, Message: "point id does not exist"}
	}
	if e := validatePoint(s, p, id); e != nil {
		return triangulate.ChangeSet{}, e
	}
	return s.mesh.Move(id, p)
}

// Snapshot returns the current stateless triangulation of a session.
func (mng *Manager) Snapshot(handle uint64) (*triangulate.Result, *geom.Error) {
	s, gerr := mng.get(handle)
	if gerr != nil {
		return nil, gerr
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.mesh.Snapshot()
}

// Destroy releases a session. Destroying an unknown/closed handle returns
// SESSION_NOT_FOUND; otherwise the session is removed and its memory freed.
func (mng *Manager) Destroy(handle uint64) *geom.Error {
	mng.mu.Lock()
	s, ok := mng.sessions[handle]
	if !ok {
		mng.mu.Unlock()
		return ErrNotFound()
	}
	delete(mng.sessions, handle)
	mng.mu.Unlock()

	s.mu.Lock()
	s.closed = true
	s.mesh = nil
	s.mu.Unlock()
	return nil
}

// get fetches a live session under the registry lock.
func (mng *Manager) get(handle uint64) (*Session, *geom.Error) {
	mng.mu.Lock()
	s, ok := mng.sessions[handle]
	mng.mu.Unlock()
	if !ok {
		return nil, ErrNotFound()
	}
	return s, nil
}

// validatePoint rejects non-finite coordinates and coordinates coincident
// with another live point; exclude is the point id being moved (which may
// keep its own coordinate) and is absent for insertion.
func validatePoint(s *Session, p geom.Point, exclude ...int) *geom.Error {
	if math.IsNaN(p.X) || math.IsInf(p.X, 0) || math.IsNaN(p.Y) || math.IsInf(p.Y, 0) {
		return &geom.Error{Code: geom.ErrInvalidCoord, Message: "coordinate is NaN or infinite"}
	}
	ex := -1
	if len(exclude) > 0 {
		ex = exclude[0]
	}
	n := s.mesh.PointCount()
	for i := 0; i < n; i++ {
		if i == ex {
			continue
		}
		q, ok := s.mesh.PointAt(i)
		if ok && q == p {
			return &geom.Error{Code: geom.ErrDuplicatePoint, Message: "point coincides with an existing point"}
		}
	}
	return nil
}
