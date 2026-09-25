package api

import (
	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
)

// ---- session create -----------------------------------------------------

type SessionCreateRequest struct {
	Points []geom.Point `json:"points"`
}

type SessionCreateResponse struct {
	SessionID  uint64          `json:"session_id"`
	PointCount int             `json:"point_count"`
	Triangles  []geom.Triangle `json:"triangles"`
	Hull       []int           `json:"hull"`
}

// ---- session status -----------------------------------------------------

type SessionStatusResponse struct {
	SessionID  uint64          `json:"session_id"`
	PointCount int             `json:"point_count"`
	TriCount   int             `json:"triangle_count"`
	Hull       []int           `json:"hull"`
	Triangles  []geom.Triangle `json:"triangles"`
	IsDelaunay bool            `json:"is_delaunay"`
}

// ---- session edits ------------------------------------------------------

type SessionInsertRequest struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

type SessionMoveRequest struct {
	PointID int     `json:"point_id"`
	X       float64 `json:"x"`
	Y       float64 `json:"y"`
}

// ChangeSetResponse is the local triangle delta returned by an edit.
type ChangeSetResponse struct {
	Removed []geom.Triangle `json:"removed"`
	Added   []geom.Triangle `json:"added"`
}

type SessionInsertResponse struct {
	SessionID uint64            `json:"session_id"`
	PointID   int               `json:"point_id"`
	Changes   ChangeSetResponse `json:"changes"`
}

type SessionMoveResponse struct {
	SessionID uint64            `json:"session_id"`
	PointID   int               `json:"point_id"`
	Changes   ChangeSetResponse `json:"changes"`
}

func toChangeSet(c triangulate.ChangeSet) ChangeSetResponse {
	return ChangeSetResponse{Removed: c.Removed, Added: c.Added}
}
