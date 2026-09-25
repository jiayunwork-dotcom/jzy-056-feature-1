package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/session"
)

// SessionRouter groups the incremental-session endpoints over a shared
// session Manager.
type SessionRouter struct {
	Sessions *session.Manager
}

// NewSessionRouter builds a router with a fresh session registry.
func NewSessionRouter() *SessionRouter {
	return &SessionRouter{Sessions: session.NewManager()}
}

// Register mounts the session routes under /api/v1/sessions.
func (sr *SessionRouter) Register(r *gin.Engine) {
	r.POST("/api/v1/sessions", sr.createSession)
	r.GET("/api/v1/sessions/:id", sr.getSession)
	r.POST("/api/v1/sessions/:id/insert", sr.insertPoint)
	r.POST("/api/v1/sessions/:id/move", sr.movePoint)
	r.DELETE("/api/v1/sessions/:id", sr.destroySession)
}

func (sr *SessionRouter) createSession(c *gin.Context) {
	var req SessionCreateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON of the form {\"points\":[{\"x\":..,\"y\":..}]}")
		return
	}
	id, gerr := sr.Sessions.Create(req.Points)
	if gerr != nil {
		failError(c, gerr)
		return
	}
	res, _ := sr.Sessions.Snapshot(id)
	c.JSON(http.StatusCreated, SessionCreateResponse{
		SessionID:  id,
		PointCount: len(res.Points),
		Triangles:  res.Triangles,
		Hull:       res.Hull,
	})
}

func (sr *SessionRouter) getSession(c *gin.Context) {
	id, ok := parseID(c)
	if !ok {
		return
	}
	res, gerr := sr.Sessions.Snapshot(id)
	if gerr != nil {
		failError(c, gerr)
		return
	}
	isD, _ := res.IsDelaunay()
	c.JSON(http.StatusOK, SessionStatusResponse{
		SessionID:  id,
		PointCount: len(res.Points),
		TriCount:   len(res.Triangles),
		Hull:       res.Hull,
		Triangles:  res.Triangles,
		IsDelaunay: isD,
	})
}

func (sr *SessionRouter) insertPoint(c *gin.Context) {
	id, ok := parseID(c)
	if !ok {
		return
	}
	var req SessionInsertRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON with x and y")
		return
	}
	pid, cs, gerr := sr.Sessions.Insert(id, geom.Point{X: req.X, Y: req.Y})
	if gerr != nil {
		failError(c, gerr)
		return
	}
	c.JSON(http.StatusOK, SessionInsertResponse{
		SessionID: id,
		PointID:   pid,
		Changes:   toChangeSet(cs),
	})
}

func (sr *SessionRouter) movePoint(c *gin.Context) {
	id, ok := parseID(c)
	if !ok {
		return
	}
	var req SessionMoveRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON with point_id, x and y")
		return
	}
	cs, gerr := sr.Sessions.Move(id, req.PointID, geom.Point{X: req.X, Y: req.Y})
	if gerr != nil {
		failError(c, gerr)
		return
	}
	c.JSON(http.StatusOK, SessionMoveResponse{
		SessionID: id,
		PointID:   req.PointID,
		Changes:   toChangeSet(cs),
	})
}

func (sr *SessionRouter) destroySession(c *gin.Context) {
	id, ok := parseID(c)
	if !ok {
		return
	}
	if gerr := sr.Sessions.Destroy(id); gerr != nil {
		failError(c, gerr)
		return
	}
	c.JSON(http.StatusOK, gin.H{"destroyed": true, "session_id": id})
}
