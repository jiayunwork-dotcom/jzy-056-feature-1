package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/session"
)

// ---- request / response DTOs --------------------------------------------

type sessionCreateRequest struct {
	Points []geom.Point `json:"points"`
}

type sessionView struct {
	ID            string          `json:"id"`
	PointCount    int             `json:"point_count"`
	TriangleCount int             `json:"triangle_count"`
	Triangles     []geom.Triangle `json:"triangles"`
	Hull          []int           `json:"hull"`
}

type changeView struct {
	Removed    []geom.Triangle `json:"removed"`
	Added      []geom.Triangle `json:"added"`
	PointCount int             `json:"point_count"`
}

type insertRequest struct {
	Point geom.Point `json:"point"`
}

type moveRequest struct {
	Index int        `json:"index"`
	Point geom.Point `json:"point"`
}

func toSessionView(s *session.Session, snap *session.Snapshot) sessionView {
	return sessionView{
		ID:            s.ID(),
		PointCount:    snap.PointCount,
		TriangleCount: len(snap.Triangles),
		Triangles:     snap.Triangles,
		Hull:          snap.Hull,
	}
}

// ---- handlers -----------------------------------------------------------

func postSessionCreate(c *gin.Context) {
	var req sessionCreateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON of the form {\"points\":[..]}: "+err.Error())
		return
	}
	s, snap, gerr := sessions.Create(req.Points)
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	c.JSON(http.StatusCreated, toSessionView(s, snap))
}

func getSession(c *gin.Context) {
	s, gerr := sessions.Get(c.Param("id"))
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	snap, gerr := sessions.Snapshot(s)
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	c.JSON(http.StatusOK, toSessionView(s, snap))
}

func deleteSession(c *gin.Context) {
	if gerr := sessions.Destroy(c.Param("id")); gerr != nil {
		failGeom(c, gerr)
		return
	}
	c.JSON(http.StatusOK, gin.H{"status": "destroyed"})
}

func postSessionInsert(c *gin.Context) {
	s, gerr := sessions.Get(c.Param("id"))
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	var req insertRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON of the form {\"point\":{\"x\":..,\"y\":..}}: "+err.Error())
		return
	}
	ch, gerr := sessions.Insert(s, req.Point)
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	c.JSON(http.StatusOK, changeView{
		Removed:    nonNilTriangles(ch.Removed),
		Added:      nonNilTriangles(ch.Added),
		PointCount: ch.PointCount,
	})
}

func postSessionMove(c *gin.Context) {
	s, gerr := sessions.Get(c.Param("id"))
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	var req moveRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON of the form "+
				"{\"index\":..,\"point\":{\"x\":..,\"y\":..}}: "+err.Error())
		return
	}
	ch, gerr := sessions.Move(s, req.Index, req.Point)
	if gerr != nil {
		failGeom(c, gerr)
		return
	}
	c.JSON(http.StatusOK, changeView{
		Removed:    nonNilTriangles(ch.Removed),
		Added:      nonNilTriangles(ch.Added),
		PointCount: ch.PointCount,
	})
}

func nonNilTriangles(ts []geom.Triangle) []geom.Triangle {
	if ts == nil {
		return []geom.Triangle{}
	}
	return ts
}

// failGeom maps a kernel/session structured error onto the wire status
// and envelope. Unknown sessions are 404; everything else is 400.
func failGeom(c *gin.Context, e *geom.Error) {
	status := http.StatusBadRequest
	if e.Code == geom.ErrSessionNotFound {
		status = http.StatusNotFound
	}
	fail(c, status, string(e.Code), e.Message)
}
