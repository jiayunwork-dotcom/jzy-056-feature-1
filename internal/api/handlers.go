package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/triangulate"
	"delaunaysvc/internal/voronoi"
)

// Register mounts the service routes on r.
func Register(r *gin.Engine) {
	r.GET("/healthz", healthz)
	r.GET("/api/v1/sample", getSample)
	r.POST("/api/v1/triangulate", postTriangulate)
	r.POST("/api/v1/voronoi", postVoronoi)
}

func healthz(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"status": "ok"})
}

func getSample(c *gin.Context) {
	c.JSON(http.StatusOK, newSampleResponse())
}

func postTriangulate(c *gin.Context) {
	var req PointsRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON of the form {\"points\":[{\"x\":..,\"y\":..}]}: "+err.Error())
		return
	}

	points, verr := geom.ParsePoints(req.Points)
	if verr != nil {
		failError(c, verr)
		return
	}

	res, terr := triangulate.Build(points)
	if terr != nil {
		failError(c, terr)
		return
	}
	ok, violations := res.IsDelaunay()
	if violations == nil {
		violations = []triangulate.Violation{}
	}

	c.JSON(http.StatusOK, TriangulateResponse{
		PointCount:      len(res.Points),
		TriangleCount:   len(res.Triangles),
		Triangles:       res.Triangles,
		Hull:            res.Hull,
		HullEdgeCount:   len(res.Hull),
		HullArea:        res.HullArea(),
		TriangleAreaSum: res.AreaSum(),
		IsDelaunay:      ok,
		Violations:      violations,
	})
}

func postVoronoi(c *gin.Context) {
	var req struct {
		Points      []geom.Point `json:"points"`
		IncludeRays bool         `json:"include_rays"`
	}
	if err := c.ShouldBindJSON(&req); err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON",
			"request body must be JSON of the form {\"points\":[..]}: "+err.Error())
		return
	}

	points, verr := geom.ParsePoints(req.Points)
	if verr != nil {
		failError(c, verr)
		return
	}

	res, terr := triangulate.Build(points)
	if terr != nil {
		failError(c, terr)
		return
	}
	diagram := voronoi.Build(res)

	c.JSON(http.StatusOK, toVoronoiResponse(diagram, res, req.IncludeRays))
}

func fail(c *gin.Context, status int, code, msg string) {
	c.AbortWithStatusJSON(status, ErrorResponse{Code: code, Message: msg})
}

func failError(c *gin.Context, e *geom.Error) {
	fail(c, http.StatusBadRequest, string(e.Code), e.Message)
}
