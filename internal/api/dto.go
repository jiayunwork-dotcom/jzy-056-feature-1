package api

import (
	"delaunaysvc/internal/geom"
	"delaunaysvc/internal/sample"
	"delaunaysvc/internal/triangulate"
	"delaunaysvc/internal/voronoi"
)

// PointsRequest is the shared request body of the geometry endpoints.
type PointsRequest struct {
	Points []geom.Point `json:"points"`
}

// ErrorResponse is the only error envelope ever returned.
type ErrorResponse struct {
	Code    string `json:"code"`
	Message string `json:"message"`
}

// TriangulateResponse is returned by POST /api/v1/triangulate.
type TriangulateResponse struct {
	PointCount      int                     `json:"point_count"`
	TriangleCount   int                     `json:"triangle_count"`
	Triangles       []geom.Triangle         `json:"triangles"`
	Hull            []int                   `json:"hull"`
	HullEdgeCount   int                     `json:"hull_edge_count"`
	HullArea        float64                 `json:"hull_area"`
	TriangleAreaSum float64                 `json:"triangle_area_sum"`
	IsDelaunay      bool                    `json:"is_delaunay"`
	Violations      []triangulate.Violation `json:"violations,omitempty"`
}

// voronoiVertexDTO mirrors voronoi.Vertex on the wire.
type voronoiVertexDTO struct {
	Index int        `json:"index"`
	Point geom.Point `json:"point"`
}

// voronoiEdgeDTO mirrors voronoi.FiniteEdge on the wire.
type voronoiEdgeDTO struct {
	U int `json:"u"`
	V int `json:"v"`
}

// voronoiRayDTO mirrors voronoi.Ray on the wire.
type voronoiRayDTO struct {
	Origin    int        `json:"origin"`
	Direction geom.Point `json:"direction"`
	HullU     int        `json:"hull_u"`
	HullV     int        `json:"hull_v"`
}

// VoronoiResponse is returned by POST /api/v1/voronoi.
type VoronoiResponse struct {
	PointCount  int                `json:"point_count"`
	Triangles   []geom.Triangle    `json:"triangles"`
	VertexCount int                `json:"vertex_count"`
	Vertices    []voronoiVertexDTO `json:"vertices"`
	EdgeCount   int                `json:"edge_count"`
	Edges       []voronoiEdgeDTO   `json:"edges"`
	RayCount    int                `json:"ray_count"`
	Rays        []voronoiRayDTO    `json:"rays"`
	IncludeRays bool               `json:"include_rays"`
}

// SampleResponse is returned by GET /api/v1/sample.
type SampleResponse struct {
	Description       string       `json:"description"`
	Points            []geom.Point `json:"points"`
	PointCount        int          `json:"point_count"`
	ExpectedTriangles int          `json:"expected_triangles"`
	ExpectedHullEdges int          `json:"expected_hull_edges"`
	Formula           string       `json:"euler_formula"`
}

func newSampleResponse() SampleResponse {
	return SampleResponse{
		Description: "4x4 regular grid with deterministic perturbations of " +
			"interior points; boundary points are fixed so the hull stays the " +
			"12-edge square boundary.",
		Points:            sample.Grid(),
		PointCount:        sample.Size,
		ExpectedTriangles: sample.ExpectedTriangles,
		ExpectedHullEdges: sample.ExpectedHullEdges,
		Formula:           "T = 2N - H - 2",
	}
}

func toVoronoiResponse(d *voronoi.Diagram, tr *triangulate.Result, includeRays bool) VoronoiResponse {
	vs := make([]voronoiVertexDTO, len(d.Vertices))
	for i, v := range d.Vertices {
		vs[i] = voronoiVertexDTO{Index: v.Index, Point: v.Point}
	}
	es := make([]voronoiEdgeDTO, len(d.Edges))
	for i, e := range d.Edges {
		es[i] = voronoiEdgeDTO{U: e.U, V: e.V}
	}
	rs := []voronoiRayDTO{}
	if includeRays {
		rs = make([]voronoiRayDTO, len(d.Rays))
		for i, r := range d.Rays {
			rs[i] = voronoiRayDTO{
				Origin:    r.Origin,
				Direction: r.Direction,
				HullU:     r.HullU,
				HullV:     r.HullV,
			}
		}
	}
	return VoronoiResponse{
		PointCount:  len(tr.Points),
		Triangles:   tr.Triangles,
		VertexCount: len(vs),
		Vertices:    vs,
		EdgeCount:   len(es),
		Edges:       es,
		RayCount:    len(d.Rays),
		Rays:        rs,
		IncludeRays: includeRays,
	}
}
