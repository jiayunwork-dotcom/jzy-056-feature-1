package api_test

import (
	"bytes"
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"

	"delaunaysvc/internal/api"
	"delaunaysvc/internal/sample"
)

func newRouter() *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	api.Register(r)
	return r
}

func doJSON(t *testing.T, r http.Handler, method, path string, body any) (int, map[string]any) {
	t.Helper()
	var buf bytes.Buffer
	if body != nil {
		if err := json.NewEncoder(&buf).Encode(body); err != nil {
			t.Fatal(err)
		}
	}
	req := httptest.NewRequest(method, path, &buf)
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)

	var out map[string]any
	if w.Body.Len() > 0 {
		if err := json.Unmarshal(w.Body.Bytes(), &out); err != nil {
			t.Fatalf("response not JSON (%d): %s", w.Code, w.Body.String())
		}
	}
	return w.Code, out
}

func p(x, y float64) map[string]any { return map[string]any{"x": x, "y": y} }

func TestHealthz(t *testing.T) {
	r := newRouter()
	code, body := doJSON(t, r, http.MethodGet, "/healthz", nil)
	if code != http.StatusOK || body["status"] != "ok" {
		t.Fatalf("code=%d body=%v", code, body)
	}
}

func TestTriangulate_Success(t *testing.T) {
	r := newRouter()
	body := map[string]any{"points": []map[string]any{p(0, 0), p(2, 0), p(2, 2), p(0, 2)}}
	code, resp := doJSON(t, r, http.MethodPost, "/api/v1/triangulate", body)
	if code != http.StatusOK {
		t.Fatalf("code=%d body=%v", code, resp)
	}
	if resp["is_delaunay"] != true {
		t.Fatalf("is_delaunay = %v", resp["is_delaunay"])
	}
	if int(resp["triangle_count"].(float64)) != 2 {
		t.Fatalf("triangle_count = %v", resp["triangle_count"])
	}
	if int(resp["hull_edge_count"].(float64)) != 4 {
		t.Fatalf("hull_edge_count = %v", resp["hull_edge_count"])
	}
	if math.Abs(resp["hull_area"].(float64)-4) > 1e-9 {
		t.Fatalf("hull_area = %v", resp["hull_area"])
	}
	if math.Abs(resp["triangle_area_sum"].(float64)-4) > 1e-9 {
		t.Fatalf("triangle_area_sum = %v", resp["triangle_area_sum"])
	}
	tris := resp["triangles"].([]any)
	if len(tris) != 2 {
		t.Fatalf("triangles len = %d", len(tris))
	}
	// No ghost index may ever appear.
	for _, tr := range tris {
		m := tr.(map[string]any)
		for _, k := range []string{"a", "b", "c"} {
			idx := int(m[k].(float64))
			if idx < 0 || idx > 3 {
				t.Fatalf("ghost index %d leaked: %v", idx, m)
			}
		}
	}
}

func TestTriangulate_RejectedCases(t *testing.T) {
	r := newRouter()
	cases := []struct {
		name string
		body any
		code string
	}{
		{"empty", map[string]any{"points": []any{}}, "TOO_FEW_POINTS"},
		{"two points", map[string]any{"points": []map[string]any{p(0, 0), p(1, 0)}}, "TOO_FEW_POINTS"},
		{"duplicate", map[string]any{"points": []map[string]any{p(0, 0), p(1, 0), p(0, 1), p(1, 0)}}, "DUPLICATE_POINT"},
		{"collinear", map[string]any{"points": []map[string]any{p(0, 0), p(1, 0), p(2, 0), p(3, 0)}}, "ALL_POINTS_COLLINEAR"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			code, resp := doJSON(t, r, http.MethodPost, "/api/v1/triangulate", tc.body)
			if code != http.StatusBadRequest {
				t.Fatalf("status = %d, want 400; body=%v", code, resp)
			}
			if resp["code"] != tc.code {
				t.Fatalf("error code = %v, want %s (message: %v)", resp["code"], tc.code, resp["message"])
			}
			if resp["message"] == nil || resp["message"] == "" {
				t.Fatal("error message missing")
			}
		})
	}
}

// NaN and Infinity are not legal JSON numbers at all, so an HTTP client
// can only deliver them as malformed tokens; the JSON layer must reject
// those cleanly with an error envelope rather than crashing. The
// kernel-level INVALID_COORD path (NaN/Inf arriving programmatically) is
// covered by the geom package tests.
func TestTriangulate_NonFiniteTokensRejected(t *testing.T) {
	r := newRouter()
	for _, token := range []string{"NaN", "Infinity", "-Infinity", "1e999"} {
		raw := `{"points":[{"x":0,"y":0},{"x":1,"y":0},{"x":` + token + `,"y":1}]}`
		req := httptest.NewRequest(http.MethodPost, "/api/v1/triangulate", bytes.NewBufferString(raw))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		r.ServeHTTP(w, req)
		if w.Code != http.StatusBadRequest {
			t.Fatalf("token %s: status = %d, want 400", token, w.Code)
		}
		var resp map[string]any
		if err := json.Unmarshal(w.Body.Bytes(), &resp); err != nil {
			t.Fatalf("token %s: non-JSON error body: %s", token, w.Body.String())
		}
		if resp["code"] != "INVALID_JSON" {
			t.Fatalf("token %s: code = %v, want INVALID_JSON", token, resp["code"])
		}
	}
}

func TestTriangulate_MalformedJSON(t *testing.T) {
	r := newRouter()
	req := httptest.NewRequest(http.MethodPost, "/api/v1/triangulate", bytes.NewBufferString("{not json"))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	if w.Code != http.StatusBadRequest {
		t.Fatalf("status = %d", w.Code)
	}
	var resp map[string]any
	if err := json.Unmarshal(w.Body.Bytes(), &resp); err != nil {
		t.Fatal(err)
	}
	if resp["code"] != "INVALID_JSON" {
		t.Fatalf("code = %v", resp["code"])
	}
}

func TestVoronoi_Success(t *testing.T) {
	r := newRouter()
	body := map[string]any{"points": []map[string]any{p(0, 0), p(4, 0), p(4, 4), p(0, 4), p(2, 2)}}
	code, resp := doJSON(t, r, http.MethodPost, "/api/v1/voronoi", body)
	if code != http.StatusOK {
		t.Fatalf("code=%d body=%v", code, resp)
	}
	// Center point fan: 4 triangles -> 4 Voronoi vertices.
	if int(resp["vertex_count"].(float64)) != 4 {
		t.Fatalf("vertex_count = %v", resp["vertex_count"])
	}
	verts := resp["vertices"].([]any)
	for _, v := range verts {
		m := v.(map[string]any)
		pt := m["point"].(map[string]any)
		// All circumcenters in this symmetric configuration are at
		// distance sqrt(2) from two corners and (2,2); sanity check only
		// finite coordinates come back.
		if math.IsNaN(pt["x"].(float64)) || math.IsNaN(pt["y"].(float64)) {
			t.Fatal("NaN voronoi vertex")
		}
	}
	// Hull is the square -> 4 unbounded rays, but they are omitted by
	// default from the payload (ray_count still reports the count).
	if int(resp["ray_count"].(float64)) != 4 {
		t.Fatalf("ray_count = %v", resp["ray_count"])
	}
	if len(resp["rays"].([]any)) != 0 {
		t.Fatal("rays should be omitted unless requested")
	}
}

func TestVoronoi_IncludeRays(t *testing.T) {
	r := newRouter()
	body := map[string]any{
		"points":       []map[string]any{p(0, 0), p(4, 0), p(4, 4), p(0, 4)},
		"include_rays": true,
	}
	_, resp := doJSON(t, r, http.MethodPost, "/api/v1/voronoi", body)
	rays := resp["rays"].([]any)
	if len(rays) != 4 {
		t.Fatalf("rays len = %d, want 4", len(rays))
	}
	for _, r0 := range rays {
		m := r0.(map[string]any)
		dir := m["direction"].(map[string]any)
		norm := math.Hypot(dir["x"].(float64), dir["y"].(float64))
		if math.Abs(norm-1) > 1e-12 {
			t.Fatalf("ray direction not unit: %v", norm)
		}
	}
}

func TestVoronoi_RejectsInvalid(t *testing.T) {
	r := newRouter()
	code, resp := doJSON(t, r, http.MethodPost, "/api/v1/voronoi",
		map[string]any{"points": []map[string]any{p(0, 0), p(1, 1)}})
	if code != http.StatusBadRequest || resp["code"] != "TOO_FEW_POINTS" {
		t.Fatalf("code=%d body=%v", code, resp)
	}
}

// The built-in reference case: fetch the sample payload, triangulate it
// through the live endpoint and pin the Euler-derived triangle count.
func TestSampleEndpointAndEulerCount(t *testing.T) {
	r := newRouter()
	code, resp := doJSON(t, r, http.MethodGet, "/api/v1/sample", nil)
	if code != http.StatusOK {
		t.Fatalf("sample status = %d", code)
	}
	if int(resp["point_count"].(float64)) != sample.Size {
		t.Fatalf("point_count = %v", resp["point_count"])
	}
	if int(resp["expected_triangles"].(float64)) != sample.ExpectedTriangles {
		t.Fatalf("expected_triangles = %v", resp["expected_triangles"])
	}

	// Feed the returned points straight back into /triangulate.
	rawPts := resp["points"].([]any)
	pts := make([]map[string]any, len(rawPts))
	for i, rp := range rawPts {
		q := rp.(map[string]any)
		pts[i] = p(q["x"].(float64), q["y"].(float64))
	}
	code2, tri := doJSON(t, r, http.MethodPost, "/api/v1/triangulate",
		map[string]any{"points": pts})
	if code2 != http.StatusOK {
		t.Fatalf("triangulate sample: %d %v", code2, tri)
	}
	if int(tri["triangle_count"].(float64)) != sample.ExpectedTriangles {
		t.Fatalf("triangle_count = %v, pinned %d",
			tri["triangle_count"], sample.ExpectedTriangles)
	}
	if tri["is_delaunay"] != true {
		t.Fatal("sample triangulation not Delaunay")
	}
	if int(tri["hull_edge_count"].(float64)) != sample.ExpectedHullEdges {
		t.Fatalf("hull_edge_count = %v, want %d",
			tri["hull_edge_count"], sample.ExpectedHullEdges)
	}
}

// End-to-end translation invariance: same point set at two positions
// returns identical triangle index triples and correspondingly shifted
// Voronoi centers.
func TestEndToEndTranslationInvariance(t *testing.T) {
	r := newRouter()
	base := []map[string]any{p(0, 0), p(4, 0), p(4, 3), p(0, 3), p(2, 1.5)}
	shift := 2.5e6
	moved := make([]map[string]any, len(base))
	for i, q := range base {
		moved[i] = p(q["x"].(float64)+shift, q["y"].(float64)-shift)
	}

	_, t0 := doJSON(t, r, http.MethodPost, "/api/v1/triangulate", map[string]any{"points": base})
	_, t1 := doJSON(t, r, http.MethodPost, "/api/v1/triangulate", map[string]any{"points": moved})
	tris0 := t0["triangles"].([]any)
	tris1 := t1["triangles"].([]any)
	if len(tris0) != len(tris1) {
		t.Fatalf("triangle counts differ: %d vs %d", len(tris0), len(tris1))
	}
	// Both responses are canonically sorted, so compare index triples.
	for i := range tris0 {
		a := tris0[i].(map[string]any)
		b := tris1[i].(map[string]any)
		for _, k := range []string{"a", "b", "c"} {
			if int(a[k].(float64)) != int(b[k].(float64)) {
				t.Fatalf("topology differs at triangle %d field %s", i, k)
			}
		}
	}

	_, v0 := doJSON(t, r, http.MethodPost, "/api/v1/voronoi", map[string]any{"points": base})
	_, v1 := doJSON(t, r, http.MethodPost, "/api/v1/voronoi", map[string]any{"points": moved})
	c0 := v0["vertices"].([]any)
	c1 := v1["vertices"].([]any)
	if len(c0) != len(c1) {
		t.Fatalf("voronoi vertex counts differ")
	}
	for i := range c0 {
		a := c0[i].(map[string]any)["point"].(map[string]any)
		b := c1[i].(map[string]any)["point"].(map[string]any)
		dx := b["x"].(float64) - a["x"].(float64)
		dy := b["y"].(float64) - a["y"].(float64)
		if math.Abs(dx-shift) > 1e-4 || math.Abs(dy+shift) > 1e-4 {
			t.Fatalf("vertex %d did not covary: (%g,%g)", i, dx, dy)
		}
	}
}
