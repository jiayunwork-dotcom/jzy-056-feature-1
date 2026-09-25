package api_test

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"

	"delaunaysvc/internal/api"
)

func newSessionRouter(t *testing.T) (*gin.Engine, *api.SessionRouter) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	r := gin.New()
	sr := api.NewSessionRouter()
	sr.Register(r)
	return r, sr
}

func doSessJSON(t *testing.T, r http.Handler, method, path string, body any) (int, map[string]any) {
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
		_ = json.Unmarshal(w.Body.Bytes(), &out)
	}
	return w.Code, out
}

func TestSessionHTTPLifecycle(t *testing.T) {
	r, _ := newSessionRouter(t)

	// Create.
	code, body := doSessJSON(t, r, http.MethodPost, "/api/v1/sessions",
		map[string]any{"points": []map[string]float64{
			{"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 4}, {"x": 0, "y": 4}}})
	if code != http.StatusCreated {
		t.Fatalf("create status %d body %v", code, body)
	}
	sid, _ := body["session_id"].(float64)
	if sid == 0 {
		t.Fatalf("missing session id: %v", body)
	}
	id := uint64(sid)

	// Status.
	if code, body := doSessJSON(t, r, http.MethodGet, pathSession(id), nil); code != http.StatusOK {
		t.Fatalf("get status %d %v", code, body)
	}

	// Insert a point; response carries a change set.
	code, body = doSessJSON(t, r, http.MethodPost, pathInsert(id),
		map[string]any{"x": 2, "y": 2})
	if code != http.StatusOK {
		t.Fatalf("insert status %d body %v", code, body)
	}
	if pid, _ := body["point_id"].(float64); pid != 4 {
		t.Fatalf("expected point_id 4, got %v", body["point_id"])
	}
	ch, _ := body["changes"].(map[string]any)
	if ch == nil {
		t.Fatal("missing changes")
	}
	added, _ := ch["added"].([]any)
	removed, _ := ch["removed"].([]any)
	if len(added) != 4 || len(removed) != 2 {
		t.Fatalf("interior insertion should add 4 remove 2, got add=%d rem=%d",
			len(added), len(removed))
	}

	// Move point 4; status stays legal.
	code, body = doSessJSON(t, r, http.MethodPost, pathMove(id),
		map[string]any{"point_id": 4, "x": 3, "y": 1})
	if code != http.StatusOK {
		t.Fatalf("move status %d body %v", code, body)
	}

	// Destroy then access => 404.
	if code, _ := doSessJSON(t, r, http.MethodDelete, pathSession(id), nil); code != http.StatusOK {
		t.Fatalf("destroy status %d", code)
	}
	if code, body := doSessJSON(t, r, http.MethodGet, pathSession(id), nil); code != http.StatusNotFound {
		t.Fatalf("expected 404 after destroy, got %d %v", code, body)
	} else if body["code"] != "SESSION_NOT_FOUND" {
		t.Fatalf("expected SESSION_NOT_FOUND, got %v", body["code"])
	}
}

func TestSessionHTTPRejections(t *testing.T) {
	r, _ := newSessionRouter(t)

	// Too few points.
	code, body := doSessJSON(t, r, http.MethodPost, "/api/v1/sessions",
		map[string]any{"points": []map[string]float64{{"x": 0, "y": 0}}})
	if code != http.StatusBadRequest || body["code"] != "TOO_FEW_POINTS" {
		t.Fatalf("create validation: %d %v", code, body)
	}

	// Create a valid session.
	code, body = doSessJSON(t, r, http.MethodPost, "/api/v1/sessions",
		map[string]any{"points": []map[string]float64{
			{"x": 0, "y": 0}, {"x": 3, "y": 0}, {"x": 0, "y": 3}}})
	if code != http.StatusCreated {
		t.Fatalf("create: %d %v", code, body)
	}
	id := uint64(body["session_id"].(float64))

	// Duplicate insert.
	code, body = doSessJSON(t, r, http.MethodPost, pathInsert(id),
		map[string]any{"x": 3, "y": 0})
	if code != http.StatusBadRequest || body["code"] != "DUPLICATE_POINT" {
		t.Fatalf("duplicate insert: %d %v", code, body)
	}

	// Move unknown point.
	code, body = doSessJSON(t, r, http.MethodPost, pathMove(id),
		map[string]any{"point_id": 99, "x": 1, "y": 1})
	if code != http.StatusBadRequest || body["code"] != "POINT_NOT_FOUND" {
		t.Fatalf("move unknown: %d %v", code, body)
	}

	// Move onto an existing point.
	code, body = doSessJSON(t, r, http.MethodPost, pathMove(id),
		map[string]any{"point_id": 2, "x": 0, "y": 0})
	if code != http.StatusBadRequest || body["code"] != "DUPLICATE_POINT" {
		t.Fatalf("move coincident: %d %v", code, body)
	}

	// Operations on a handle that never existed.
	code, body = doSessJSON(t, r, http.MethodPost, "/api/v1/sessions/555/insert",
		map[string]any{"x": 1, "y": 1})
	if code != http.StatusNotFound || body["code"] != "SESSION_NOT_FOUND" {
		t.Fatalf("missing session insert: %d %v", code, body)
	}
	code, body = doSessJSON(t, r, http.MethodPost, "/api/v1/sessions/555/move",
		map[string]any{"point_id": 0, "x": 1, "y": 1})
	if code != http.StatusNotFound || body["code"] != "SESSION_NOT_FOUND" {
		t.Fatalf("missing session move: %d %v", code, body)
	}

	// Malformed JSON.
	req := httptest.NewRequest(http.MethodPost, pathInsert(id), strings.NewReader("{not json"))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	if w.Code != http.StatusBadRequest {
		t.Fatalf("malformed JSON status %d", w.Code)
	}
}

func pathSession(id uint64) string { return "/api/v1/sessions/" + uitoa(id) }
func pathInsert(id uint64) string  { return "/api/v1/sessions/" + uitoa(id) + "/insert" }
func pathMove(id uint64) string    { return "/api/v1/sessions/" + uitoa(id) + "/move" }

func uitoa(u uint64) string {
	if u == 0 {
		return "0"
	}
	var b []byte
	for u > 0 {
		b = append([]byte{byte('0' + u%10)}, b...)
		u /= 10
	}
	return string(b)
}
