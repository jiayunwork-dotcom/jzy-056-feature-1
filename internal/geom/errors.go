// Package geom holds the geometry kernel primitives shared by the Delaunay
// triangulation and its Voronoi dual: points, triangles, predicates with
// tolerance handling, point-set validation and convex hull computation.
package geom

import "fmt"

// ErrorCode identifies a class of invalid input or geometry failure.
type ErrorCode string

const (
	ErrTooFewPoints   ErrorCode = "TOO_FEW_POINTS"
	ErrDuplicatePoint ErrorCode = "DUPLICATE_POINT"
	ErrInvalidCoord   ErrorCode = "INVALID_COORDINATE"
	ErrAllCollinear   ErrorCode = "ALL_POINTS_COLLINEAR"
	ErrDegenerate     ErrorCode = "DEGENERATE_GEOMETRY"
)

// Error is the structured error returned by every geometry-kernel entry
// point. The HTTP layer maps Code onto a machine-readable error code and
// returns Message as the human-readable explanation.
type Error struct {
	Code    ErrorCode
	Message string
}

func (e *Error) Error() string {
	return fmt.Sprintf("%s: %s", e.Code, e.Message)
}

func newError(code ErrorCode, format string, args ...any) *Error {
	return &Error{Code: code, Message: fmt.Sprintf(format, args...)}
}
