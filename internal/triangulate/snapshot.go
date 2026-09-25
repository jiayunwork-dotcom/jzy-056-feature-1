package triangulate

import "delaunaysvc/internal/geom"

// Snapshot exports the current stateful mesh as an ordinary, stateless
// Result: ghost triangles shaved off, triangles canonicalized, the hull
// ring derived from the one-owner edges of the finished soup — exactly
// what a one-shot Build returns for the same point set. All geometry
// consumers (the independent empty-circle check, hull/area accounting)
// work from the internal work coordinates via Result.WorkPoints.
func (m *Mesh) Snapshot() (*Result, *geom.Error) {
	final := m.realTriangles()
	hull, herr := boundaryRing(final)
	if herr != nil {
		return nil, herr
	}
	return &Result{
		Points:    m.Points,
		work:      append([]geom.Point(nil), m.work...),
		Triangles: final,
		Hull:      hull,
		Origin:    m.Origin,
	}, nil
}
