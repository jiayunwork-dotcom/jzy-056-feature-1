package triangulate

import (
	"math"

	"delaunaysvc/internal/geom"
)

// cellKey identifies a uniform-grid cell.
type cellKey struct{ i, j int64 }

// forceLinearAll routes every triangle through the oversize list,
// making candidate lookup an exhaustive linear scan. It is a debug switch
// used to separate spatial-index defects from kernel defects.
var forceLinearAll = false

// workTri is a construction-time triangle with a stable identity. IDs
// are never reused: deleted triangles stay in the backing slice with
// alive=false so that adjacency and spatial-index references remain
// unambiguous while building.
type workTri struct {
	a, b, c int
	alive   bool
	// cells are the uniform-grid buckets this triangle was filed under
	// (empty when it is on the always-tested oversize list).
	cells []cellKey
	over  bool
}

// spatialIndex is the acceleration structure rebuilt neither per
// insertion nor ever: buckets are maintained incrementally as triangles
// are born and die.
type spatialIndex struct {
	minX, minY, cell float64
	buckets          map[cellKey][]int
	oversize         []int // triangles spanning too many cells
	// maxSpan caps how many cells a triangle may occupy in one
	// dimension before being moved to the oversize list.
	maxSpan int64
}

func newSpatialIndex(minX, minY, spanX, spanY float64, n int) *spatialIndex {
	cell := math.Sqrt(spanX*spanY/float64(n)) * 1.5
	if cell <= 0 || math.IsNaN(cell) || math.IsInf(cell, 1) {
		cell = math.Max(spanX, spanY)
		if cell <= 0 {
			cell = 1
		}
	}
	return &spatialIndex{
		minX:    minX,
		minY:    minY,
		cell:    cell,
		buckets: make(map[cellKey][]int, 4*n),
		maxSpan: 16,
	}
}

func (s *spatialIndex) cellOf(p geom.Point) cellKey {
	return cellKey{
		i: int64(math.Floor((p.X - s.minX) / s.cell)),
		j: int64(math.Floor((p.Y - s.minY) / s.cell)),
	}
}

// insertTriangle files a (living) triangle under every cell overlapped
// by its axis-aligned bounding box. Triangles whose box spans too many
// cells (the large ghost triangles hugging the super-triangle) go to the
// oversize list instead, which is consulted on every query.
func (s *spatialIndex) insertTriangle(id int, t workTri, coords []geom.Point) workTri {
	pa, pb, pc := coords[t.a], coords[t.b], coords[t.c]
	x0 := math.Min(pa.X, math.Min(pb.X, pc.X))
	x1 := math.Max(pa.X, math.Max(pb.X, pc.X))
	y0 := math.Min(pa.Y, math.Min(pb.Y, pc.Y))
	y1 := math.Max(pa.Y, math.Max(pb.Y, pc.Y))

	c0 := s.cellOf(geom.Point{X: x0, Y: y0})
	c1 := s.cellOf(geom.Point{X: x1, Y: y1})
	if forceLinearAll || c1.i-c0.i > s.maxSpan || c1.j-c0.j > s.maxSpan {
		t.over = true
		s.oversize = append(s.oversize, id)
		return t
	}
	for i := c0.i; i <= c1.i; i++ {
		for j := c0.j; j <= c1.j; j++ {
			k := cellKey{i, j}
			s.buckets[k] = append(s.buckets[k], id)
			t.cells = append(t.cells, k)
		}
	}
	return t
}

// removeIn drops dead triangle id from every bucket (or the oversize
// list) it occupied.
func (s *spatialIndex) removeIn(id int, t workTri) {
	if t.over {
		out := s.oversize[:0]
		for _, x := range s.oversize {
			if x != id {
				out = append(out, x)
			}
		}
		s.oversize = out
		return
	}
	for _, k := range t.cells {
		b := s.buckets[k]
		out := b[:0]
		for _, x := range b {
			if x != id {
				out = append(out, x)
			}
		}
		s.buckets[k] = out
	}
}

// candidates returns every triangle filed in p's cell plus every
// oversize triangle: this is a superset of the triangles whose
// (bounding-box-covered) interior or circumdisk could contain p.
func (s *spatialIndex) candidates(p geom.Point) []int {
	out := append([]int(nil), s.buckets[s.cellOf(p)]...)
	out = append(out, s.oversize...)
	return out
}
