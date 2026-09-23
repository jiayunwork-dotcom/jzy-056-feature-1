package geom

import "math"

// Point is a planar input point. It is also used directly as the wire
// format for points in request/response bodies.
type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Triangle is a counter-clockwise oriented triangle whose fields are
// indices into the input point slice. It is the wire representation of a
// triangle returned to clients.
type Triangle struct {
	A int `json:"a"`
	B int `json:"b"`
	C int `json:"c"`
}

// det2 returns the 2x2 determinant |a b; c d|.
func det2(a, b, c, d float64) float64 {
	return a*d - b*c
}

// Orient2D returns twice the signed area of triangle (p, q, r).
// Positive means p->q->r is counter-clockwise, zero means collinear.
func Orient2D(p, q, r Point) float64 {
	return det2(q.X-p.X, q.Y-p.Y, r.X-p.X, r.Y-p.Y)
}

// Area2 returns the unsigned double area of triangle (p, q, r).
func Area2(p, q, r Point) float64 {
	return math.Abs(Orient2D(p, q, r))
}

// SignedArea returns the ordinary signed area of triangle (p, q, r).
func SignedArea(p, q, r Point) float64 {
	return Orient2D(p, q, r) * 0.5
}

// dist2 returns the squared Euclidean distance between p and q.
func dist2(p, q Point) float64 {
	dx := q.X - p.X
	dy := q.Y - p.Y
	return dx*dx + dy*dy
}
