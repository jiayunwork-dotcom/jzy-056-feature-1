package triangulate

import "delaunaysvc/internal/geom"

// boundaryRing returns the boundary polygon of a finished triangle soup
// as vertex indices in counter-clockwise order.
//
// Every triangle is stored counter-clockwise. For an interior undirected
// edge the two adjacent triangles traverse it in opposite directions, so
// each orientation occurs exactly once; for a boundary edge only the CCW
// orientation (triangle interior on the left) occurs at all. Hence an
// oriented edge without its reverse present is precisely a boundary
// edge, and chaining those edges gives the CCW boundary ring, including
// input points lying on a straight hull edge.
func boundaryRing(ts []geom.Triangle) ([]int, *geom.Error) {
	// Presence of every directed edge.
	has := make(map[[2]int]bool, 3*len(ts))
	for _, t := range ts {
		for _, e := range [3][2]int{{t.A, t.B}, {t.B, t.C}, {t.C, t.A}} {
			if has[e] {
				// Same directed edge used by two triangles: either a
				// duplicated/degenerate element or an inconsistently
				// oriented shared edge. Either way the soup is not the
				// oriented manifold the rest of the kernel assumes.
				return nil, &geom.Error{
					Code:    geom.ErrDegenerate,
					Message: "finished mesh contains a duplicated directed edge (non-manifold or non-CCW soup)",
				}
			}
			has[e] = true
		}
	}

	// Boundary edges are directed edges with no reverse counterpart.
	next := make(map[int]int, len(ts)/2)
	for e := range has {
		if !has[[2]int{e[1], e[0]}] {
			if other, collision := next[e[0]]; collision && other != e[1] {
				return nil, &geom.Error{
					Code:    geom.ErrDegenerate,
					Message: "boundary vertex has two outgoing boundary edges",
				}
			}
			next[e[0]] = e[1]
		}
	}
	if len(next) == 0 {
		return nil, &geom.Error{
			Code:    geom.ErrDegenerate,
			Message: "finished mesh has no boundary edges",
		}
	}

	var start int
	for u := range next {
		start = u
		break
	}
	ring := make([]int, 0, len(next))
	cur := start
	for {
		ring = append(ring, cur)
		nxt, ok := next[cur]
		if !ok {
			return nil, &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh boundary is not a closed ring",
			}
		}
		cur = nxt
		if cur == start {
			break
		}
		if len(ring) > len(next) {
			return nil, &geom.Error{
				Code:    geom.ErrDegenerate,
				Message: "mesh boundary walk did not close",
			}
		}
	}
	return ring, nil
}
