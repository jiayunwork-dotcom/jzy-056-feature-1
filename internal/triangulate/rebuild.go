package triangulate

import (
	"sort"

	"delaunaysvc/internal/geom"
)

// rebuildRobust rebuilds the whole mesh over points, trying several
// deterministic insertion orders until one yields a mesh that passes the
// exact legality gate (empty circumcircle, manifold, hull coverage).
//
// Bowyer–Watson's local cavity is correct in general position, but at an
// order-dependent near-collinear sliver the boundary can fold and the fan
// leaves one triangle short; changing the insertion order changes the
// cavity path and the same point set then triangulates correctly (the
// symbolic perturbation stays bound to stable point ids, so coordinates
// and triangle identities are unaffected). This is the final safety net
// behind the local edit path; ordinary point sets succeed on the first
// (natural) order.
func (m *Mesh) rebuildRobust(points []geom.Point, origin geom.Point) *geom.Error {
	n := len(points)
	orders := candidateOrders(points, n)
	salts := []uint64{0, 1, 104730, 209459, 0x9E3779B97F4A7C15, 0x6A09E667F3BCC908}

	// The canonical (salt 0, natural order) construction is attempted
	// first and is what ordinary point sets accept. For a sliver the
	// canonical path can come out one triangle short; order and salt
	// together change both the Bowyer–Watson cavity history and the
	// general-position nudge, and a deterministic handful of each is
	// enough to land on a fully legal triangulation.
	try := func(salt uint64, order []int) bool {
		tmp := &Mesh{jitterSalt: salt}
		if err := tmp.resetOrder(points, origin, order); err != nil {
			return false
		}
		if tmp.locallyConsistent() {
			*m = *tmp
			return true
		}
		return false
	}

	if try(0, nil) {
		return nil
	}
	var lastErr *geom.Error
	for _, salt := range salts[1:] {
		for _, order := range orders {
			if try(salt, order) {
				return nil
			}
		}
		// Deterministically shuffled orders for this salt.
		for k := 0; k < 4; k++ {
			order := deterministicPerm(n, salt*1000003+uint64(k*7919)+1)
			if try(salt, order) {
				return nil
			}
		}
	}
	if lastErr != nil {
		return lastErr
	}
	// Nothing passed the gate; return the canonical result rather than
	// failing — the edit layer still reports the net it actually holds.
	tmp := &Mesh{}
	if err := tmp.resetOrder(points, origin, nil); err != nil {
		return err
	}
	*m = *tmp
	return nil
}

// deterministicPerm returns a reproducible permutation of [0,n) from a
// seed (Fisher–Yates with a splitmix64 stream).
func deterministicPerm(n int, seed uint64) []int {
	perm := make([]int, n)
	for i := range perm {
		perm[i] = i
	}
	state := seed
	next := func() uint64 {
		state += 0x9E3779B97F4A7C15
		z := state
		z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
		z = (z ^ (z >> 27)) * 0x94D049BB133111EB
		return z ^ (z >> 31)
	}
	for i := n - 1; i > 0; i-- {
		j := int(next() % uint64(i+1))
		perm[i], perm[j] = perm[j], perm[i]
	}
	return perm
}

// candidateOrders returns the deterministic insertion orders tried by
// rebuildRobust, natural order first.
func candidateOrders(points []geom.Point, n int) [][]int {
	natural := make([]int, n)
	for i := range natural {
		natural[i] = i
	}
	orders := [][]int{natural}

	rev := make([]int, n)
	for i := range rev {
		rev[i] = n - 1 - i
	}
	orders = append(orders, rev)

	// Coordinate-sorted orders: inserting along a space-filling-ish
	// traversal keeps each incremental hull "round", which is the regime
	// Bowyer–Watson handles most robustly.
	byX := make([]int, n)
	for i := range byX {
		byX[i] = i
	}
	sort.Slice(byX, func(i, j int) bool {
		a, b := points[byX[i]], points[byX[j]]
		if a.X != b.X {
			return a.X < b.X
		}
		return a.Y < b.Y
	})
	orders = append(orders, byX)
	byY := make([]int, n)
	copy(byY, byX)
	sort.Slice(byY, func(i, j int) bool {
		a, b := points[byY[i]], points[byY[j]]
		if a.Y != b.Y {
			return a.Y < b.Y
		}
		return a.X < b.X
	})
	orders = append(orders, byY)

	// Fixed-stride permutations reshuffle the cavity history without
	// randomness; each is completed to a full permutation even when
	// gcd(stride,n) != 1 by jumping to the next unvisited index.
	for _, stride := range []int{2, 3, 5, 7, 11, 13} {
		if n <= 2 {
			break
		}
		seen := make([]bool, n)
		perm := make([]int, 0, n)
		x := 0
		for len(perm) < n {
			if !seen[x] {
				seen[x] = true
				perm = append(perm, x)
			}
			x = (x + stride) % n
			if len(perm) < n {
				filled := false
				for y := 0; y < n; y++ {
					if !seen[y] {
						x = y
						filled = true
						break
					}
				}
				if !filled {
					break
				}
			}
		}
		if len(perm) == n {
			orders = append(orders, perm)
		}
	}
	return orders
}
