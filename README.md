# delaunayd — Delaunay triangulation & Voronoi dual service

A correctness-focused 2D geometry microservice. Feed it a point set over
HTTP; it returns a **Delaunay triangulation** satisfying the empty
circumcircle property and the **Voronoi diagram** that is its exact dual.
There is no UI, no rendering — only the geometry kernel and a JSON API.

- Language: **Go 1.22** (toolchain pinned in `go.mod` and `Dockerfile`)
- HTTP layer: **Gin**
- Algorithm: incremental **Bowyer–Watson** insertion
- Package layout: parsing/validation, predicates, insertion, Voronoi
  export and the transport layer are separate packages and files.

## API

Base path: `/api/v1`. All bodies are JSON. The service listens on
`:8080` by default (override with `LISTEN_ADDR`).

The service exposes two kinds of client:

- **stateless** — one-shot compute endpoints (`/triangulate`, `/voronoi`,
  `/sample`); a full mesh is built per request and nothing is retained;
- **stateful incremental sessions** (`/api/v1/sessions…`) — create a
  long-lived triangulation, then edit it by inserting or moving points.
  Each edit is applied **locally** (only the affected "dig a hole,
  re-fan" cavity is retriangulated) and returns just the triangles that
  changed, so incremental renderers can patch the previous mesh instead
  of receiving the whole net. Edits on one session handle are serialised
  on the server; distinct handles are independent.

### Stateful incremental sessions

#### `POST /api/v1/sessions`

Create a session from an initial point set (same point validation as
`/triangulate`: at least 3, non-duplicate, non-collinear). Returns the
handle plus the initial mesh.

```json
{ "points": [ {"x":0,"y":0}, {"x":4,"y":0}, {"x":4,"y":4}, {"x":0,"y":4} ] }
```

```json
{
  "session_id": 1,
  "point_count": 4,
  "triangles": [ {"a":2,"b":0,"c":1}, {"a":3,"b":0,"c":2} ],
  "hull": [0, 1, 2, 3]
}
```

Point indices are the stable identity of a point for the whole life of
the session; an insert appends one new index, a move keeps the index.

#### `POST /api/v1/sessions/:id/insert`

Insert one point. Returns its new `point_id` and the local change set
(the triangles removed and added by the cavity refan).

Request: `{"x":2.0,"y":2.0}`

```json
{
  "session_id": 1,
  "point_id": 4,
  "changes": {
    "removed": [ {"a":2,"b":0,"c":1}, {"a":3,"b":0,"c":2} ],
    "added":   [ {"a":4,"b":0,"c":1}, {"a":4,"b":1,"c":2},
                 {"a":4,"b":2,"c":3}, {"a":4,"b":3,"c":0} ]
  }
}
```

Applying `removed` then `added` to the old triangle set yields exactly
the current mesh.

#### `POST /api/v1/sessions/:id/move`

Move an existing point atomically: the point's star is collapsed, the
hole is refilled and driven to Delaunay, then the point is re-inserted
at the new location with the same cavity machinery. Request:

```json
{ "point_id": 4, "x": 3.0, "y": 1.0 }
```

Response carries the same `changes` envelope (an empty change set means
the topology is unchanged, e.g. a move inside the same topological cell).

#### `GET /api/v1/sessions/:id`

Current point/triangle counts, full triangle list, hull ring and an
`is_delaunay` flag.

#### `DELETE /api/v1/sessions/:id`

Destroys the session and releases its memory.

After every accepted edit the session mesh is itself a legal Delaunay
triangulation: every triangle satisfies the empty-circumcircle property
in the exact predicate frame and the triangles cover the current convex
hull without overlap or gap. Co-circular configurations may choose any
valid diagonal (two co-circular meshes need not carry identical triangle
sets), so tests/clients must verify a mesh **as it stands**, never by an
exact triangle-set comparison against a from-scratch rebuild.

Session errors are reported with the same structured envelope, e.g.
`DUPLICATE_POINT` (400), `POINT_NOT_FOUND` (400) and
`SESSION_NOT_FOUND` (404 for an unknown or already-destroyed handle).

### `GET /healthz`

```json
{ "status": "ok" }
```

### `POST /api/v1/triangulate`

Request:

```json
{
  "points": [
    { "x": 0.0, "y": 0.0 },
    { "x": 4.0, "y": 0.0 },
    { "x": 4.0, "y": 3.0 }
  ]
}
```

Response (`200`):

```json
{
  "point_count": 3,
  "triangle_count": 1,
  "triangles": [ { "a": 0, "b": 1, "c": 2 } ],
  "hull": [0, 1, 2],
  "hull_edge_count": 3,
  "hull_area": 6,
  "triangle_area_sum": 6,
  "is_delaunay": true,
  "violations": []
}
```

- `triangles` are index triples into the request's `points` array,
  always oriented **counter-clockwise**.
- `hull` is the convex-hull ring in CCW order; input points lying on a
  hull edge are real vertices and are included.
- `is_delaunay` is the result of an independent post-build empty-circle
  check; `violations` lists any offending `(triangle, point)` pair.

### `POST /api/v1/voronoi`

Request accepts an optional `"include_rays": true`. Response:

- `vertices[]`: Voronoi vertices. `index` is the generating triangle
  index and `point` is that triangle's **circumcenter**.
- `edges[]`: finite Voronoi edges, each joining the circumcenters of the
  two triangles sharing a Delaunay edge (`{u, v}` are triangle indices).
- `ray_count` / `rays[]`: unbounded Voronoi edges dual to hull edges;
  each ray carries its origin triangle, a **unit outward direction**,
  and the hull edge it is dual to. Rays are omitted from the body unless
  `include_rays` is set (the count is always reported).

### `GET /api/v1/sample`

The built-in, reproducible reference case: a `4×4` regular grid (16
points) whose interior points carry fixed deterministic perturbations.
It also reports the Euler-pinned counts:

```
T = 2N - H - 2 = 2·16 - 12 - 2 = 18
```

Feed the returned `points` straight back into `/triangulate` to
re-check the pinned count at any time.

### Errors

All failures return `400` with one envelope:

```json
{ "code": "TOO_FEW_POINTS", "message": "at least 3 points are required, got 2" }
```

| Code                  | Meaning                                              |
|-----------------------|------------------------------------------------------|
| `TOO_FEW_POINTS`      | fewer than 3 points                                  |
| `DUPLICATE_POINT`     | two exactly identical points (initial set, insert, or move) |
| `INVALID_COORDINATE`  | a coordinate is `NaN` or infinite at the kernel level|
| `ALL_POINTS_COLLINEAR`| every point lies on one line; hull area is zero      |
| `POINT_NOT_FOUND`     | a session move names a point id that does not exist  |
| `SESSION_NOT_FOUND`   | the session handle is unknown or already destroyed (`404`) |
| `INVALID_JSON`        | body is not parseable JSON (includes non-finite JSON numbers such as `NaN`/`Infinity`) |
| `DEGENERATE_GEOMETRY` | numerical degeneracy detected during construction    |

## Kernel details

1. **Translation to local coordinates.** Every point is translated by
   `-points[0]` before any predicate runs. Determinant signs are
   invariant under translation, and doing the arithmetic near the
   origin keeps the fast error bounds tight for point sets far from
   `(0,0)`. Circumcenters are translated back, so Voronoi vertices
   co-vary exactly with the input.
2. **Super-triangle.** A strictly CCW triangle with a wide margin
   contains every point. After all insertions, *every* triangle touching
   a virtual vertex is removed — ghost triangles can never appear in a
   response.
3. **Bowyer–Watson.** For each new point the triangles whose
   circumcircle strictly contains it are found with a local adjacency
   walk seeded by the triangle that geometrically contains the point
   (a uniform-grid spatial index supplies the candidates); the cavity
   boundary (edges occurring once) is extracted and the point is fanned
   over it. Every fan triangle is oriented CCW with an exact orientation
   test. A point landing on an existing edge splits that edge so both
   sides close.
3a. **Incremental sessions reuse this exact loop.** The stateless build
   and the stateful session run the *same* cavity/fan routine; the
   session simply keeps the triangle store, edge adjacency, per-vertex
   stars, spatial index and super-triangle alive between edits, so only
   the cavity triangles are touched per operation. A **move** collapses
   the moved point's star, refills the hole by ear clipping and drives it
   to Delaunay with edge flips, then re-runs the identical insertion loop
   at the new coordinate. A rare order-dependent near-collinear sliver is
   closed by a topological post-fan repair (a no-op on an ordinary cavity)
   and, only if that still fails, an order/salt-robust whole rebuild that
   is verified before it is accepted — so every returned mesh satisfies
   the empty-circumcircle and hull-coverage guarantees. Each edit returns
   only the removed/added triangle sets.
4. **Adaptive exact predicates.** Orientation and in-circle signs are
   decided in two stages: a binary64 determinant with a proven
   forward-error bound (`(3+16ε)ε` for orientation, a conservative
   `256·ε·M⁴` bound for the degree-four in-circle determinant), falling
   back to **512-bit `math/big` exact evaluation** only when the sign is
   too close to trust. There is no hand-tuned absolute epsilon: nearly
   co-circular points or nearly coincident vertices never flip from
   float jitter, while the common case costs a few FP multiplies.
5. **Deterministic general-position perturbation.** Exactly co-circular
   configurations (e.g. regular polygons) make the strict cavity wrap an
   existing vertex. In the translated internal frame each point receives
   a deterministic, index-derived, scale-free `~1e-12·span` nudge,
   putting the set in general position. Because it is a pure function of
   the index, the same request always yields the same triangulation (no
   flip oscillation), and because it is applied *after* translation is
   removed, rigidly translating the request leaves the topology
   unchanged. All geometric checks, areas and Voronoi circumcenters are
   evaluated in this exact-Delaunay internal frame; reported point
   coordinates remain the caller's originals.
6. **Voronoi dual.** Triangle → circumcenter vertex; interior Delaunay
   edge → finite edge between the two circumcenters; hull edge →
   outward-pointing ray.

## Build & run

Local:

```sh
go test ./...
go build ./...
LISTEN_ADDR=:8080 go run ./cmd/delaunayd
```

Docker:

```sh
docker build -t delaunayd .
docker run --rm -p 8080:8080 delaunayd
```

Quick call:

```sh
curl -s localhost:8080/api/v1/sample | head
curl -s -XPOST localhost:8080/api/v1/triangulate \
  -H 'content-type: application/json' \
  -d '{"points":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1},{"x":0,"y":1}]}'
```

## Tests

`go test ./...` pins, with automated assertions:

- **empty circumcircle** on fixed and randomized point sets (incl.
  violation reporting);
- **hull area conservation**: `Σ triangle area == hull area`;
- **rigid translation invariance**: identical triangle index triples,
  and Voronoi vertices shifted by exactly the same vector (incl.
  end-to-end HTTP tests);
- **interior insertion adds exactly two triangles**;
- **four co-circular points / regular polygons / perturbation sweeps**:
  no crossing edges, no oscillation, always a valid mesh;
- structural invariants on every case: all-CCW, no ghost indices,
  manifold edge multiplicities, no crossing edges, Euler `T-E+N = 1` and
  `T = 2N-H-2`;
- Voronoi vertices equal exact circumcenters, dual edges are
  perpendicular bisectors, one ray per hull edge;
- rejection of too-few, duplicate, non-finite and collinear inputs;
- the sample grid's triangle count pinned to **18**;
- **stateful sessions** (long randomised insert/move/mixed series): at
  every step the mesh passes the global empty-circumcircle check and
  covers the convex hull, the returned change set applies exactly to the
  previous mesh, and rejected edits (duplicate, unknown point, unknown /
  destroyed handle) leave the mesh intact; includes concurrent-session
  and end-to-end HTTP tests.

## Layout

```
cmd/delaunayd/          service entrypoint (HTTP bootstrap only)
internal/geom/          points/triangles, predicates+tolerance,
                        circumcircle, validation, convex hull
internal/triangulate/   stateless Build + the stateful incremental Mesh
                        (cavity/fan insert, star-collapse move, local
                        sliver repair, robust rebuild, change-set diff)
internal/session/       long-lived session handles, serialised edits,
                        input rejection and explicit destruction
internal/voronoi/       Voronoi dual export
internal/sample/        built-in perturbed-grid reference case
internal/api/           Gin routes, request binding, JSON DTOs
                        (stateless endpoints and session endpoints)
```
