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
| `DUPLICATE_POINT`     | two exactly identical, undeclared input points       |
| `INVALID_COORD`       | a coordinate is `NaN` or infinite at the kernel level|
| `ALL_POINTS_COLLINEAR`| every point lies on one line; hull area is zero      |
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
   walk (a uniform-grid spatial index supplies the seed triangle); the
   cavity boundary (edges occurring once) is extracted and the point is
   fanned over it. Every fan triangle is oriented CCW with an exact
   orientation test. A star-shape expansion across any non-visible
   boundary edge keeps the cavity well-formed near hull insertions.
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
- the sample grid's triangle count pinned to **18**.

## Layout

```
cmd/delaunayd/          service entrypoint (HTTP bootstrap only)
internal/geom/          points/triangles, predicates+tolerance,
                        circumcircle, validation, convex hull
internal/triangulate/   Bowyer–Watson insertion, cavity, hull ring,
                        empty-circle verification
internal/voronoi/       Voronoi dual export
internal/sample/        built-in perturbed-grid reference case
internal/api/           Gin routes, request binding, JSON DTOs
```
