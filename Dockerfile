# syntax=docker/dockerfile:1

# ---- build stage: pinned to the Go 1.22 toolchain -----------------------
FROM golang:1.22-bookworm AS build

WORKDIR /src

# Resolve dependencies first so the layer is cached across source-only
# changes.
COPY go.mod go.sum ./
RUN --mount=type=cache,target=/go/pkg/mod \
    go mod download

COPY . .

# Build a fully static binary (CGO disabled) from the API command only.
RUN --mount=type=cache,target=/go/pkg/mod \
    --mount=type=cache,target=/root/.cache/go-build \
    CGO_ENABLED=0 GOOS=linux go build \
        -trimpath \
        -ldflags="-s -w" \
        -o /out/delaunayd ./cmd/delaunayd

# ---- runtime stage: minimal distroless image ----------------------------
FROM gcr.io/distroless/static-debian12:nonroot

WORKDIR /
COPY --from=build /out/delaunayd /usr/local/bin/delaunayd

EXPOSE 8080
USER nonroot:nonroot

ENTRYPOINT ["/usr/local/bin/delaunayd"]
