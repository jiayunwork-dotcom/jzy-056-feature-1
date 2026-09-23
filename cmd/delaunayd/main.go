// Command delaunayd serves the Delaunay triangulation and Voronoi dual
// HTTP API. It contains no geometry code of its own; the interface layer
// only marshals requests and responses.
package main

import (
	"log"
	"os"

	"github.com/gin-gonic/gin"

	"delaunaysvc/internal/api"
)

func main() {
	addr := os.Getenv("LISTEN_ADDR")
	if addr == "" {
		addr = ":8080"
	}
	if v := os.Getenv("GIN_MODE"); v != "" {
		gin.SetMode(v)
	}

	r := gin.New()
	r.Use(gin.Logger(), gin.Recovery())
	api.Register(r)

	log.Printf("delaunayd listening on %s", addr)
	if err := r.Run(addr); err != nil {
		log.Fatalf("server failed: %v", err)
	}
}
