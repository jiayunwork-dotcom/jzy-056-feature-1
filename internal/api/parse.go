package api

import (
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"
)

// parseID reads the :id path parameter and reports a 400 on failure.
func parseID(c *gin.Context) (uint64, bool) {
	raw := c.Param("id")
	id, err := strconv.ParseUint(raw, 10, 64)
	if err != nil {
		fail(c, http.StatusBadRequest, "INVALID_JSON", "session id must be a non-negative integer")
		return 0, false
	}
	return id, true
}
