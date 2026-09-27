package trajectory

import (
	"sort"
	"time"
)

// history keeps one object's observations ordered by timestamp,
// deduplicated, and tracks gaps. It is NOT safe for concurrent use;
// callers (Service) must hold the appropriate lock.
type history struct {
	obs []Observation // ascending by T
}

// equal reports whether two observations are identical duplicates.
func equalObservation(a, b Observation) bool {
	return a.ObjectID == b.ObjectID && a.T.Equal(b.T) &&
		a.X == b.X && a.Y == b.Y && a.VX == b.VX && a.VY == b.VY &&
		a.Heading == b.Heading
}

// insert adds an observation, keeping timestamp order and dropping
// exact duplicates. Out-of-order observations are placed correctly.
// It reports whether the observation was actually added.
func (h *history) insert(o Observation) bool {
	// Binary search for insertion point (first index with obs[i].T >= o.T).
	i := sort.Search(len(h.obs), func(i int) bool {
		return !h.obs[i].T.Before(o.T)
	})
	// Dedup: same timestamp and identical payload => duplicate.
	if i < len(h.obs) && h.obs[i].T.Equal(o.T) && equalObservation(h.obs[i], o) {
		return false
	}
	// Same timestamp but different payload: keep both (distinct samples),
	// inserting after existing same-timestamp entries for stability.
	for i < len(h.obs) && h.obs[i].T.Equal(o.T) {
		i++
	}
	h.obs = append(h.obs, Observation{})
	copy(h.obs[i+1:], h.obs[i:])
	h.obs[i] = o
	return true
}

// latest returns the most recent observation, if any.
func (h *history) latest() (Observation, bool) {
	if len(h.obs) == 0 {
		return Observation{}, false
	}
	return h.obs[len(h.obs)-1], true
}

// snapshot returns a copy of the ordered observations.
func (h *history) snapshot() []Observation {
	out := make([]Observation, len(h.obs))
	copy(out, h.obs)
	return out
}

// gaps returns missing intervals given the expected sampling period.
// A gap exists wherever the time between consecutive observations
// exceeds 1.5×period. Missing data is never fabricated.
func (h *history) gaps(period time.Duration) []Gap {
	if period <= 0 || len(h.obs) < 2 {
		return nil
	}
	threshold := period + period/2
	var out []Gap
	for i := 1; i < len(h.obs); i++ {
		d := h.obs[i].T.Sub(h.obs[i-1].T)
		if d > threshold {
			out = append(out, Gap{Start: h.obs[i-1].T, End: h.obs[i].T})
		}
	}
	return out
}
