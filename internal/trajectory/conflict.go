package trajectory

import (
	"math"
	"sort"
)

// detectConflicts finds future conflicts between predicted trajectories.
// Two objects conflict when their predicted positions at the same future
// time step come within threshold distance. For each conflicting pair the
// earliest conflicting moment is reported. States are aligned by index;
// trajectories are expected to share the prediction time grid (the
// Service guarantees this). The result is sorted deterministically by
// time, then object IDs.
func detectConflicts(trajs []Trajectory, threshold float64) []Conflict {
	var out []Conflict
	for i := 0; i < len(trajs); i++ {
		for j := i + 1; j < len(trajs); j++ {
			if c, ok := earliestConflict(trajs[i], trajs[j], threshold); ok {
				out = append(out, c)
			}
		}
	}
	sort.Slice(out, func(a, b int) bool {
		if !out[a].T.Equal(out[b].T) {
			return out[a].T.Before(out[b].T)
		}
		if out[a].ObjectA != out[b].ObjectA {
			return out[a].ObjectA < out[b].ObjectA
		}
		return out[a].ObjectB < out[b].ObjectB
	})
	return out
}

// earliestConflict scans aligned states of two trajectories and returns
// the first moment their distance drops below threshold.
func earliestConflict(a, b Trajectory, threshold float64) (Conflict, bool) {
	n := len(a.States)
	if len(b.States) < n {
		n = len(b.States)
	}
	thr2 := threshold * threshold
	for k := 0; k < n; k++ {
		sa, sb := a.States[k], b.States[k]
		if !sa.T.Equal(sb.T) {
			continue // misaligned grids: skip
		}
		dx, dy := sa.X-sb.X, sa.Y-sb.Y
		d2 := dx*dx + dy*dy
		if d2 < thr2 {
			return Conflict{
				ObjectA:  a.ObjectID,
				ObjectB:  b.ObjectID,
				T:        sa.T,
				Distance: math.Sqrt(d2),
			}, true
		}
	}
	return Conflict{}, false
}
