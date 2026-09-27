package trajectory

import "time"

// PredictRequest describes a prediction query.
type PredictRequest struct {
	ObjectID string
	Model    ModelKind // empty means use configured default/per-object
	Horizon  time.Duration
	Step     time.Duration
}

// predict computes a trajectory from an ordered observation snapshot.
// Predicted states are emitted at t0+step, t0+2*step, …, t0+horizon,
// where t0 is the latest observation time. The propagated position
// covariance is enforced to be monotonically non-decreasing in trace.
// Returns an empty Trajectory if obs is empty or parameters invalid.
func predict(obs []Observation, kind ModelKind, cfg NoiseConfig, horizon, step time.Duration) Trajectory {
	if len(obs) == 0 || horizon <= 0 || step <= 0 {
		return Trajectory{}
	}
	m := modelFor(kind, cfg)
	x := m.initial(obs)
	P := m.initialCov()
	t0 := obs[len(obs)-1].T

	n := int(horizon / step)
	dt := step.Seconds()
	tr := Trajectory{
		ObjectID: obs[len(obs)-1].ObjectID,
		Model:    kind,
		States:   make([]PredictedState, 0, n),
	}
	prevTrace := 0.0
	for i := 1; i <= n; i++ {
		var F, Q matrix
		x, F, Q = m.step(x, dt)
		P = add(mul(mul(F, P), transpose(F)), Q)
		st := m.extract(x, P)
		st.T = t0.Add(time.Duration(i) * step)
		// Enforce monotone non-decreasing position uncertainty.
		trace := st.CovXX + st.CovYY
		if trace < prevTrace && trace > 0 {
			scale := prevTrace / trace
			st.CovXX *= scale
			st.CovXY *= scale
			st.CovYY *= scale
		} else if trace <= 0 && prevTrace > 0 {
			st.CovXX = prevTrace / 2
			st.CovYY = prevTrace / 2
			st.CovXY = 0
		}
		prevTrace = st.CovXX + st.CovYY
		tr.States = append(tr.States, st)
	}
	return tr
}
