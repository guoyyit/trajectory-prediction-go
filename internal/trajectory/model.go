package trajectory

import "math"

// NoiseConfig configures initial covariance and process noise.
// Larger values yield faster-growing prediction uncertainty.
type NoiseConfig struct {
	InitialPosVar float64 `json:"initial_pos_var"` // initial variance of x, y
	InitialVelVar float64 `json:"initial_vel_var"` // initial variance of velocity components
	InitialAccVar float64 `json:"initial_acc_var"` // initial variance of acceleration (CA)
	ProcessPosVar float64 `json:"process_pos_var"` // CTRV positional process noise density
	ProcessVelVar float64 `json:"process_vel_var"` // CV/CA accel spectral density; CTRV speed noise
	ProcessAccVar float64 `json:"process_acc_var"` // CA jerk spectral density
	ProcessYawVar float64 `json:"process_yaw_var"` // CTRV yaw / yaw-rate noise density
}

// motionModel propagates a state vector and its covariance.
type motionModel interface {
	// dim is the state dimension.
	dim() int
	// initial builds a state vector from ordered observations.
	initial(obs []Observation) []float64
	// initialCov returns the covariance at the initial state.
	initialCov() matrix
	// step advances state x by dt seconds, returning the new state,
	// the Jacobian F and the process-noise covariance Q for that step.
	step(x []float64, dt float64) (nx []float64, F, Q matrix)
	// extract converts a state vector and covariance into a PredictedState.
	extract(x []float64, cov matrix) PredictedState
}

// modelFor returns the motion model for a kind.
func modelFor(kind ModelKind, cfg NoiseConfig) motionModel {
	switch kind {
	case ModelCA:
		return caModel{cfg: cfg}
	case ModelCTRV:
		return ctrvModel{cfg: cfg}
	default:
		return cvModel{cfg: cfg}
	}
}

// speed computes planar speed.
func speed(o Observation) float64 { return math.Hypot(o.VX, o.VY) }

// ---------------- constant velocity ----------------

// cvModel state: [x, y, vx, vy].
type cvModel struct{ cfg NoiseConfig }

func (cvModel) dim() int { return 4 }

func (m cvModel) initial(obs []Observation) []float64 {
	o := obs[len(obs)-1]
	return []float64{o.X, o.Y, o.VX, o.VY}
}

func (m cvModel) initialCov() matrix {
	return diag([]float64{m.cfg.InitialPosVar, m.cfg.InitialPosVar,
		m.cfg.InitialVelVar, m.cfg.InitialVelVar})
}

func (m cvModel) step(x []float64, dt float64) ([]float64, matrix, matrix) {
	F := identity(4)
	F.set(0, 2, dt)
	F.set(1, 3, dt)
	nx := []float64{
		x[0] + dt*x[2],
		x[1] + dt*x[3],
		x[2],
		x[3],
	}
	// Discrete white-noise acceleration model.
	q := m.cfg.ProcessVelVar
	dt2, dt3, dt4 := dt*dt, dt*dt*dt, dt*dt*dt*dt
	Q := matFromFlat(4, []float64{
		dt4 / 4 * q, 0, dt3 / 2 * q, 0,
		0, dt4 / 4 * q, 0, dt3 / 2 * q,
		dt3 / 2 * q, 0, dt2 * q, 0,
		0, dt3 / 2 * q, 0, dt2 * q,
	})
	return nx, F, Q
}

func (cvModel) extract(x []float64, cov matrix) PredictedState {
	return PredictedState{
		X: x[0], Y: x[1], VX: x[2], VY: x[3],
		Heading: math.Atan2(x[3], x[2]),
		CovXX:   cov.at(0, 0), CovXY: cov.at(0, 1), CovYY: cov.at(1, 1),
	}
}

// ---------------- constant acceleration ----------------

// caModel state: [x, y, vx, vy, ax, ay].
type caModel struct{ cfg NoiseConfig }

func (caModel) dim() int { return 6 }

func (m caModel) initial(obs []Observation) []float64 {
	o := obs[len(obs)-1]
	ax, ay := 0.0, 0.0
	// Estimate acceleration from the last two distinct-time observations.
	for i := len(obs) - 1; i > 0; i-- {
		prev, cur := obs[i-1], obs[i]
		dt := cur.T.Sub(prev.T).Seconds()
		if dt > 0 {
			ax = (cur.VX - prev.VX) / dt
			ay = (cur.VY - prev.VY) / dt
			break
		}
	}
	return []float64{o.X, o.Y, o.VX, o.VY, ax, ay}
}

func (m caModel) initialCov() matrix {
	return diag([]float64{m.cfg.InitialPosVar, m.cfg.InitialPosVar,
		m.cfg.InitialVelVar, m.cfg.InitialVelVar,
		m.cfg.InitialAccVar, m.cfg.InitialAccVar})
}

func (m caModel) step(x []float64, dt float64) ([]float64, matrix, matrix) {
	half := dt * dt / 2
	F := identity(6)
	F.set(0, 2, dt)
	F.set(1, 3, dt)
	F.set(0, 4, half)
	F.set(1, 5, half)
	F.set(2, 4, dt)
	F.set(3, 5, dt)
	nx := []float64{
		x[0] + dt*x[2] + half*x[4],
		x[1] + dt*x[3] + half*x[5],
		x[2] + dt*x[4],
		x[3] + dt*x[5],
		x[4],
		x[5],
	}
	// Discrete white-noise jerk model.
	q := m.cfg.ProcessAccVar
	dt2 := dt * dt
	dt3 := dt2 * dt
	dt4 := dt3 * dt
	dt5 := dt4 * dt
	Q := matFromFlat(6, []float64{
		dt5 / 20 * q, 0, dt4 / 8 * q, 0, dt3 / 6 * q, 0,
		0, dt5 / 20 * q, 0, dt4 / 8 * q, 0, dt3 / 6 * q,
		dt4 / 8 * q, 0, dt3 / 3 * q, 0, dt2 / 2 * q, 0,
		0, dt4 / 8 * q, 0, dt3 / 3 * q, 0, dt2 / 2 * q,
		dt3 / 6 * q, 0, dt2 / 2 * q, 0, dt * q, 0,
		0, dt3 / 6 * q, 0, dt2 / 2 * q, 0, dt * q,
	})
	return nx, F, Q
}

func (caModel) extract(x []float64, cov matrix) PredictedState {
	return PredictedState{
		X: x[0], Y: x[1], VX: x[2], VY: x[3],
		Heading: math.Atan2(x[3], x[2]),
		CovXX:   cov.at(0, 0), CovXY: cov.at(0, 1), CovYY: cov.at(1, 1),
	}
}

// ---------------- constant turn rate and velocity ----------------

// ctrvModel state: [x, y, v, heading, yawRate].
type ctrvModel struct{ cfg NoiseConfig }

func (ctrvModel) dim() int { return 5 }

func (m ctrvModel) initial(obs []Observation) []float64 {
	o := obs[len(obs)-1]
	yawRate := 0.0
	// Estimate yaw rate from the last two distinct-time observations.
	for i := len(obs) - 1; i > 0; i-- {
		prev, cur := obs[i-1], obs[i]
		dt := cur.T.Sub(prev.T).Seconds()
		if dt > 0 {
			yawRate = wrapAngle(cur.Heading-prev.Heading) / dt
			break
		}
	}
	return []float64{o.X, o.Y, speed(o), o.Heading, yawRate}
}

func (m ctrvModel) initialCov() matrix {
	return diag([]float64{m.cfg.InitialPosVar, m.cfg.InitialPosVar,
		m.cfg.InitialVelVar, m.cfg.ProcessYawVar, m.cfg.ProcessYawVar})
}

const yawEps = 1e-6

func (m ctrvModel) step(x []float64, dt float64) ([]float64, matrix, matrix) {
	px, py, v, yaw, w := x[0], x[1], x[2], x[3], x[4]
	nx := make([]float64, 5)
	F := identity(5)
	if math.Abs(w) < yawEps {
		nx[0] = px + v*dt*math.Cos(yaw)
		nx[1] = py + v*dt*math.Sin(yaw)
		F.set(0, 2, dt*math.Cos(yaw))
		F.set(0, 3, -v*dt*math.Sin(yaw))
		F.set(1, 2, dt*math.Sin(yaw))
		F.set(1, 3, v*dt*math.Cos(yaw))
	} else {
		yaw2 := yaw + w*dt
		nx[0] = px + v/w*(math.Sin(yaw2)-math.Sin(yaw))
		nx[1] = py + v/w*(math.Cos(yaw)-math.Cos(yaw2))
		F.set(0, 2, (math.Sin(yaw2)-math.Sin(yaw))/w)
		F.set(0, 3, v/w*(math.Cos(yaw2)-math.Cos(yaw)))
		F.set(0, 4, v*dt*math.Cos(yaw2)/w-v*(math.Sin(yaw2)-math.Sin(yaw))/(w*w))
		F.set(1, 2, (math.Cos(yaw)-math.Cos(yaw2))/w)
		F.set(1, 3, v/w*(math.Sin(yaw2)-math.Sin(yaw)))
		F.set(1, 4, v*dt*math.Sin(yaw2)/w-v*(math.Cos(yaw)-math.Cos(yaw2))/(w*w))
	}
	nx[2] = v
	nx[3] = wrapAngle(yaw + w*dt)
	nx[4] = w
	F.set(3, 4, dt)
	// Diagonal process noise, scaled by step duration.
	Q := diag([]float64{
		m.cfg.ProcessPosVar * dt * dt,
		m.cfg.ProcessPosVar * dt * dt,
		m.cfg.ProcessVelVar * dt,
		m.cfg.ProcessYawVar * dt,
		m.cfg.ProcessYawVar * dt,
	})
	return nx, F, Q
}

func (ctrvModel) extract(x []float64, cov matrix) PredictedState {
	v, yaw := x[2], x[3]
	return PredictedState{
		X: x[0], Y: x[1],
		VX:      v * math.Cos(yaw),
		VY:      v * math.Sin(yaw),
		Heading: yaw,
		CovXX:   cov.at(0, 0), CovXY: cov.at(0, 1), CovYY: cov.at(1, 1),
	}
}

// wrapAngle normalizes an angle to (-π, π].
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
