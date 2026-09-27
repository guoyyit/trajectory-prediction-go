// Package trajectory implements a local moving-object trajectory
// prediction service: ordered observation history, motion models,
// uncertainty propagation, conflict detection and persistence.
package trajectory

import "time"

// ModelKind identifies a motion model.
type ModelKind string

const (
	ModelCV   ModelKind = "cv"   // constant velocity
	ModelCA   ModelKind = "ca"   // constant acceleration
	ModelCTRV ModelKind = "ctrv" // constant turn rate and velocity
)

// Observation is a single sensor observation of a dynamic object.
type Observation struct {
	ObjectID string    `json:"object_id"`
	T        time.Time `json:"t"`
	X        float64   `json:"x"`
	Y        float64   `json:"y"`
	VX       float64   `json:"vx"`
	VY       float64   `json:"vy"`
	Heading  float64   `json:"heading"` // radians, CCW from +x axis
}

// PredictedState is the estimated state at one future time.
type PredictedState struct {
	T       time.Time `json:"t"`
	X       float64   `json:"x"`
	Y       float64   `json:"y"`
	VX      float64   `json:"vx"`
	VY      float64   `json:"vy"`
	Heading float64   `json:"heading"`
	// Covariance of (x, y): symmetric 2x2, row-major.
	CovXX float64 `json:"cov_xx"`
	CovXY float64 `json:"cov_xy"`
	CovYY float64 `json:"cov_yy"`
}

// Trajectory is a sequence of predicted states for one object.
type Trajectory struct {
	ObjectID string           `json:"object_id"`
	Model    ModelKind        `json:"model"`
	States   []PredictedState `json:"states"`
}

// Gap describes a missing time interval in an object's history.
type Gap struct {
	Start time.Time `json:"start"`
	End   time.Time `json:"end"`
}

// Conflict describes two predicted trajectories coming within a
// threshold distance of each other.
type Conflict struct {
	ObjectA  string    `json:"object_a"`
	ObjectB  string    `json:"object_b"`
	T        time.Time `json:"t"`
	Distance float64   `json:"distance"`
}
