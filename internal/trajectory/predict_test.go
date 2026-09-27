package trajectory

import (
	"math"
	"testing"
	"time"
)

func testNoise() NoiseConfig {
	return NoiseConfig{
		InitialPosVar: 0.5, InitialVelVar: 0.3, InitialAccVar: 0.2,
		ProcessPosVar: 0.05, ProcessVelVar: 0.2,
		ProcessAccVar: 0.1, ProcessYawVar: 0.02,
	}
}

func straightObs(id string, n int) []Observation {
	out := make([]Observation, n)
	for i := 0; i < n; i++ {
		out[i] = Observation{
			ObjectID: id, T: t0.Add(time.Duration(i) * 100 * time.Millisecond),
			X: float64(i), Y: 0, VX: 10, VY: 0, Heading: 0,
		}
	}
	return out
}

func TestPredictConstantVelocity(t *testing.T) {
	obs := straightObs("a", 5) // last: t0+0.4s, x=4, vx=10
	tr := predict(obs, ModelCV, testNoise(), time.Second, 100*time.Millisecond)
	if len(tr.States) != 10 {
		t.Fatalf("want 10 states, got %d", len(tr.States))
	}
	for i, st := range tr.States {
		wantX := 4 + 10*0.1*float64(i+1)
		if math.Abs(st.X-wantX) > 1e-9 {
			t.Fatalf("state %d: x=%v, want %v", i, st.X, wantX)
		}
		wantT := t0.Add(400*time.Millisecond + time.Duration(i+1)*100*time.Millisecond)
		if !st.T.Equal(wantT) {
			t.Fatalf("state %d: t=%v, want %v", i, st.T, wantT)
		}
	}
}

func TestPredictConstantAcceleration(t *testing.T) {
	// Object accelerating at 2 m/s²: v = 2t, x = t².
	obs := make([]Observation, 6)
	for i := 0; i < 6; i++ {
		tt := float64(i)
		obs[i] = Observation{
			ObjectID: "a", T: t0.Add(time.Duration(i) * time.Second),
			X: tt * tt, Y: 0, VX: 2 * tt, VY: 0, Heading: 0,
		}
	}
	tr := predict(obs, ModelCA, testNoise(), 2*time.Second, time.Second)
	if len(tr.States) != 2 {
		t.Fatalf("want 2 states, got %d", len(tr.States))
	}
	// From t=5 (x=25, v=10, a=2): x(6)=25+10+1=36, x(7)=25+20+4=49.
	if math.Abs(tr.States[0].X-36) > 1e-9 || math.Abs(tr.States[1].X-49) > 1e-9 {
		t.Fatalf("CA prediction wrong: %v, %v", tr.States[0].X, tr.States[1].X)
	}
}

func TestPredictCTRVTurns(t *testing.T) {
	// Object turning: yaw rate 0.1 rad/s.
	obs := []Observation{
		{ObjectID: "a", T: t0, X: 0, Y: 0, VX: 10, VY: 0, Heading: 0},
		{ObjectID: "a", T: t0.Add(time.Second), X: 10, Y: 0.5, VX: 10, VY: 1, Heading: 0.1},
	}
	tr := predict(obs, ModelCTRV, testNoise(), 2*time.Second, time.Second)
	if len(tr.States) != 2 {
		t.Fatalf("want 2 states, got %d", len(tr.States))
	}
	// Heading must advance by ~0.1 rad/s.
	if math.Abs(tr.States[0].Heading-0.2) > 1e-9 {
		t.Fatalf("heading at +1s = %v, want 0.2", tr.States[0].Heading)
	}
	if math.Abs(tr.States[1].Heading-0.3) > 1e-9 {
		t.Fatalf("heading at +2s = %v, want 0.3", tr.States[1].Heading)
	}
	// A turning object must curve away from the straight-line path.
	if tr.States[1].Y <= 1.0 {
		t.Fatalf("CTRV should curve: y=%v", tr.States[1].Y)
	}
}

func TestUncertaintyMonotonicNonDecreasing(t *testing.T) {
	obs := straightObs("a", 5)
	for _, kind := range []ModelKind{ModelCV, ModelCA, ModelCTRV} {
		tr := predict(obs, kind, testNoise(), 5*time.Second, 100*time.Millisecond)
		prev := 0.0
		for i, st := range tr.States {
			trace := st.CovXX + st.CovYY
			if trace < prev {
				t.Fatalf("%s: uncertainty decreased at step %d: %v < %v", kind, i, trace, prev)
			}
			prev = trace
		}
	}
}

func TestNoiseConfigChangesUncertainty(t *testing.T) {
	obs := straightObs("a", 5)
	low := testNoise()
	high := testNoise()
	high.ProcessVelVar = 2.0
	high.ProcessYawVar = 0.5
	for _, kind := range []ModelKind{ModelCV, ModelCTRV} {
		trLow := predict(obs, kind, low, 2*time.Second, 100*time.Millisecond)
		trHigh := predict(obs, kind, high, 2*time.Second, 100*time.Millisecond)
		last := len(trLow.States) - 1
		tl := trLow.States[last].CovXX + trLow.States[last].CovYY
		th := trHigh.States[last].CovXX + trHigh.States[last].CovYY
		if th <= tl {
			t.Fatalf("%s: higher noise must give higher uncertainty: %v <= %v", kind, th, tl)
		}
	}
}

func TestModelsDiffer(t *testing.T) {
	obs := straightObs("a", 5)
	cv := predict(obs, ModelCV, testNoise(), 2*time.Second, 100*time.Millisecond)
	ca := predict(obs, ModelCA, testNoise(), 2*time.Second, 100*time.Millisecond)
	last := len(cv.States) - 1
	// Same mean motion, but uncertainty must differ between models.
	cvTrace := cv.States[last].CovXX + cv.States[last].CovYY
	caTrace := ca.States[last].CovXX + ca.States[last].CovYY
	if cvTrace == caTrace {
		t.Fatalf("models must produce distinguishable uncertainty: both %v", cvTrace)
	}
}

func TestPredictDeterministic(t *testing.T) {
	obs := straightObs("a", 5)
	a := predict(obs, ModelCTRV, testNoise(), 2*time.Second, 50*time.Millisecond)
	b := predict(obs, ModelCTRV, testNoise(), 2*time.Second, 50*time.Millisecond)
	if len(a.States) != len(b.States) {
		t.Fatal("state count mismatch")
	}
	for i := range a.States {
		if a.States[i] != b.States[i] {
			t.Fatalf("state %d differs: %+v vs %+v", i, a.States[i], b.States[i])
		}
	}
}
