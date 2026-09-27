package trajectory

import (
	"errors"
	"fmt"
	"path/filepath"
	"sync"
	"testing"
	"time"
)

func testConfig() Config {
	return Config{
		DefaultModel: ModelCV,
		Noise:        testNoise(),
		SamplePeriod: 100 * time.Millisecond,
	}
}

func TestServiceConcurrentIngestAndQuery(t *testing.T) {
	svc, err := New(testConfig())
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	const writers = 4
	const perWriter = 200
	var wg sync.WaitGroup
	// Writers: ingest observations (some out of order, some duplicated).
	for w := 0; w < writers; w++ {
		wg.Add(1)
		go func(w int) {
			defer wg.Done()
			id := fmt.Sprintf("obj-%d", w)
			for i := perWriter - 1; i >= 0; i-- { // reverse = out of order
				o := Observation{
					ObjectID: id, T: t0.Add(time.Duration(i) * time.Millisecond),
					X: float64(i), VX: 1,
				}
				if err := svc.Ingest(o); err != nil {
					t.Error(err)
					return
				}
				if i%3 == 0 { // duplicate every third
					if err := svc.Ingest(o); err != nil {
						t.Error(err)
						return
					}
				}
			}
		}(w)
	}
	// Readers: query predictions and gaps concurrently.
	stop := make(chan struct{})
	var rwg sync.WaitGroup
	for r := 0; r < 4; r++ {
		rwg.Add(1)
		go func(r int) {
			defer rwg.Done()
			for {
				select {
				case <-stop:
					return
				default:
				}
				id := fmt.Sprintf("obj-%d", r%writers)
				tr, err := svc.Predict(PredictRequest{ObjectID: id, Horizon: 500 * time.Millisecond, Step: 50 * time.Millisecond})
				if err != nil {
					if !errors.Is(err, ErrUnknownObject) {
						t.Errorf("predict: %v", err)
						return
					}
					continue
				}
				// Snapshot consistency: predicted times must be strictly
				// increasing and anchored after the latest observation.
				for i := 1; i < len(tr.States); i++ {
					if !tr.States[i].T.After(tr.States[i-1].T) {
						t.Errorf("predicted times out of order")
						return
					}
				}
				_ = svc.Gaps(id)
				_, _ = svc.Conflicts(200*time.Millisecond, 50*time.Millisecond, 10)
			}
		}(r)
	}
	wg.Wait()
	close(stop)
	rwg.Wait()

	// After all writes: each object must have exactly perWriter ordered obs.
	for w := 0; w < writers; w++ {
		id := fmt.Sprintf("obj-%d", w)
		svc.mu.RLock()
		h := svc.hist[id]
		snap := h.snapshot()
		svc.mu.RUnlock()
		if len(snap) != perWriter {
			t.Fatalf("%s: want %d obs, got %d", id, perWriter, len(snap))
		}
		for i := 1; i < len(snap); i++ {
			if !snap[i].T.After(snap[i-1].T) {
				t.Fatalf("%s: history out of order at %d", id, i)
			}
		}
	}
}

func TestServiceConflictDetection(t *testing.T) {
	svc, err := New(testConfig())
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()
	// Two objects approaching head-on along x.
	for i := 0; i < 5; i++ {
		tt := t0.Add(time.Duration(i) * 100 * time.Millisecond)
		mustIngest(t, svc, Observation{ObjectID: "east", T: tt, X: float64(i), VX: 10, Heading: 0})
		mustIngest(t, svc, Observation{ObjectID: "west", T: tt, X: 30 - float64(i), VX: -10, Heading: 3.14159})
	}
	// Far-away third object must not conflict.
	mustIngest(t, svc, Observation{ObjectID: "far", T: t0, X: 0, Y: 1000, VX: 1})

	conflicts, err := svc.Conflicts(2*time.Second, 100*time.Millisecond, 3.0)
	if err != nil {
		t.Fatal(err)
	}
	if len(conflicts) != 1 {
		t.Fatalf("want 1 conflict, got %d: %v", len(conflicts), conflicts)
	}
	c := conflicts[0]
	if (c.ObjectA != "east" || c.ObjectB != "west") && (c.ObjectA != "west" || c.ObjectB != "east") {
		t.Fatalf("conflict objects = %q,%q, want east/west", c.ObjectA, c.ObjectB)
	}
	if c.Distance >= 3.0 {
		t.Fatalf("conflict distance %v >= threshold", c.Distance)
	}
	if !c.T.After(t0) {
		t.Fatalf("conflict time %v not in the future", c.T)
	}
}

func TestServiceNoConflictWhenDistant(t *testing.T) {
	svc, err := New(testConfig())
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()
	mustIngest(t, svc, Observation{ObjectID: "a", T: t0, X: 0, Y: 0, VX: 1})
	mustIngest(t, svc, Observation{ObjectID: "b", T: t0, X: 100, Y: 100, VX: 1})
	conflicts, err := svc.Conflicts(time.Second, 100*time.Millisecond, 3.0)
	if err != nil {
		t.Fatal(err)
	}
	if len(conflicts) != 0 {
		t.Fatalf("unexpected conflicts: %v", conflicts)
	}
}

func TestServiceRestartRecovery(t *testing.T) {
	path := filepath.Join(t.TempDir(), "obs.jsonl")
	cfg := testConfig()
	cfg.StorePath = path

	// Session 1: ingest out-of-order with duplicates.
	svc1, err := New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	var inputs []Observation
	for _, sec := range []int{4, 2, 0, 3, 1} {
		o := Observation{ObjectID: "a", T: t0.Add(time.Duration(sec) * time.Second), X: float64(sec), VX: 1}
		inputs = append(inputs, o)
		mustIngest(t, svc1, o)
		mustIngest(t, svc1, o) // duplicate
	}
	tr1, err := svc1.Predict(PredictRequest{ObjectID: "a", Horizon: 2 * time.Second, Step: 500 * time.Millisecond})
	if err != nil {
		t.Fatal(err)
	}
	if err := svc1.Close(); err != nil {
		t.Fatal(err)
	}

	// Session 2: restart, history must be fully recovered.
	svc2, err := New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc2.Close()
	svc2.mu.RLock()
	snap := svc2.hist["a"].snapshot()
	svc2.mu.RUnlock()
	if len(snap) != 5 {
		t.Fatalf("recovered %d obs, want 5 (duplicates must be dropped)", len(snap))
	}
	for i := 1; i < len(snap); i++ {
		if !snap[i].T.After(snap[i-1].T) {
			t.Fatalf("recovered history out of order at %d", i)
		}
	}
	tr2, err := svc2.Predict(PredictRequest{ObjectID: "a", Horizon: 2 * time.Second, Step: 500 * time.Millisecond})
	if err != nil {
		t.Fatal(err)
	}
	if len(tr1.States) != len(tr2.States) {
		t.Fatal("prediction length differs after restart")
	}
	for i := range tr1.States {
		if tr1.States[i] != tr2.States[i] {
			t.Fatalf("state %d differs after restart: %+v vs %+v", i, tr1.States[i], tr2.States[i])
		}
	}
}

func TestServiceDeterministicAcrossRuns(t *testing.T) {
	// Same input sequence into two fresh services must give identical output.
	run := func() Trajectory {
		svc, err := New(testConfig())
		if err != nil {
			t.Fatal(err)
		}
		defer svc.Close()
		for _, sec := range []int{3, 0, 2, 1, 4} {
			mustIngest(t, svc, Observation{ObjectID: "a",
				T: t0.Add(time.Duration(sec) * 100 * time.Millisecond),
				X: float64(sec), VX: 2, Heading: 0.1})
		}
		tr, err := svc.Predict(PredictRequest{ObjectID: "a", Model: ModelCTRV, Horizon: time.Second, Step: 100 * time.Millisecond})
		if err != nil {
			t.Fatal(err)
		}
		return tr
	}
	a, b := run(), run()
	if len(a.States) != len(b.States) {
		t.Fatal("state count mismatch")
	}
	for i := range a.States {
		if a.States[i] != b.States[i] {
			t.Fatalf("run mismatch at state %d", i)
		}
	}
}

func TestServiceModelSelection(t *testing.T) {
	cfg := testConfig()
	cfg.DefaultModel = ModelCV
	cfg.PerObject = map[string]ModelKind{"special": ModelCTRV}
	svc, err := New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()
	mustIngest(t, svc, Observation{ObjectID: "special", T: t0, X: 0, VX: 1})
	mustIngest(t, svc, Observation{ObjectID: "plain", T: t0, X: 0, VX: 1})

	tr, err := svc.Predict(PredictRequest{ObjectID: "special", Horizon: time.Second, Step: time.Second})
	if err != nil {
		t.Fatal(err)
	}
	if tr.Model != ModelCTRV {
		t.Fatalf("per-object model = %q, want ctrv", tr.Model)
	}
	tr, err = svc.Predict(PredictRequest{ObjectID: "plain", Horizon: time.Second, Step: time.Second})
	if err != nil {
		t.Fatal(err)
	}
	if tr.Model != ModelCV {
		t.Fatalf("default model = %q, want cv", tr.Model)
	}
	// Explicit request overrides per-object config.
	tr, err = svc.Predict(PredictRequest{ObjectID: "special", Model: ModelCA, Horizon: time.Second, Step: time.Second})
	if err != nil {
		t.Fatal(err)
	}
	if tr.Model != ModelCA {
		t.Fatalf("explicit model = %q, want ca", tr.Model)
	}
}

func TestServiceUnknownObject(t *testing.T) {
	svc, err := New(testConfig())
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()
	_, err = svc.Predict(PredictRequest{ObjectID: "ghost", Horizon: time.Second, Step: time.Second})
	if !errors.Is(err, ErrUnknownObject) {
		t.Fatalf("err = %v, want ErrUnknownObject", err)
	}
}

func mustIngest(t *testing.T, svc *Service, o Observation) {
	t.Helper()
	if err := svc.Ingest(o); err != nil {
		t.Fatal(err)
	}
}
