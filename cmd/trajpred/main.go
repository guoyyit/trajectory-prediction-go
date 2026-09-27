// Command trajpred demonstrates the trajectory prediction service over
// a local in-memory pipe (no real sensors or external services).
package main

import (
	"encoding/json"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"time"

	"github.com/guoyyit/trajectory-prediction-go/internal/trajectory"
)

func main() {
	dir, err := os.MkdirTemp("", "trajpred")
	if err != nil {
		log.Fatal(err)
	}
	defer os.RemoveAll(dir)

	svc, err := trajectory.New(trajectory.Config{
		DefaultModel: trajectory.ModelCTRV,
		Noise: trajectory.NoiseConfig{
			InitialPosVar: 0.5, InitialVelVar: 0.3, InitialAccVar: 0.2,
			ProcessPosVar: 0.05, ProcessVelVar: 0.2,
			ProcessAccVar: 0.1, ProcessYawVar: 0.02,
		},
		SamplePeriod: 100 * time.Millisecond,
		StorePath:    filepath.Join(dir, "observations.jsonl"),
	})
	if err != nil {
		log.Fatal(err)
	}
	defer svc.Close()

	// Simulated observation source feeding an in-memory pipe.
	pipe := make(chan trajectory.Observation, 16)
	go func() {
		defer close(pipe)
		base := time.Date(2026, 9, 27, 12, 0, 0, 0, time.UTC)
		for i := 0; i < 10; i++ {
			t := base.Add(time.Duration(i) * 100 * time.Millisecond)
			pipe <- trajectory.Observation{
				ObjectID: "veh-1", T: t,
				X: float64(i), Y: 0, VX: 10, VY: 0, Heading: 0,
			}
			pipe <- trajectory.Observation{
				ObjectID: "veh-2", T: t,
				X: float64(20 - i), Y: 0.5, VX: -10, VY: 0, Heading: 3.14159,
			}
		}
	}()
	for o := range pipe {
		if err := svc.Ingest(o); err != nil {
			log.Fatal(err)
		}
	}

	tr, err := svc.Predict(trajectory.PredictRequest{
		ObjectID: "veh-1", Horizon: 2 * time.Second, Step: 200 * time.Millisecond,
	})
	if err != nil {
		log.Fatal(err)
	}
	enc := json.NewEncoder(os.Stdout)
	enc.SetIndent("", "  ")
	fmt.Println("== predicted trajectory for veh-1 ==")
	_ = enc.Encode(tr)

	conflicts, err := svc.Conflicts(2*time.Second, 200*time.Millisecond, 5.0)
	if err != nil {
		log.Fatal(err)
	}
	fmt.Println("== conflicts (threshold 5m) ==")
	_ = enc.Encode(conflicts)
}
