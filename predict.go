package trajectory

// predictOne 基于一致的历史快照为单个对象生成预测。
// obs 必须已按时间戳升序去重；为空时返回无状态的预测。
func predictOne(objectID string, obs []Observation, cfg Config) Prediction {
	model := cfg.modelFor(objectID)
	p := Prediction{
		ObjectID: objectID,
		Model:    model,
		Gaps:     gapsOf(obs, cfg.MaxGapNanos),
	}
	if len(obs) == 0 || cfg.Steps <= 0 || cfg.StepNanos <= 0 {
		return p
	}

	mm := newMotionModel(model)
	st := mm.estimate(obs, cfg.Noise, cfg.MaxGapNanos)
	dt := float64(cfg.StepNanos) / 1e9
	t0 := obs[len(obs)-1].Timestamp

	p.States = make([]PredictedState, 0, cfg.Steps)
	for k := 1; k <= cfg.Steps; k++ {
		st = mm.propagate(st, dt, cfg.Noise)
		x, y, vx, vy, heading, unc := mm.extract(st)
		p.States = append(p.States, PredictedState{
			Timestamp:   t0 + int64(k)*cfg.StepNanos,
			X:           x,
			Y:           y,
			VX:          vx,
			VY:          vy,
			Heading:     heading,
			Uncertainty: unc,
		})
	}
	return p
}
