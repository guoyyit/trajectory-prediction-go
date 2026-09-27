package trajectory

import (
	"math"
	"testing"
)

func testConfig() Config {
	cfg := DefaultConfig()
	cfg.Steps = 5
	cfg.StepNanos = 1_000_000_000 // 1s
	return cfg
}

// 匀速模型：预测应沿速度方向线性延伸。
func TestPredictCVLinear(t *testing.T) {
	cfg := testConfig()
	cfg.DefaultModel = ModelCV
	obs := []Observation{
		{ObjectID: "a", Timestamp: 1_000_000_000, X: 0, Y: 0, VX: 2, VY: 1},
		{ObjectID: "a", Timestamp: 2_000_000_000, X: 2, Y: 1, VX: 2, VY: 1},
	}
	p := predictOne("a", obs, cfg)
	if len(p.States) != cfg.Steps {
		t.Fatalf("期望 %d 个预测时刻, 实际 %d", cfg.Steps, len(p.States))
	}
	for k, st := range p.States {
		wantX := 2 + 2*float64(k+1)
		wantY := 1 + 1*float64(k+1)
		if math.Abs(st.X-wantX) > 1e-9 || math.Abs(st.Y-wantY) > 1e-9 {
			t.Fatalf("第 %d 步位置错误: got (%v,%v), want (%v,%v)", k, st.X, st.Y, wantX, wantY)
		}
		if math.Abs(st.VX-2) > 1e-9 || math.Abs(st.VY-1) > 1e-9 {
			t.Fatalf("第 %d 步速度应保持不变: got (%v,%v)", k, st.VX, st.VY)
		}
		if st.Timestamp != 2_000_000_000+int64(k+1)*1_000_000_000 {
			t.Fatalf("第 %d 步时间戳错误: %d", k, st.Timestamp)
		}
	}
}

// 匀加速模型：三点同段观测应估计出加速度并影响预测。
func TestPredictCAAcceleration(t *testing.T) {
	cfg := testConfig()
	cfg.DefaultModel = ModelCA
	// x = t² (t 以秒计)，三点 t=1,2,3
	obs := []Observation{
		{ObjectID: "a", Timestamp: 1_000_000_000, X: 1, Y: 0},
		{ObjectID: "a", Timestamp: 2_000_000_000, X: 4, Y: 0},
		{ObjectID: "a", Timestamp: 3_000_000_000, X: 9, Y: 0},
	}
	p := predictOne("a", obs, cfg)
	// 估计加速度应约为 2 m/s²；1s 后位置约 9 + 7*1 + 0.5*2*1 = 16
	st := p.States[0]
	if math.Abs(st.X-16) > 1e-6 {
		t.Fatalf("CA 第一步预测应约为 16, 实际 %v", st.X)
	}
}

// 恒定转弯率模型：带朝向变化的观测应产生转弯轨迹。
func TestPredictCTRVTurns(t *testing.T) {
	cfg := testConfig()
	cfg.DefaultModel = ModelCTRV
	obs := []Observation{
		{ObjectID: "a", Timestamp: 1_000_000_000, X: 0, Y: 0, VX: 1, VY: 0, Heading: 0},
		{ObjectID: "a", Timestamp: 2_000_000_000, X: 1, Y: 0.1, VX: 1, VY: 0, Heading: 0.2},
	}
	p := predictOne("a", obs, cfg)
	if p.Model != ModelCTRV {
		t.Fatalf("模型应为 CTRV, 实际 %v", p.Model)
	}
	// 转弯率 > 0，朝向应逐步增大
	prev := 0.2
	for _, st := range p.States {
		if st.Heading <= prev {
			t.Fatalf("CTRV 预测朝向应持续增大: prev=%v got=%v", prev, st.Heading)
		}
		prev = st.Heading
	}
}

// 不确定度应随预测时刻单调不减。
func TestUncertaintyMonotonic(t *testing.T) {
	for _, m := range []ModelType{ModelCV, ModelCA, ModelCTRV} {
		cfg := testConfig()
		cfg.DefaultModel = m
		obs := []Observation{
			{ObjectID: "a", Timestamp: 1_000_000_000, X: 0, Y: 0, VX: 1, VY: 0, Heading: 0},
			{ObjectID: "a", Timestamp: 2_000_000_000, X: 1, Y: 0, VX: 1, VY: 0, Heading: 0},
			{ObjectID: "a", Timestamp: 3_000_000_000, X: 2, Y: 0, VX: 1, VY: 0, Heading: 0},
		}
		p := predictOne("a", obs, cfg)
		prev := -1.0
		for k, st := range p.States {
			if st.Uncertainty.PosStd < prev-1e-12 {
				t.Fatalf("模型 %s 第 %d 步 PosStd 下降: %v -> %v", m, k, prev, st.Uncertainty.PosStd)
			}
			prev = st.Uncertainty.PosStd
		}
	}
}

// 不同噪声参数应产生可区分的不确定度。
func TestUncertaintyDependsOnNoise(t *testing.T) {
	obs := []Observation{
		{ObjectID: "a", Timestamp: 1_000_000_000, X: 0, Y: 0, VX: 1, VY: 0},
		{ObjectID: "a", Timestamp: 2_000_000_000, X: 1, Y: 0, VX: 1, VY: 0},
	}
	low := testConfig()
	low.Noise.ProcessVel = 0.01
	high := testConfig()
	high.Noise.ProcessVel = 10.0
	pl := predictOne("a", obs, low)
	ph := predictOne("a", obs, high)
	for k := range pl.States {
		if ph.States[k].Uncertainty.PosStd <= pl.States[k].Uncertainty.PosStd {
			t.Fatalf("第 %d 步: 高过程噪声的不确定度应更大 (%v <= %v)",
				k, ph.States[k].Uncertainty.PosStd, pl.States[k].Uncertainty.PosStd)
		}
	}
}

// 缺失段不得被当作静止或零值：跨缺失段不应使用差分速度。
func TestGapNotTreatedAsStatic(t *testing.T) {
	cfg := testConfig()
	cfg.DefaultModel = ModelCV
	cfg.MaxGapNanos = 5_000_000_000
	obs := []Observation{
		{ObjectID: "a", Timestamp: 1_000_000_000, X: 0, Y: 0, VX: 1, VY: 0},
		// 大间隔（缺失段）
		{ObjectID: "a", Timestamp: 100_000_000_000, X: 5, Y: 0, VX: 1, VY: 0},
	}
	p := predictOne("a", obs, cfg)
	// 应识别出缺失段
	if len(p.Gaps) != 1 {
		t.Fatalf("应识别 1 段缺失, 实际 %v", p.Gaps)
	}
	// 若错误地把缺失段当作连续段做差分，速度会接近 0（5m/99s），
	// 预测位置将几乎不动；正确行为是使用最新观测上报的速度 1 m/s。
	st := p.States[0]
	if math.Abs(st.X-6) > 1e-9 {
		t.Fatalf("跨缺失段不应差分: 期望 X≈6, 实际 %v", st.X)
	}
}

// 按对象选择模型应覆盖全局默认。
func TestPerObjectModelOverride(t *testing.T) {
	cfg := testConfig()
	cfg.DefaultModel = ModelCV
	cfg.ObjectModels = map[string]ModelType{"turner": ModelCTRV}
	obs := []Observation{
		{ObjectID: "turner", Timestamp: 1_000_000_000, X: 0, Y: 0, VX: 1, Heading: 0},
		{ObjectID: "turner", Timestamp: 2_000_000_000, X: 1, Y: 0, VX: 1, Heading: 0.1},
	}
	if p := predictOne("turner", obs, cfg); p.Model != ModelCTRV {
		t.Fatalf("对象级模型覆盖未生效: %v", p.Model)
	}
	if p := predictOne("other", obs, cfg); p.Model != ModelCV {
		t.Fatalf("全局默认模型未生效: %v", p.Model)
	}
}
