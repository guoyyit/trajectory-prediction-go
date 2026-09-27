package trajectory

import (
	"math"
	"testing"
)

// 两个对象相向而行，应在未来某时刻产生冲突。
func TestDetectConflictsHeadOn(t *testing.T) {
	cfg := testConfig()
	cfg.ConflictDistance = 2.0
	svc, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	// a 从 (0,0) 向 +x，b 从 (10,0) 向 -x，两者 1s 后相距 8，逐步靠近。
	mk := func(id string, x, vx float64) Observation {
		return Observation{ObjectID: id, Timestamp: 1_000_000_000, X: x, Y: 0, VX: vx, VY: 0}
	}
	if _, err := svc.Ingest(mk("a", 0, 1)); err != nil {
		t.Fatal(err)
	}
	if _, err := svc.Ingest(mk("b", 10, -1)); err != nil {
		t.Fatal(err)
	}

	conflicts := svc.DetectConflicts()
	if len(conflicts) != 1 {
		t.Fatalf("期望 1 个冲突, 实际 %d: %+v", len(conflicts), conflicts)
	}
	c := conflicts[0]
	if c.ObjectA != "a" || c.ObjectB != "b" {
		t.Fatalf("冲突对象错误: %+v", c)
	}
	// 首次距离 < 2 的时刻：t 秒时距离 = 10 - 2t，t=5 时距离 0 < 2；t=4.000...?
	// 离散时刻 t=1..5：距离 8,6,4,2,0 → 首次 <2 在 t=5s。
	wantTs := int64(1_000_000_000) + 5*cfg.StepNanos
	if c.Timestamp != wantTs {
		t.Fatalf("冲突时刻错误: got %d, want %d", c.Timestamp, wantTs)
	}
	if c.Distance >= c.Threshold {
		t.Fatalf("冲突距离应小于阈值: %+v", c)
	}
}

// 同向同速、间距大于阈值时不应有冲突。
func TestDetectConflictsNone(t *testing.T) {
	cfg := testConfig()
	cfg.ConflictDistance = 2.0
	svc, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	mk := func(id string, x float64) Observation {
		return Observation{ObjectID: id, Timestamp: 1_000_000_000, X: x, Y: 0, VX: 1, VY: 0}
	}
	_, _ = svc.Ingest(mk("a", 0))
	_, _ = svc.Ingest(mk("b", 5)) // 始终保持 5m 间距
	if c := svc.DetectConflicts(); len(c) != 0 {
		t.Fatalf("不应检测到冲突: %+v", c)
	}
}

// 冲突提示应可区分不同对象对。
func TestDetectConflictsDistinguishable(t *testing.T) {
	cfg := testConfig()
	cfg.ConflictDistance = 3.0
	svc, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	mk := func(id string, x, y, vx float64) Observation {
		return Observation{ObjectID: id, Timestamp: 1_000_000_000, X: x, Y: y, VX: vx, VY: 0}
	}
	// a-b 相向（会冲突），c 远离（不冲突）
	_, _ = svc.Ingest(mk("a", 0, 0, 1))
	_, _ = svc.Ingest(mk("b", 8, 0, -1))
	_, _ = svc.Ingest(mk("c", 0, 100, 1))

	conflicts := svc.DetectConflicts()
	if len(conflicts) != 1 {
		t.Fatalf("期望仅 a-b 冲突, 实际 %+v", conflicts)
	}
	pair := conflicts[0].ObjectA + "-" + conflicts[0].ObjectB
	if pair != "a-b" {
		t.Fatalf("冲突对应为 a-b, 实际 %s", pair)
	}
	if math.IsNaN(conflicts[0].Distance) {
		t.Fatal("冲突距离不应为 NaN")
	}
}
