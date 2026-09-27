package trajectory

import "testing"

func obsAt(id string, ts int64) Observation {
	return Observation{ObjectID: id, Timestamp: ts, X: float64(ts), Y: 1, VX: 1, VY: 0, Heading: 0}
}

func TestHistoryOutOfOrderInsert(t *testing.T) {
	var h objectHistory
	// 乱序到达
	for _, ts := range []int64{30, 10, 50, 20, 40} {
		if !h.insert(obsAt("a", ts)) {
			t.Fatalf("首次插入 ts=%d 应成功", ts)
		}
	}
	snap := h.snapshot()
	if len(snap) != 5 {
		t.Fatalf("期望 5 条观测, 实际 %d", len(snap))
	}
	for i := 1; i < len(snap); i++ {
		if snap[i-1].Timestamp >= snap[i].Timestamp {
			t.Fatalf("历史未按时间戳升序: %v", snap)
		}
	}
	want := []int64{10, 20, 30, 40, 50}
	for i, ts := range want {
		if snap[i].Timestamp != ts {
			t.Fatalf("位置 %d 期望 ts=%d, 实际 %d", i, ts, snap[i].Timestamp)
		}
	}
}

func TestHistoryDuplicateInsert(t *testing.T) {
	var h objectHistory
	if !h.insert(obsAt("a", 10)) {
		t.Fatal("首次插入应成功")
	}
	// 完全相同的重复观测
	if h.insert(obsAt("a", 10)) {
		t.Fatal("重复观测应被拒绝")
	}
	// 同时间戳但内容不同的观测也视为重复（保留先到者）
	dup := obsAt("a", 10)
	dup.X = 999
	if h.insert(dup) {
		t.Fatal("同时间戳观测应被去重")
	}
	snap := h.snapshot()
	if len(snap) != 1 {
		t.Fatalf("去重后应只剩 1 条, 实际 %d", len(snap))
	}
	if snap[0].X != 10 {
		t.Fatalf("应保留先到的观测, 实际 X=%v", snap[0].X)
	}
}

func TestHistoryGapDetection(t *testing.T) {
	obs := []Observation{
		obsAt("a", 1_000_000_000),
		obsAt("a", 2_000_000_000),
		// 缺失 (2s, 10s)
		obsAt("a", 10_000_000_000),
		obsAt("a", 11_000_000_000),
	}
	gaps := gapsOf(obs, 5_000_000_000)
	if len(gaps) != 1 {
		t.Fatalf("期望识别 1 段缺失, 实际 %d", len(gaps))
	}
	if gaps[0].Start != 2_000_000_000 || gaps[0].End != 10_000_000_000 {
		t.Fatalf("缺失区间错误: %+v", gaps[0])
	}
	// 无缺失时
	if g := gapsOf(obs[:2], 5_000_000_000); len(g) != 0 {
		t.Fatalf("不应识别出缺失: %v", g)
	}
}
