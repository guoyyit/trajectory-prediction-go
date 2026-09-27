package trajectory

import (
	"encoding/json"
	"fmt"
	"math/rand"
	"sync"
	"testing"
)

// 并发写入与查询：查询不得 panic，历史不得错乱。
func TestConcurrentWriteAndQuery(t *testing.T) {
	cfg := testConfig()
	svc, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	const objects = 8
	const perObject = 200

	// 写入者：每个对象由独立 goroutine 乱序写入。
	var writeWg sync.WaitGroup
	for o := 0; o < objects; o++ {
		writeWg.Add(1)
		go func(o int) {
			defer writeWg.Done()
			id := fmt.Sprintf("obj-%d", o)
			rng := rand.New(rand.NewSource(int64(o)))
			order := rng.Perm(perObject)
			for _, k := range order {
				_, err := svc.Ingest(Observation{
					ObjectID:  id,
					Timestamp: int64(k+1) * 1_000_000_000,
					X:         float64(k), Y: float64(o),
					VX: 1, VY: 0,
				})
				if err != nil {
					t.Error(err)
					return
				}
			}
		}(o)
	}
	// 查询者：与写入并发执行，直到写入全部完成。
	stop := make(chan struct{})
	var readWg sync.WaitGroup
	for r := 0; r < 4; r++ {
		readWg.Add(1)
		go func() {
			defer readWg.Done()
			for {
				select {
				case <-stop:
					return
				default:
				}
				_ = svc.PredictAll()
				_ = svc.DetectConflicts()
				_, _ = svc.Predict("obj-0")
			}
		}()
	}

	writeWg.Wait()
	close(stop)
	readWg.Wait()

	// 验证每个对象历史严格升序且数量正确（无丢失、无重复）。
	for o := 0; o < objects; o++ {
		id := fmt.Sprintf("obj-%d", o)
		svc.mu.RLock()
		h := svc.hist[id]
		var snap []Observation
		if h != nil {
			snap = h.snapshot()
		}
		svc.mu.RUnlock()
		if len(snap) != perObject {
			t.Fatalf("对象 %s 期望 %d 条观测, 实际 %d", id, perObject, len(snap))
		}
		for i := 1; i < len(snap); i++ {
			if snap[i-1].Timestamp >= snap[i].Timestamp {
				t.Fatalf("对象 %s 历史顺序错乱", id)
			}
		}
	}
}

// 相同输入顺序下预测结果必须逐条一致。
func TestDeterministicPredictions(t *testing.T) {
	build := func() string {
		cfg := testConfig()
		cfg.Seed = 7
		svc, err := NewService(cfg)
		if err != nil {
			t.Fatal(err)
		}
		defer svc.Close()
		rng := rand.New(rand.NewSource(123))
		for _, id := range []string{"a", "b", "c"} {
			order := rng.Perm(50)
			for _, k := range order {
				_, _ = svc.Ingest(Observation{
					ObjectID:  id,
					Timestamp: int64(k+1) * 100_000_000,
					X:         float64(k) * 1.5, Y: float64(k % 3),
					VX: 1.5, VY: 0.2, Heading: 0.1,
				})
			}
		}
		data, err := json.Marshal(svc.PredictAll())
		if err != nil {
			t.Fatal(err)
		}
		return string(data)
	}
	first := build()
	for i := 0; i < 3; i++ {
		if got := build(); got != first {
			t.Fatalf("第 %d 次重建结果不一致", i)
		}
	}
}

// 内存管道写入应被完整消费。
func TestPipelineIngest(t *testing.T) {
	cfg := testConfig()
	svc, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	ch := svc.Pipeline(16)
	const n = 100
	for k := 0; k < n; k++ {
		ch <- Observation{ObjectID: "p", Timestamp: int64(k+1) * 1_000_000_000, X: float64(k), VX: 1}
	}
	close(ch)
	svc.WaitPipelines()

	p, err := svc.Predict("p")
	if err != nil {
		t.Fatal(err)
	}
	if len(p.States) != cfg.Steps {
		t.Fatalf("管道写入后预测状态数错误: %d", len(p.States))
	}
}

// 重启后应恢复全部历史，不丢失、不重复、不跳过。
func TestRestartRecovery(t *testing.T) {
	dir := t.TempDir()
	cfg := testConfig()
	cfg.DataDir = dir

	// 第一次运行：乱序 + 重复写入。
	svc1, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	seq := []Observation{
		{ObjectID: "a", Timestamp: 3_000_000_000, X: 3, VX: 1},
		{ObjectID: "a", Timestamp: 1_000_000_000, X: 1, VX: 1},
		{ObjectID: "a", Timestamp: 2_000_000_000, X: 2, VX: 1},
		{ObjectID: "a", Timestamp: 2_000_000_000, X: 2, VX: 1}, // 重复
		{ObjectID: "b", Timestamp: 1_000_000_000, X: 0, Y: 5, VX: 1},
	}
	var accepted int
	for _, o := range seq {
		ok, err := svc1.Ingest(o)
		if err != nil {
			t.Fatal(err)
		}
		if ok {
			accepted++
		}
	}
	if accepted != 4 {
		t.Fatalf("期望 4 条有效观测, 实际 %d", accepted)
	}
	before, err := json.Marshal(svc1.PredictAll())
	if err != nil {
		t.Fatal(err)
	}
	if err := svc1.Close(); err != nil {
		t.Fatal(err)
	}

	// 模拟重启：同目录重新打开。
	svc2, err := NewService(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc2.Close()
	after, err := json.Marshal(svc2.PredictAll())
	if err != nil {
		t.Fatal(err)
	}
	if string(before) != string(after) {
		t.Fatalf("重启后预测不一致:\nbefore=%s\nafter=%s", before, after)
	}
	if ids := svc2.ObjectIDs(); len(ids) != 2 || ids[0] != "a" || ids[1] != "b" {
		t.Fatalf("重启后对象列表错误: %v", ids)
	}

	// 重启后继续写入应正常工作且不产生重复。
	if ok, _ := svc2.Ingest(seq[2]); ok {
		t.Fatal("重启后重复观测仍应被去重")
	}
	if ok, _ := svc2.Ingest(Observation{ObjectID: "a", Timestamp: 4_000_000_000, X: 4, VX: 1}); !ok {
		t.Fatal("重启后新观测应被接受")
	}
}
