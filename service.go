package trajectory

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"sync"
)

// Service 为轨迹预测服务，支持并发写入观测与查询预测。
//
// 并发语义：写入持写锁，先落盘后入内存，保证日志顺序与内存顺序一致；
// 查询持读锁复制观测快照后即释放，预测计算在快照上进行，
// 因此查询结果来自某个一致的观测快照，不会混入半写入的观测。
type Service struct {
	cfg  Config
	mu   sync.RWMutex
	hist map[string]*objectHistory
	wal  *wal // nil 表示仅内存
	wg   sync.WaitGroup
}

// NewService 创建服务；若 cfg.DataDir 非空则加载并重放持久化历史。
// cfg 中的零值字段会被替换为 DefaultConfig 的对应值。
func NewService(cfg Config) (*Service, error) {
	cfg = normalizeConfig(cfg)
	s := &Service{cfg: cfg, hist: make(map[string]*objectHistory)}
	if cfg.DataDir != "" {
		w, replayed, err := openWAL(cfg.DataDir)
		if err != nil {
			return nil, err
		}
		s.wal = w
		// 按原始写入顺序重放，历史层负责排序与去重。
		for _, o := range replayed {
			s.insertLocked(o)
		}
	}
	return s, nil
}

// normalizeConfig 为零值字段填充默认值。
func normalizeConfig(cfg Config) Config {
	d := DefaultConfig()
	if cfg.DefaultModel == "" {
		cfg.DefaultModel = d.DefaultModel
	}
	if cfg.Steps <= 0 {
		cfg.Steps = d.Steps
	}
	if cfg.StepNanos <= 0 {
		cfg.StepNanos = d.StepNanos
	}
	if cfg.MaxGapNanos <= 0 {
		cfg.MaxGapNanos = d.MaxGapNanos
	}
	if cfg.ConflictDistance <= 0 {
		cfg.ConflictDistance = d.ConflictDistance
	}
	return cfg
}

// validate 校验观测合法性。
func validate(o Observation) error {
	if o.ObjectID == "" {
		return errors.New("观测缺少对象标识")
	}
	for _, v := range []float64{o.X, o.Y, o.VX, o.VY, o.Heading} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return fmt.Errorf("观测包含非法数值: %v", o)
		}
	}
	return nil
}

// insertLocked 在写锁内将观测插入历史（调用者须已持锁）。
func (s *Service) insertLocked(o Observation) bool {
	h, ok := s.hist[o.ObjectID]
	if !ok {
		h = &objectHistory{}
		s.hist[o.ObjectID] = h
	}
	return h.insert(o)
}

// Ingest 同步写入一条观测；返回是否为有效新观测（重复观测返回 false）。
// 观测先追加到持久化日志，再插入内存历史，二者在同一写锁内完成。
func (s *Service) Ingest(o Observation) (bool, error) {
	if err := validate(o); err != nil {
		return false, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.wal.append(o); err != nil {
		return false, err
	}
	return s.insertLocked(o), nil
}

// IngestBatch 按给定顺序写入多条观测。
func (s *Service) IngestBatch(obs []Observation) error {
	for _, o := range obs {
		if _, err := s.Ingest(o); err != nil {
			return err
		}
	}
	return nil
}

// Pipeline 返回一个内存管道，向其发送观测即异步写入服务。
// 调用方关闭该 channel 后，可调用 WaitPipelines 等待全部观测处理完毕。
func (s *Service) Pipeline(buf int) chan<- Observation {
	ch := make(chan Observation, buf)
	s.wg.Add(1)
	go func() {
		defer s.wg.Done()
		for o := range ch {
			_, _ = s.Ingest(o)
		}
	}()
	return ch
}

// WaitPipelines 等待所有已关闭的管道处理完毕。
func (s *Service) WaitPipelines() {
	s.wg.Wait()
}

// snapshotLocked 复制全部历史（调用者须已持读锁）。
func (s *Service) snapshotLocked() map[string][]Observation {
	out := make(map[string][]Observation, len(s.hist))
	for id, h := range s.hist {
		out[id] = h.snapshot()
	}
	return out
}

// Predict 返回单个对象基于一致快照的预测。
func (s *Service) Predict(objectID string) (Prediction, error) {
	s.mu.RLock()
	h, ok := s.hist[objectID]
	var snap []Observation
	if ok {
		snap = h.snapshot()
	}
	s.mu.RUnlock()
	if !ok {
		return Prediction{}, fmt.Errorf("未知对象: %q", objectID)
	}
	return predictOne(objectID, snap, s.cfg), nil
}

// PredictAll 返回当前所有已知对象的预测（基于同一读快照）。
// 输出按对象 ID 排序，保证确定性。
func (s *Service) PredictAll() []Prediction {
	s.mu.RLock()
	snap := s.snapshotLocked()
	s.mu.RUnlock()

	ids := make([]string, 0, len(snap))
	for id := range snap {
		ids = append(ids, id)
	}
	sort.Strings(ids)

	preds := make([]Prediction, 0, len(ids))
	for _, id := range ids {
		preds = append(preds, predictOne(id, snap[id], s.cfg))
	}
	return preds
}

// DetectConflicts 对所有对象的预测轨迹做冲突检测。
func (s *Service) DetectConflicts() []Conflict {
	return detectConflicts(s.PredictAll(), s.cfg.ConflictDistance)
}

// ObjectIDs 返回当前已知对象列表（排序后，保证确定性输出）。
func (s *Service) ObjectIDs() []string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	ids := make([]string, 0, len(s.hist))
	for id := range s.hist {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	return ids
}

// Close 关闭持久化并释放资源。
// 若使用了 Pipeline，调用方应先关闭管道 channel 并 WaitPipelines。
func (s *Service) Close() error {
	return s.wal.close()
}
