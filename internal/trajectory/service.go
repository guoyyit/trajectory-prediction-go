package trajectory

import (
	"errors"
	"fmt"
	"sort"
	"sync"
	"time"
)

// Config controls the service.
type Config struct {
	DefaultModel ModelKind            `json:"default_model"` // global default model
	PerObject    map[string]ModelKind `json:"per_object"`    // per-object model override
	Noise        NoiseConfig          `json:"noise"`
	SamplePeriod time.Duration        `json:"sample_period"` // expected sampling period, for gap detection
	StorePath    string               `json:"store_path"`    // empty disables persistence
}

// ErrUnknownObject is returned when querying an object with no history.
var ErrUnknownObject = errors.New("trajectory: unknown object")

// Service is the trajectory prediction service. Writes and queries may
// run concurrently: every query operates on a consistent snapshot taken
// under a read lock, so it never observes a half-written observation.
type Service struct {
	cfg   Config
	mu    sync.RWMutex
	hist  map[string]*history
	store *store
}

// New creates a service, restoring persisted history if configured.
// Persisted observations are replayed through the same insert path as
// live ingestion, so duplicates are dropped and order is restored.
func New(cfg Config) (*Service, error) {
	if cfg.DefaultModel == "" {
		cfg.DefaultModel = ModelCV
	}
	s := &Service{
		cfg:  cfg,
		hist: make(map[string]*history),
	}
	if cfg.StorePath != "" {
		s.store = newStore(cfg.StorePath)
		obs, err := s.store.load()
		if err != nil {
			return nil, err
		}
		for _, o := range obs {
			s.historyFor(o.ObjectID).insert(o)
		}
	}
	return s, nil
}

// historyFor returns (creating if needed) the history for an object.
// Caller must hold the write lock.
func (s *Service) historyFor(id string) *history {
	h, ok := s.hist[id]
	if !ok {
		h = &history{}
		s.hist[id] = h
	}
	return h
}

// Ingest writes one observation. Out-of-order observations are placed
// correctly; exact duplicates are dropped. The observation is appended
// to the persistent log (if enabled) under the same write lock, so the
// log order matches the serialized ingestion order.
func (s *Service) Ingest(o Observation) error {
	if o.ObjectID == "" {
		return errors.New("trajectory: observation with empty object id")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.store != nil {
		if err := s.store.append(o); err != nil {
			return err
		}
	}
	s.historyFor(o.ObjectID).insert(o)
	return nil
}

// modelFor resolves the model for a request: explicit request model
// wins, then per-object override, then the global default.
func (s *Service) modelFor(req PredictRequest) ModelKind {
	if req.Model != "" {
		return req.Model
	}
	if m, ok := s.cfg.PerObject[req.ObjectID]; ok {
		return m
	}
	return s.cfg.DefaultModel
}

// Predict returns a predicted trajectory for one object, computed from
// a consistent snapshot of its history.
func (s *Service) Predict(req PredictRequest) (Trajectory, error) {
	if req.Horizon <= 0 || req.Step <= 0 {
		return Trajectory{}, errors.New("trajectory: horizon and step must be positive")
	}
	s.mu.RLock()
	h, ok := s.hist[req.ObjectID]
	var snap []Observation
	if ok {
		snap = h.snapshot()
	}
	s.mu.RUnlock()
	if !ok || len(snap) == 0 {
		return Trajectory{}, fmt.Errorf("%w: %q", ErrUnknownObject, req.ObjectID)
	}
	return predict(snap, s.modelFor(req), s.cfg.Noise, req.Horizon, req.Step), nil
}

// Conflicts predicts all known objects on a common time grid and
// reports pairwise conflicts within the distance threshold.
func (s *Service) Conflicts(horizon, step time.Duration, threshold float64) ([]Conflict, error) {
	if horizon <= 0 || step <= 0 {
		return nil, errors.New("trajectory: horizon and step must be positive")
	}
	if threshold <= 0 {
		return nil, errors.New("trajectory: threshold must be positive")
	}
	s.mu.RLock()
	snaps := make(map[string][]Observation, len(s.hist))
	for id, h := range s.hist {
		snaps[id] = h.snapshot()
	}
	s.mu.RUnlock()
	ids := make([]string, 0, len(snaps))
	for id := range snaps {
		ids = append(ids, id)
	}
	sort.Strings(ids) // deterministic trajectory order
	trajs := make([]Trajectory, 0, len(ids))
	for _, id := range ids {
		tr := predict(snaps[id], s.modelFor(PredictRequest{ObjectID: id}), s.cfg.Noise, horizon, step)
		if len(tr.States) > 0 {
			trajs = append(trajs, tr)
		}
	}
	return detectConflicts(trajs, threshold), nil
}

// Gaps reports missing intervals for an object, based on the configured
// expected sampling period. Unknown objects yield nil.
func (s *Service) Gaps(objectID string) []Gap {
	s.mu.RLock()
	defer s.mu.RUnlock()
	h, ok := s.hist[objectID]
	if !ok {
		return nil
	}
	return h.gaps(s.cfg.SamplePeriod)
}

// ObjectIDs returns the known object IDs in sorted order.
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

// Close releases resources (flushes the persistent log).
func (s *Service) Close() error {
	if s.store != nil {
		return s.store.close()
	}
	return nil
}
