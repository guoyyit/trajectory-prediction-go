package trajectory

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"sync"
)

// store persists observations as an append-only JSON-lines log so that
// accumulated history survives restarts. Recovery replays the log
// through the same insert logic, so duplicates or out-of-order entries
// are reconciled exactly as they were during live ingestion.
type store struct {
	path string
	mu   sync.Mutex
	f    *os.File
	enc  *json.Encoder
}

func newStore(path string) *store { return &store{path: path} }

// open lazily opens the log file for appending.
func (s *store) open() error {
	if s.f != nil {
		return nil
	}
	f, err := os.OpenFile(s.path, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o644)
	if err != nil {
		return fmt.Errorf("open store: %w", err)
	}
	s.f = f
	s.enc = json.NewEncoder(f)
	return nil
}

// append persists one observation and flushes it to disk.
func (s *store) append(o Observation) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.open(); err != nil {
		return err
	}
	if err := s.enc.Encode(o); err != nil {
		return fmt.Errorf("encode observation: %w", err)
	}
	return s.f.Sync()
}

// load returns all persisted observations in write order. A truncated
// final line (crash mid-write) is tolerated and ignored.
func (s *store) load() ([]Observation, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	f, err := os.Open(s.path)
	if errors.Is(err, os.ErrNotExist) {
		return nil, nil
	}
	if err != nil {
		return nil, fmt.Errorf("open store: %w", err)
	}
	defer f.Close()
	var out []Observation
	dec := json.NewDecoder(bufio.NewReader(f))
	for {
		var o Observation
		err := dec.Decode(&o)
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			// Tolerate a corrupt/truncated tail record.
			break
		}
		out = append(out, o)
	}
	return out, nil
}

// close flushes and closes the log file.
func (s *store) close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.f == nil {
		return nil
	}
	err := s.f.Close()
	s.f = nil
	return err
}
