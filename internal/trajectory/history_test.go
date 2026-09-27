package trajectory

import (
	"testing"
	"time"
)

var t0 = time.Date(2026, 9, 27, 0, 0, 0, 0, time.UTC)

func obsAt(id string, sec int, x float64) Observation {
	return Observation{ObjectID: id, T: t0.Add(time.Duration(sec) * time.Second),
		X: x, Y: 0, VX: 1, VY: 0, Heading: 0}
}

func TestHistoryOutOfOrderInsert(t *testing.T) {
	var h history
	// Insert out of order: 2, 0, 3, 1.
	for _, sec := range []int{2, 0, 3, 1} {
		if !h.insert(obsAt("a", sec, float64(sec))) {
			t.Fatalf("insert %d rejected", sec)
		}
	}
	snap := h.snapshot()
	if len(snap) != 4 {
		t.Fatalf("want 4 obs, got %d", len(snap))
	}
	for i := 1; i < len(snap); i++ {
		if !snap[i].T.After(snap[i-1].T) {
			t.Fatalf("history not ordered at %d: %v then %v", i, snap[i-1].T, snap[i].T)
		}
	}
	latest, ok := h.latest()
	if !ok || latest.X != 3 {
		t.Fatalf("latest = %+v, want X=3", latest)
	}
}

func TestHistoryDuplicateInsert(t *testing.T) {
	var h history
	o := obsAt("a", 1, 1.5)
	if !h.insert(o) {
		t.Fatal("first insert rejected")
	}
	if h.insert(o) {
		t.Fatal("exact duplicate accepted")
	}
	// Same timestamp, different payload is a distinct sample, kept.
	different := o
	different.X = 9
	if !h.insert(different) {
		t.Fatal("same-timestamp distinct sample rejected")
	}
	if got := len(h.snapshot()); got != 2 {
		t.Fatalf("want 2 obs, got %d", got)
	}
}

func TestHistoryGaps(t *testing.T) {
	var h history
	// Samples at 0,1,2 then missing 3,4,5 then 6,7.
	for _, sec := range []int{0, 1, 2, 6, 7} {
		h.insert(obsAt("a", sec, 0))
	}
	gaps := h.gaps(time.Second)
	if len(gaps) != 1 {
		t.Fatalf("want 1 gap, got %d: %v", len(gaps), gaps)
	}
	if !gaps[0].Start.Equal(t0.Add(2*time.Second)) || !gaps[0].End.Equal(t0.Add(6*time.Second)) {
		t.Fatalf("gap = %+v, want [t0+2s, t0+6s]", gaps[0])
	}
	// No fabrication: history must contain exactly the 5 inserted samples.
	if got := len(h.snapshot()); got != 5 {
		t.Fatalf("gap must not be filled, got %d obs", got)
	}
}

func TestHistoryNoGapWhenDense(t *testing.T) {
	var h history
	for sec := 0; sec < 10; sec++ {
		h.insert(obsAt("a", sec, 0))
	}
	if gaps := h.gaps(time.Second); len(gaps) != 0 {
		t.Fatalf("unexpected gaps: %v", gaps)
	}
}
