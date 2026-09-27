package trajectory

import "sort"

// objectHistory 维护单个对象按时间戳有序、去重后的观测序列。
type objectHistory struct {
	obs []Observation // 始终按 Timestamp 升序
}

// insert 将观测按时间戳归位；重复观测（同 ObjectID 且同 Timestamp）被忽略。
// 返回是否真正插入了新观测。
func (h *objectHistory) insert(o Observation) bool {
	// 二分查找第一个 Timestamp >= o.Timestamp 的位置。
	i := sort.Search(len(h.obs), func(i int) bool {
		return h.obs[i].Timestamp >= o.Timestamp
	})
	if i < len(h.obs) && h.obs[i].Timestamp == o.Timestamp {
		// 重复观测：保留先到者，忽略后到的重复。
		return false
	}
	// 在 i 处插入，保持升序。
	h.obs = append(h.obs, Observation{})
	copy(h.obs[i+1:], h.obs[i:])
	h.obs[i] = o
	return true
}

// snapshot 返回当前观测序列的副本，供一致快照查询使用。
func (h *objectHistory) snapshot() []Observation {
	out := make([]Observation, len(h.obs))
	copy(out, h.obs)
	return out
}

// gaps 识别相邻观测间隔超过 maxGapNanos 的缺失区间。
// 缺失区间只被标记，绝不用静止或零值填充。
func gapsOf(obs []Observation, maxGapNanos int64) []Gap {
	if maxGapNanos <= 0 || len(obs) < 2 {
		return nil
	}
	var gaps []Gap
	for i := 1; i < len(obs); i++ {
		if dt := obs[i].Timestamp - obs[i-1].Timestamp; dt > maxGapNanos {
			gaps = append(gaps, Gap{Start: obs[i-1].Timestamp, End: obs[i].Timestamp})
		}
	}
	return gaps
}
