package trajectory

import (
	"math"
	"sort"
)

// detectConflicts 在多条预测轨迹间检测未来冲突。
// 当任意两个对象在相同预测时刻的距离小于 threshold 时，
// 记录首次进入阈值的时刻，并返回可区分的冲突提示。
// 输出按 (Timestamp, ObjectA, ObjectB) 排序，保证确定性。
func detectConflicts(preds []Prediction, threshold float64) []Conflict {
	if threshold <= 0 || len(preds) < 2 {
		return nil
	}
	// 按对象 ID 排序，保证遍历顺序确定。
	sorted := make([]Prediction, len(preds))
	copy(sorted, preds)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i].ObjectID < sorted[j].ObjectID })

	// 为每条预测建立 时刻 -> 位置 索引。
	type pos struct{ x, y float64 }
	tracks := make([]map[int64]pos, len(sorted))
	for i, p := range sorted {
		m := make(map[int64]pos, len(p.States))
		for _, st := range p.States {
			m[st.Timestamp] = pos{st.X, st.Y}
		}
		tracks[i] = m
	}

	var conflicts []Conflict
	for i := 0; i < len(sorted); i++ {
		for j := i + 1; j < len(sorted); j++ {
			// 收集共同预测时刻并排序。
			var common []int64
			for t := range tracks[i] {
				if _, ok := tracks[j][t]; ok {
					common = append(common, t)
				}
			}
			sort.Slice(common, func(a, b int) bool { return common[a] < common[b] })

			for _, t := range common {
				a, b := tracks[i][t], tracks[j][t]
				d := math.Hypot(a.x-b.x, a.y-b.y)
				if d < threshold {
					conflicts = append(conflicts, Conflict{
						ObjectA:   sorted[i].ObjectID,
						ObjectB:   sorted[j].ObjectID,
						Timestamp: t,
						Distance:  d,
						Threshold: threshold,
					})
					break // 每对对象只报告首次进入阈值的时刻
				}
			}
		}
	}

	sort.Slice(conflicts, func(i, j int) bool {
		if conflicts[i].Timestamp != conflicts[j].Timestamp {
			return conflicts[i].Timestamp < conflicts[j].Timestamp
		}
		if conflicts[i].ObjectA != conflicts[j].ObjectA {
			return conflicts[i].ObjectA < conflicts[j].ObjectA
		}
		return conflicts[i].ObjectB < conflicts[j].ObjectB
	})
	return conflicts
}
