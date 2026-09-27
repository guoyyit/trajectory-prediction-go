package trajectory

import (
	"math"
	"testing"
)

// 用有限差分验证 CTRV 解析雅可比（转弯与近直线两种情形）。
func TestCTRVJacobianFiniteDifference(t *testing.T) {
	cases := [][]float64{
		{0, 0, 2.0, 0.5, 0.3},   // 正常转弯
		{1, -2, 1.5, -1.0, 0.0}, // 直线（ω=0）
		{0, 0, 3.0, 2.0, -0.7},  // 反向转弯
	}
	const dt = 0.5
	const h = 1e-7
	for ci, x := range cases {
		nx, F := ctrvStep(x, dt)
		for j := 0; j < 5; j++ {
			xp := make([]float64, 5)
			copy(xp, x)
			xp[j] += h
			nxp, _ := ctrvStep(xp, dt)
			for i := 0; i < 5; i++ {
				num := wrapAngle(nxp[i]-nx[i]) / h
				if i != 3 { // 只有朝向需要角度回绕
					num = (nxp[i] - nx[i]) / h
				}
				if math.Abs(num-F[i*5+j]) > 1e-4 {
					t.Fatalf("case %d: F[%d][%d] 解析值 %v, 数值值 %v", ci, i, j, F[i*5+j], num)
				}
			}
		}
	}
}

// CTRV 均值传播应与小步长数值积分一致。
func TestCTRVMeanMatchesIntegration(t *testing.T) {
	x := []float64{0, 0, 2.0, 0.3, 0.4}
	nx, _ := ctrvStep(x, 1.0)
	// 小步长欧拉积分圆周运动
	px, py, th := 0.0, 0.0, 0.3
	const n = 100000
	dt := 1.0 / n
	for i := 0; i < n; i++ {
		px += 2.0 * math.Cos(th) * dt
		py += 2.0 * math.Sin(th) * dt
		th += 0.4 * dt
	}
	if math.Abs(nx[0]-px) > 1e-4 || math.Abs(nx[1]-py) > 1e-4 {
		t.Fatalf("CTRV 均值 (%v,%v) 与数值积分 (%v,%v) 不一致", nx[0], nx[1], px, py)
	}
}
