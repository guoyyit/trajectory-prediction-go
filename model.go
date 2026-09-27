package trajectory

import "math"

// stateVector 为某运动模型下的状态均值与协方差（行主序 n*n）。
type stateVector struct {
	x []float64
	P []float64
}

// motionModel 抽象不同运动模型的状态估计与传播。
type motionModel interface {
	// dim 返回状态维度。
	dim() int
	// estimate 根据有序历史观测估计当前状态（观测已去重排序）。
	estimate(obs []Observation, noise NoiseConfig, maxGapNanos int64) stateVector
	// propagate 将状态向前传播 dt 秒，并叠加过程噪声。
	propagate(st stateVector, dt float64, noise NoiseConfig) stateVector
	// extract 从状态中提取预测输出所需的量。
	extract(st stateVector) (x, y, vx, vy, heading float64, unc Uncertainty)
}

// newMotionModel 按类型构造运动模型。
func newMotionModel(m ModelType) motionModel {
	switch m {
	case ModelCA:
		return caModel{}
	case ModelCTRV:
		return ctrvModel{}
	default:
		return cvModel{}
	}
}

// ---------- 通用小矩阵工具（n <= 6，行主序） ----------

func matZeros(n int) []float64 { return make([]float64, n*n) }

func matIdent(n int) []float64 {
	m := matZeros(n)
	for i := 0; i < n; i++ {
		m[i*n+i] = 1
	}
	return m
}

func matMul(a, b []float64, n int) []float64 {
	out := matZeros(n)
	for i := 0; i < n; i++ {
		for k := 0; k < n; k++ {
			aik := a[i*n+k]
			if aik == 0 {
				continue
			}
			for j := 0; j < n; j++ {
				out[i*n+j] += aik * b[k*n+j]
			}
		}
	}
	return out
}

func matTranspose(a []float64, n int) []float64 {
	out := matZeros(n)
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			out[j*n+i] = a[i*n+j]
		}
	}
	return out
}

func matAddInPlace(a, b []float64) {
	for i := range a {
		a[i] += b[i]
	}
}

// propagateLinear 执行 x'=Fx, P'=F P Fᵀ + Q。
func propagateLinear(st stateVector, F, Q []float64, n int) stateVector {
	x := make([]float64, n)
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			x[i] += F[i*n+j] * st.x[j]
		}
	}
	P := matMul(matMul(F, st.P, n), matTranspose(F, n), n)
	matAddInPlace(P, Q)
	return stateVector{x: x, P: P}
}

// wrapAngle 将角度规范到 (-π, π]。
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// inSegment 判断相邻两条观测是否属于同一连续段（间隔不超过 maxGap）。
func inSegment(a, b Observation, maxGapNanos int64) bool {
	dt := b.Timestamp - a.Timestamp
	return dt > 0 && (maxGapNanos <= 0 || dt <= maxGapNanos)
}

// speedHeadingUnc 由速度均值与协方差近似速度大小与朝向的不确定度。
func speedHeadingUnc(vx, vy, pvx, pvy, pvxy float64) (speedStd, headingStd float64) {
	s2 := vx*vx + vy*vy
	if s2 < 1e-12 {
		// 速度接近零时朝向不可知，给最大不确定度。
		return math.Sqrt(pvx + pvy), math.Pi
	}
	// speed = sqrt(vx²+vy²)，梯度为 (vx/s, vy/s)，s = sqrt(s2)。
	varSpeed := (vx*vx*pvx + vy*vy*pvy + 2*vx*vy*pvxy) / s2
	// heading = atan2(vy, vx)，梯度为 (-vy/s², vx/s²)。
	varHeading := (vy*vy*pvx + vx*vx*pvy - 2*vx*vy*pvxy) / (s2 * s2)
	if varSpeed < 0 {
		varSpeed = 0
	}
	if varHeading < 0 {
		varHeading = 0
	}
	return math.Sqrt(varSpeed), math.Sqrt(varHeading)
}

// ---------- 匀速模型 CV：状态 [x, y, vx, vy] ----------

type cvModel struct{}

func (cvModel) dim() int { return 4 }

func (m cvModel) estimate(obs []Observation, noise NoiseConfig, maxGapNanos int64) stateVector {
	last := obs[len(obs)-1]
	vx, vy := last.VX, last.VY
	// 若最后两条观测处于同一连续段，用位置差分修正速度；
	// 跨缺失段时不做差分，避免把缺失当作静止或零值。
	if len(obs) >= 2 && inSegment(obs[len(obs)-2], last, maxGapNanos) {
		prev := obs[len(obs)-2]
		dt := float64(last.Timestamp-prev.Timestamp) / 1e9
		vx = (last.X - prev.X) / dt
		vy = (last.Y - prev.Y) / dt
	}
	P := matZeros(4)
	P[0*4+0] = noise.InitialPosVar
	P[1*4+1] = noise.InitialPosVar
	P[2*4+2] = noise.InitialVelVar
	P[3*4+3] = noise.InitialVelVar
	return stateVector{x: []float64{last.X, last.Y, vx, vy}, P: P}
}

func (m cvModel) propagate(st stateVector, dt float64, noise NoiseConfig) stateVector {
	F := matIdent(4)
	F[0*4+2] = dt
	F[1*4+3] = dt
	// 白噪声加速度模型：Q 轴块 = q * [[dt³/3, dt²/2],[dt²/2, dt]]。
	q := noise.ProcessVel
	Q := matZeros(4)
	setAxisBlock2(Q, 0, 2, q, dt)
	setAxisBlock2(Q, 1, 3, q, dt)
	return propagateLinear(st, F, Q, 4)
}

func (m cvModel) extract(st stateVector) (x, y, vx, vy, heading float64, unc Uncertainty) {
	x, y, vx, vy = st.x[0], st.x[1], st.x[2], st.x[3]
	heading = math.Atan2(vy, vx)
	unc.PosCov = [2][2]float64{
		{st.P[0*4+0], st.P[0*4+1]},
		{st.P[1*4+0], st.P[1*4+1]},
	}
	unc.PosStd = math.Sqrt(st.P[0*4+0] + st.P[1*4+1])
	unc.SpeedStd, unc.HeadingStd = speedHeadingUnc(vx, vy, st.P[2*4+2], st.P[3*4+3], st.P[2*4+3])
	return
}

// setAxisBlock2 将 2x2 运动学噪声块写入 (posIdx, velIdx) 指定的两行两列。
func setAxisBlock2(Q []float64, posIdx, velIdx int, q, dt float64) {
	n := int(math.Sqrt(float64(len(Q))))
	Q[posIdx*n+posIdx] += q * dt * dt * dt / 3
	Q[posIdx*n+velIdx] += q * dt * dt / 2
	Q[velIdx*n+posIdx] += q * dt * dt / 2
	Q[velIdx*n+velIdx] += q * dt
}

// ---------- 匀加速模型 CA：状态 [x, y, vx, vy, ax, ay] ----------

type caModel struct{}

func (caModel) dim() int { return 6 }

func (m caModel) estimate(obs []Observation, noise NoiseConfig, maxGapNanos int64) stateVector {
	last := obs[len(obs)-1]
	vx, vy := last.VX, last.VY
	ax, ay := 0.0, 0.0
	n := len(obs)
	switch {
	case n >= 3 && inSegment(obs[n-3], obs[n-2], maxGapNanos) && inSegment(obs[n-2], obs[n-1], maxGapNanos):
		// 同段内三点：非均匀间距差分估计加速度。
		o1, o2, o3 := obs[n-3], obs[n-2], obs[n-1]
		dt1 := float64(o2.Timestamp-o1.Timestamp) / 1e9
		dt2 := float64(o3.Timestamp-o2.Timestamp) / 1e9
		v1x := (o2.X - o1.X) / dt1
		v1y := (o2.Y - o1.Y) / dt1
		v2x := (o3.X - o2.X) / dt2
		v2y := (o3.Y - o2.Y) / dt2
		ax = 2 * (v2x - v1x) / (dt1 + dt2)
		ay = 2 * (v2y - v1y) / (dt1 + dt2)
		// 末端速度取最后一段差分速度外推半个区间。
		vx = v2x + ax*dt2/2
		vy = v2y + ay*dt2/2
	case n >= 2 && inSegment(obs[n-2], obs[n-1], maxGapNanos):
		prev := obs[n-2]
		dt := float64(last.Timestamp-prev.Timestamp) / 1e9
		vx = (last.X - prev.X) / dt
		vy = (last.Y - prev.Y) / dt
	}
	P := matZeros(6)
	P[0*6+0] = noise.InitialPosVar
	P[1*6+1] = noise.InitialPosVar
	P[2*6+2] = noise.InitialVelVar
	P[3*6+3] = noise.InitialVelVar
	P[4*6+4] = noise.InitialAccelVar
	P[5*6+5] = noise.InitialAccelVar
	return stateVector{x: []float64{last.X, last.Y, vx, vy, ax, ay}, P: P}
}

func (m caModel) propagate(st stateVector, dt float64, noise NoiseConfig) stateVector {
	F := matIdent(6)
	F[0*6+2] = dt
	F[1*6+3] = dt
	F[0*6+4] = dt * dt / 2
	F[1*6+5] = dt * dt / 2
	F[2*6+4] = dt
	F[3*6+5] = dt
	// 白噪声加加速度（jerk）模型。
	q := noise.ProcessAccel
	Q := matZeros(6)
	setAxisBlock3(Q, 0, 2, 4, q, dt)
	setAxisBlock3(Q, 1, 3, 5, q, dt)
	return propagateLinear(st, F, Q, 6)
}

func (m caModel) extract(st stateVector) (x, y, vx, vy, heading float64, unc Uncertainty) {
	x, y, vx, vy = st.x[0], st.x[1], st.x[2], st.x[3]
	heading = math.Atan2(vy, vx)
	unc.PosCov = [2][2]float64{
		{st.P[0*6+0], st.P[0*6+1]},
		{st.P[1*6+0], st.P[1*6+1]},
	}
	unc.PosStd = math.Sqrt(st.P[0*6+0] + st.P[1*6+1])
	unc.SpeedStd, unc.HeadingStd = speedHeadingUnc(vx, vy, st.P[2*6+2], st.P[3*6+3], st.P[2*6+3])
	return
}

// setAxisBlock3 将 3x3 运动学噪声块写入 (pos, vel, accel) 指定的三行三列。
func setAxisBlock3(Q []float64, p, v, a int, q, dt float64) {
	n := int(math.Sqrt(float64(len(Q))))
	dt2, dt3, dt4, dt5 := dt*dt, dt*dt*dt, dt*dt*dt*dt, dt*dt*dt*dt*dt
	idx := []int{p, v, a}
	vals := [3][3]float64{
		{dt5 / 20, dt4 / 8, dt3 / 6},
		{dt4 / 8, dt3 / 3, dt2 / 2},
		{dt3 / 6, dt2 / 2, dt},
	}
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			Q[idx[i]*n+idx[j]] += q * vals[i][j]
		}
	}
}

// ---------- 恒定转弯率模型 CTRV：状态 [x, y, v, heading, yawRate] ----------

type ctrvModel struct{}

func (ctrvModel) dim() int { return 5 }

func (m ctrvModel) estimate(obs []Observation, noise NoiseConfig, maxGapNanos int64) stateVector {
	last := obs[len(obs)-1]
	v := math.Hypot(last.VX, last.VY)
	heading := wrapAngle(last.Heading)
	yawRate := 0.0
	if len(obs) >= 2 && inSegment(obs[len(obs)-2], last, maxGapNanos) {
		prev := obs[len(obs)-2]
		dt := float64(last.Timestamp-prev.Timestamp) / 1e9
		// 用位置差分估计速率，用朝向差估计转弯率。
		dist := math.Hypot(last.X-prev.X, last.Y-prev.Y)
		v = dist / dt
		yawRate = wrapAngle(last.Heading-prev.Heading) / dt
	}
	P := matZeros(5)
	P[0*5+0] = noise.InitialPosVar
	P[1*5+1] = noise.InitialPosVar
	P[2*5+2] = noise.InitialVelVar
	P[3*5+3] = noise.InitialHeadingVar
	P[4*5+4] = noise.InitialYawRateVar
	return stateVector{x: []float64{last.X, last.Y, v, heading, yawRate}, P: P}
}

// ctrvStep 计算 CTRV 单步传播的均值与解析雅可比（5x5，行主序）。
func ctrvStep(x []float64, dt float64) (nx []float64, F []float64) {
	px, py, v, th, w := x[0], x[1], x[2], x[3], x[4]
	const eps = 1e-6

	var nx2, ny2 float64
	F = matIdent(5)
	if math.Abs(w) > eps {
		thNew := th + w*dt
		nx2 = px + v/w*(math.Sin(thNew)-math.Sin(th))
		ny2 = py + v/w*(-math.Cos(thNew)+math.Cos(th))
		// 解析雅可比。
		F[0*5+2] = (math.Sin(thNew) - math.Sin(th)) / w
		F[0*5+3] = v / w * (math.Cos(thNew) - math.Cos(th))
		F[0*5+4] = v * (dt*math.Cos(thNew)*w - (math.Sin(thNew) - math.Sin(th))) / (w * w)
		F[1*5+2] = (-math.Cos(thNew) + math.Cos(th)) / w
		F[1*5+3] = v / w * (math.Sin(thNew) - math.Sin(th))
		F[1*5+4] = v * (dt*math.Sin(thNew)*w - (-math.Cos(thNew) + math.Cos(th))) / (w * w)
	} else {
		// 小转弯率极限：取精确式关于 ω 的二阶展开，
		// 使均值与雅可比在两分支间光滑衔接。
		// x' ≈ x + v·cosθ·dt − v·sinθ·ω·dt²/2
		// y' ≈ y + v·sinθ·dt + v·cosθ·ω·dt²/2
		nx2 = px + v*math.Cos(th)*dt - v*math.Sin(th)*w*dt*dt/2
		ny2 = py + v*math.Sin(th)*dt + v*math.Cos(th)*w*dt*dt/2
		F[0*5+2] = math.Cos(th) * dt
		F[0*5+3] = -v * math.Sin(th) * dt
		F[0*5+4] = -v * math.Sin(th) * dt * dt / 2
		F[1*5+2] = math.Sin(th) * dt
		F[1*5+3] = v * math.Cos(th) * dt
		F[1*5+4] = v * math.Cos(th) * dt * dt / 2
	}
	nth := wrapAngle(th + w*dt)
	F[3*5+4] = dt
	return []float64{nx2, ny2, v, nth, w}, F
}

func (m ctrvModel) propagate(st stateVector, dt float64, noise NoiseConfig) stateVector {
	nxVec, F := ctrvStep(st.x, dt)

	// 过程噪声：位置/速率独立噪声 + 转弯率白噪声经积分进入朝向。
	Q := matZeros(5)
	Q[0*5+0] = noise.ProcessPos * dt
	Q[1*5+1] = noise.ProcessPos * dt
	Q[2*5+2] = noise.ProcessVel * dt
	qw := noise.ProcessYawRate
	Q[3*5+3] = qw * dt * dt * dt / 3
	Q[3*5+4] = qw * dt * dt / 2
	Q[4*5+3] = qw * dt * dt / 2
	Q[4*5+4] = qw * dt

	P := matMul(matMul(F, st.P, 5), matTranspose(F, 5), 5)
	matAddInPlace(P, Q)
	return stateVector{x: nxVec, P: P}
}

func (m ctrvModel) extract(st stateVector) (x, y, vx, vy, heading float64, unc Uncertainty) {
	x, y = st.x[0], st.x[1]
	v, heading := st.x[2], wrapAngle(st.x[3])
	vx = v * math.Cos(heading)
	vy = v * math.Sin(heading)
	unc.PosCov = [2][2]float64{
		{st.P[0*5+0], st.P[0*5+1]},
		{st.P[1*5+0], st.P[1*5+1]},
	}
	unc.PosStd = math.Sqrt(st.P[0*5+0] + st.P[1*5+1])
	unc.SpeedStd = math.Sqrt(st.P[2*5+2])
	unc.HeadingStd = math.Sqrt(st.P[3*5+3])
	return
}
