// Package trajectory 实现本地移动体轨迹预测服务。
//
// 输入为多个动态对象的观测序列（可能乱序、重复、缺失），
// 输出为每个对象未来一段时间内的轨迹预测及不确定度。
package trajectory

// ModelType 标识运动模型。
type ModelType string

const (
	// ModelCV 匀速模型（Constant Velocity）。
	ModelCV ModelType = "cv"
	// ModelCA 匀加速模型（Constant Acceleration）。
	ModelCA ModelType = "ca"
	// ModelCTRV 恒定转弯率与速度模型（Constant Turn Rate and Velocity）。
	ModelCTRV ModelType = "ctrv"
)

// Observation 是对单个动态对象的一次观测。
// Timestamp 为 Unix 纳秒时间戳。
type Observation struct {
	ObjectID  string  `json:"object_id"`
	Timestamp int64   `json:"timestamp"`
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	VX        float64 `json:"vx"`
	VY        float64 `json:"vy"`
	Heading   float64 `json:"heading"` // 弧度，x 轴正向为 0，逆时针为正
}

// Gap 表示历史观测中的一段缺失区间 [Start, End]（均为 Unix 纳秒）。
type Gap struct {
	Start int64 `json:"start"`
	End   int64 `json:"end"`
}

// Uncertainty 描述某一预测时刻的不确定度。
type Uncertainty struct {
	// PosCov 为位置 (x, y) 的 2x2 协方差。
	PosCov [2][2]float64 `json:"pos_cov"`
	// PosStd 为位置标准差，sqrt(trace(PosCov))。
	PosStd float64 `json:"pos_std"`
	// SpeedStd 为速度大小标准差。
	SpeedStd float64 `json:"speed_std"`
	// HeadingStd 为朝向标准差（弧度）。
	HeadingStd float64 `json:"heading_std"`
}

// PredictedState 是某一未来时刻的预测状态。
type PredictedState struct {
	Timestamp   int64       `json:"timestamp"`
	X           float64     `json:"x"`
	Y           float64     `json:"y"`
	VX          float64     `json:"vx"`
	VY          float64     `json:"vy"`
	Heading     float64     `json:"heading"`
	Uncertainty Uncertainty `json:"uncertainty"`
}

// Prediction 是单个对象的完整预测结果。
type Prediction struct {
	ObjectID string           `json:"object_id"`
	Model    ModelType        `json:"model"`
	States   []PredictedState `json:"states"`
	// Gaps 为预测所依据的历史中识别出的缺失区间。
	Gaps []Gap `json:"gaps,omitempty"`
}

// Conflict 表示两个对象的预测轨迹在未来某时刻进入设定距离以内。
type Conflict struct {
	ObjectA   string  `json:"object_a"`
	ObjectB   string  `json:"object_b"`
	Timestamp int64   `json:"timestamp"` // 首次进入阈值的未来时刻
	Distance  float64 `json:"distance"`  // 该时刻的预测距离
	Threshold float64 `json:"threshold"`
}
