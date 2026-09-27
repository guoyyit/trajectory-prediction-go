package trajectory

// NoiseConfig 汇总初始状态协方差与过程噪声参数（均为方差/谱密度，非标准差）。
type NoiseConfig struct {
	// 初始状态方差
	InitialPosVar     float64 `json:"initial_pos_var"`
	InitialVelVar     float64 `json:"initial_vel_var"`
	InitialAccelVar   float64 `json:"initial_accel_var"`
	InitialHeadingVar float64 `json:"initial_heading_var"`
	InitialYawRateVar float64 `json:"initial_yaw_rate_var"`

	// 过程噪声谱密度（单位时间）
	ProcessPos     float64 `json:"process_pos"`
	ProcessVel     float64 `json:"process_vel"`
	ProcessAccel   float64 `json:"process_accel"`
	ProcessYawRate float64 `json:"process_yaw_rate"`
}

// Config 为服务配置。
type Config struct {
	// DefaultModel 为全局默认运动模型。
	DefaultModel ModelType `json:"default_model"`
	// ObjectModels 允许按对象覆盖运动模型。
	ObjectModels map[string]ModelType `json:"object_models,omitempty"`

	// Steps 为未来预测的离散时刻数。
	Steps int `json:"steps"`
	// StepNanos 为相邻预测时刻的间隔（纳秒）。
	StepNanos int64 `json:"step_nanos"`

	// MaxGapNanos 为判定观测缺失的时间间隔阈值。
	MaxGapNanos int64 `json:"max_gap_nanos"`

	// ConflictDistance 为冲突检测距离阈值（与位置同单位）。
	ConflictDistance float64 `json:"conflict_distance"`

	// Noise 为不确定度传播参数。
	Noise NoiseConfig `json:"noise"`

	// DataDir 为持久化目录；为空表示仅内存，不持久化。
	DataDir string `json:"data_dir,omitempty"`

	// Seed 为随机种子，保证相同输入下结果可复现。
	Seed int64 `json:"seed"`
}

// DefaultConfig 返回一份可用的默认配置。
func DefaultConfig() Config {
	return Config{
		DefaultModel:     ModelCV,
		Steps:            10,
		StepNanos:        1_000_000_000, // 1s
		MaxGapNanos:      5_000_000_000, // 5s
		ConflictDistance: 2.0,
		Noise: NoiseConfig{
			InitialPosVar:     1.0,
			InitialVelVar:     1.0,
			InitialAccelVar:   1.0,
			InitialHeadingVar: 0.1,
			InitialYawRateVar: 0.01,
			ProcessPos:        0.1,
			ProcessVel:        0.1,
			ProcessAccel:      0.1,
			ProcessYawRate:    0.01,
		},
		Seed: 42,
	}
}

// modelFor 返回指定对象应使用的运动模型。
func (c Config) modelFor(objectID string) ModelType {
	if c.ObjectModels != nil {
		if m, ok := c.ObjectModels[objectID]; ok {
			return m
		}
	}
	return c.DefaultModel
}
