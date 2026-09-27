# trajectory-prediction-go

本地移动体轨迹预测服务。输入为多个动态对象的观测序列（对象标识、时间戳、
位置、速度、朝向），通过内存管道 / 本地回环喂入，不依赖真实传感器或外部
服务。服务基于历史观测预测每个对象未来一段时间内的轨迹，并输出预测的
不确定度。

## 构建与测试

```sh
go build ./...
go test ./... -race        # 全部测试（含并发竞态检测）
go run ./cmd/trajpred      # 本地回环演示：内存管道喂观测 → 预测 → 冲突检测
```

## 观测摄入与历史维护

- 每个对象的历史按时间戳升序维护；**乱序**到达的观测会被正确归位。
- **重复**观测（对象、时间戳、位置、速度、朝向完全相同）被去重丢弃；
  同一时间戳但内容不同的样本视为不同观测予以保留。
- **缺失**时间段通过 `Gaps(objectID)` 显式识别：相邻观测间隔超过
  1.5×`sample_period` 即记为一个缺口。缺失不会被填充为静止或零值。
- 写入与查询可并发进行。每次查询在读锁下复制一份一致的历史快照后再
  计算，不会观察到半写入的观测，也不会因并发打乱历史顺序。

## 运动模型

三种模型，可按对象（`per_object`）或全局（`default_model`）选择；
单次查询显式指定模型时优先级最高（请求 > 按对象 > 全局默认）。

| 模型 | 状态向量 | 假设 |
|------|----------|------|
| `cv`（匀速） | `[x, y, vx, vy]` | 速度恒定，加速度为零均值白噪声 |
| `ca`（匀加速） | `[x, y, vx, vy, ax, ay]` | 加速度恒定，加加速度（jerk）为零均值白噪声；初始加速度由最近两次观测估计 |
| `ctrv`（恒定转弯率） | `[x, y, v, heading, yawRate]` | 速率与转弯率恒定；初始转弯率由最近两次观测的朝向差估计 |

预测输出未来多个离散时刻（`t0+step … t0+horizon`，`t0` 为最新观测时刻）
的位置、速度、朝向，以及位置 `(x, y)` 的 2×2 协方差。

## 不确定度语义

- 每个预测状态携带位置协方差 `cov_xx, cov_xy, cov_yy`（单位 m²），
  表示该时刻预测位置的二阶不确定度；协方差越大，预测越不可信。
- 协方差按 `P ← F·P·Fᵀ + Q` 逐步传播，其中 `F` 为模型雅可比、`Q` 为
  过程噪声。输出保证**随预测时刻单调不减**（对位置协方差迹做了单调化）。
- 初始状态协方差与过程噪声均可配置（见下），不同模型与噪声参数会产生
  可区分的不确定度增长曲线（有测试保证）。

## 冲突检测

`Conflicts(horizon, step, threshold)` 在统一时间网格上预测所有已知对象，
当任意两个对象的预测位置在同一未来时刻的距离进入 `threshold`（米）以内时，
返回一条冲突记录：相关对象 ID、最早发生冲突的时刻、该时刻的距离。
结果按时间、对象 ID 排序，输出确定。

## 确定性与重启恢复

- 预测为纯确定性计算：固定输入序列（与随机种子无关，服务不使用随机数）
  必然产生逐条一致的结果，可复现历史结论。
- 配置 `store_path` 后，每条观测在摄入时追加写入 JSON Lines 日志并
  fsync。重启时按写入顺序重放，重放走与实时摄入相同的插入逻辑，
  因此重复观测被去重、乱序被归位，历史不丢失、不重复、不跳过。

## 配置项

```go
type Config struct {
    DefaultModel ModelKind            // 全局默认模型：cv / ca / ctrv
    PerObject    map[string]ModelKind // 按对象的模型覆盖
    Noise        NoiseConfig          // 不确定度参数（见下）
    SamplePeriod time.Duration        // 期望采样周期，用于缺口识别
    StorePath    string               // 持久化日志路径；空串禁用持久化
}

type NoiseConfig struct {
    InitialPosVar float64 // 初始位置方差 (m²)
    InitialVelVar float64 // 初始速度方差 (m²/s²)
    InitialAccVar float64 // 初始加速度方差（CA）
    ProcessPosVar float64 // CTRV 位置过程噪声密度
    ProcessVelVar float64 // CV/CA 加速度谱密度；CTRV 速率噪声
    ProcessAccVar float64 // CA 加加速度谱密度
    ProcessYawVar float64 // CTRV 航向 / 转弯率噪声密度
}
```

## 代码结构

```
cmd/trajpred/            本地回环演示程序
internal/trajectory/
  types.go               观测、预测状态、轨迹、缺口、冲突等类型
  history.go             按时间有序、去重、缺口识别的单对象历史
  model.go               CV / CA / CTRV 运动模型与噪声配置
  matrix.go              协方差传播所需的小矩阵运算
  predict.go             轨迹预测与不确定度传播（单调化）
  conflict.go            预测轨迹间冲突检测
  store.go               JSON Lines 追加式持久化与恢复
  service.go             并发安全的服务门面（读写锁 + 快照）
```
