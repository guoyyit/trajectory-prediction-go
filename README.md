# trajectory-prediction-go

本地移动体轨迹预测服务。输入为多个动态对象在一段时间内的观测序列（对象标识、时间戳、位置、速度、朝向），输出为每个对象未来一段时间内的轨迹预测与不确定度，并支持预测轨迹间的未来冲突检测。

观测通过内存管道（channel）或本地回环方式写入，不依赖真实传感器或外部服务。

## 功能概览

- **历史维护**：每个对象的观测按时间戳有序维护；乱序到达自动归位，重复观测（同对象同时间戳）自动去重，缺失时间段被显式识别为 `Gap`，绝不以静止或零值填充。
- **多运动模型**：匀速（CV）、匀加速（CA）、恒定转弯率（CTRV），支持全局默认与按对象覆盖。
- **不确定度传播**：基于线性/线性化协方差传播，预测时刻越远不确定度单调不减；初始协方差与过程噪声均可配置。
- **并发安全**：观测写入与预测查询可并发进行；查询基于一致的观测快照，不会混入半写入数据。
- **冲突检测**：任意两对象预测轨迹在未来某时刻距离进入阈值以内时，给出含时刻与对象对的可区分提示。
- **可复现**：相同输入顺序下预测逐条一致；观测以追加日志持久化，重启后完整恢复，不丢失、不重复、不跳过。

## 快速开始

```go
cfg := trajectory.DefaultConfig()
cfg.Steps = 10               // 未来 10 个预测时刻
cfg.StepNanos = 1_000_000_000 // 间隔 1s

svc, _ := trajectory.NewService(cfg)
defer svc.Close()

// 方式一：同步写入
svc.Ingest(trajectory.Observation{
    ObjectID: "car-1", Timestamp: 1_000_000_000,
    X: 0, Y: 0, VX: 1, VY: 0, Heading: 0,
})

// 方式二：内存管道异步写入
ch := svc.Pipeline(64)
ch <- trajectory.Observation{ObjectID: "car-1", Timestamp: 2_000_000_000, X: 1, VX: 1}
close(ch)
svc.WaitPipelines()

pred, _ := svc.Predict("car-1")   // 单对象预测
all := svc.PredictAll()           // 全部对象（按 ID 排序，确定性输出）
conflicts := svc.DetectConflicts() // 未来冲突检测
```

运行演示：`go run ./cmd/trajectory-demo`

## 运动模型

| 模型 | 状态向量 | 传播 | 历史估计 |
|------|----------|------|----------|
| `cv` 匀速 | `[x, y, vx, vy]` | 线性，白噪声加速度过程噪声 | 最新观测位置；最后两条同段观测差分修正速度，否则用上报速度 |
| `ca` 匀加速 | `[x, y, vx, vy, ax, ay]` | 线性，白噪声加加速度（jerk）过程噪声 | 同段三点非均匀差分估计加速度；两点退化为差分速度、零加速度 |
| `ctrv` 恒定转弯率 | `[x, y, v, θ, ω]` | 非线性传播 + 解析雅可比的线性化协方差传播；`ω→0` 时取直线极限 | 同段两点差分估计速率与转弯率（朝向差做角度回绕） |

**缺失段语义**：只有相邻观测间隔 `≤ MaxGapNanos` 时才被视为同一连续段并用于差分估计；跨缺失段不做差分（避免把缺失当作静止或零值），退化为使用最新观测上报的速度/朝向。所有缺失区间在 `Prediction.Gaps` 中显式返回。

## 不确定度语义

每个预测时刻输出 `Uncertainty`：

- `PosCov`：位置 (x, y) 的 2×2 协方差；
- `PosStd`：位置标准差，`sqrt(trace(PosCov))`，随预测步数单调不减；
- `SpeedStd`：速度大小标准差（CV/CA 由速度协方差一阶近似，CTRV 直接取状态方差）；
- `HeadingStd`：朝向标准差（弧度）；速度接近零时朝向不可知，取最大值 π。

传播规则为 `P' = F·P·Fᵀ + Q(dt)`：CV/CA 使用精确的运动学离散噪声矩阵，CTRV 使用解析雅可比与转弯率白噪声积分块。初始协方差（`Initial*Var`）与过程噪声谱密度（`Process*`）全部可配置，不同模型与噪声参数会产生可区分的不确定度曲线。

## 并发与一致性

- 写入（`Ingest`）持写锁：先追加持久化日志再更新内存历史，日志顺序与内存顺序一致。
- 查询（`Predict`/`PredictAll`/`DetectConflicts`）持读锁复制观测快照后立即释放，预测计算在快照上进行——结果来自某个一致快照，不会混入半写入的观测，也不会因并发导致历史顺序错乱。

## 冲突检测

`DetectConflicts` 对全部对象的预测轨迹按共同预测时刻两两比较，当距离首次小于 `ConflictDistance` 时返回 `Conflict{ObjectA, ObjectB, Timestamp, Distance, Threshold}`。输出按 `(时刻, 对象A, 对象B)` 排序，保证确定性。

## 可复现性与持久化

- 预测计算为纯确定性数学（无随机源）；`Seed` 配置保留用于未来扩展。相同输入顺序下预测结果逐条一致（测试中以 JSON 序列化逐字节比对）。
- 配置 `DataDir` 后，每条接收到的观测（含重复）以 JSONL 追加写入 `observations.jsonl` 并 fsync；重启时按原顺序重放，由历史层完成排序与去重，不丢失、不重复、不跳过任何观测。损坏的截断行（如崩溃时写了一半）会被跳过，不影响恢复。

## 配置项

| 字段 | 默认 | 说明 |
|------|------|------|
| `DefaultModel` | `cv` | 全局默认运动模型：`cv` / `ca` / `ctrv` |
| `ObjectModels` | - | 按对象覆盖运动模型 |
| `Steps` | 10 | 未来预测的离散时刻数 |
| `StepNanos` | 1s | 相邻预测时刻间隔（纳秒） |
| `MaxGapNanos` | 5s | 判定观测缺失的时间间隔阈值 |
| `ConflictDistance` | 2.0 | 冲突检测距离阈值 |
| `Noise.InitialPosVar` 等 | 见 `DefaultConfig` | 初始状态方差（位置/速度/加速度/朝向/转弯率） |
| `Noise.ProcessPos` 等 | 见 `DefaultConfig` | 过程噪声谱密度（位置/速度/加速度/转弯率） |
| `DataDir` | 空（仅内存） | 持久化目录 |
| `Seed` | 42 | 随机种子（当前实现完全确定，保留扩展） |

零值字段会被 `DefaultConfig` 的对应值替换。

## 测试

```bash
go test ./... -race
```

覆盖：乱序归位、重复去重、缺失识别（且不按静止处理）、三种模型的预测正确性、不确定度单调性与噪声可区分性、并发写入与查询（`-race`）、冲突检测、确定性复现、内存管道写入、重启恢复。

## 代码结构

```
types.go        核心类型（观测、预测、不确定度、冲突、缺失区间）
config.go       配置与默认值
history.go      单对象有序历史：归位、去重、缺失识别
model.go        CV/CA/CTRV 运动模型：状态估计、传播、协方差提取
predict.go      单对象预测生成
conflict.go     预测轨迹间冲突检测
service.go      并发安全的服务门面与内存管道
persistence.go  追加式观测日志（重启恢复）
cmd/trajectory-demo/  本地回环演示
```
