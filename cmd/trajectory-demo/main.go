// trajectory-demo 演示轨迹预测服务的本地回环使用方式：
// 通过内存管道写入观测，查询预测并检测冲突。
package main

import (
	"encoding/json"
	"fmt"
	"log"

	trajectory "github.com/guoyyit/trajectory-prediction-go"
)

func main() {
	cfg := trajectory.DefaultConfig()
	cfg.Steps = 5
	cfg.StepNanos = 1_000_000_000 // 1s
	cfg.ConflictDistance = 3.0

	svc, err := trajectory.NewService(cfg)
	if err != nil {
		log.Fatal(err)
	}
	defer svc.Close()

	// 内存管道模拟观测输入（可乱序、可重复）。
	ch := svc.Pipeline(64)
	obs := []trajectory.Observation{
		{ObjectID: "car-1", Timestamp: 2_000_000_000, X: 2, Y: 0, VX: 1, VY: 0, Heading: 0},
		{ObjectID: "car-1", Timestamp: 1_000_000_000, X: 1, Y: 0, VX: 1, VY: 0, Heading: 0}, // 乱序
		{ObjectID: "car-1", Timestamp: 2_000_000_000, X: 2, Y: 0, VX: 1, VY: 0, Heading: 0}, // 重复
		{ObjectID: "car-1", Timestamp: 3_000_000_000, X: 3, Y: 0, VX: 1, VY: 0, Heading: 0},
		{ObjectID: "car-2", Timestamp: 1_000_000_000, X: 10, Y: 0, VX: -1, VY: 0, Heading: 3.14159},
		{ObjectID: "car-2", Timestamp: 2_000_000_000, X: 9, Y: 0, VX: -1, VY: 0, Heading: 3.14159},
		{ObjectID: "car-2", Timestamp: 3_000_000_000, X: 8, Y: 0, VX: -1, VY: 0, Heading: 3.14159},
	}
	for _, o := range obs {
		ch <- o
	}
	close(ch)
	svc.WaitPipelines()

	for _, p := range svc.PredictAll() {
		out, _ := json.MarshalIndent(p, "", "  ")
		fmt.Printf("预测: %s\n", out)
	}

	for _, c := range svc.DetectConflicts() {
		fmt.Printf("冲突: 对象 %s 与 %s 将在 t=%d 进入 %.1f 距离以内 (距离 %.2f)\n",
			c.ObjectA, c.ObjectB, c.Timestamp, c.Threshold, c.Distance)
	}
}
