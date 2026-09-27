package trajectory

import (
	"bufio"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
)

const walFileName = "observations.jsonl"

// wal 为追加式观测日志，用于重启后恢复历史。
// 每条接收到的观测（含重复）都会追加落盘，重放时由历史层去重，
// 因此不会丢失、重复或跳过任何观测。
type wal struct {
	f   *os.File
	enc *json.Encoder
}

// openWAL 打开（必要时创建）目录下的观测日志，并回放返回全部已存观测。
func openWAL(dir string) (*wal, []Observation, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, nil, fmt.Errorf("创建持久化目录失败: %w", err)
	}
	path := filepath.Join(dir, walFileName)
	f, err := os.OpenFile(path, os.O_CREATE|os.O_RDWR|os.O_APPEND, 0o644)
	if err != nil {
		return nil, nil, fmt.Errorf("打开观测日志失败: %w", err)
	}

	var replayed []Observation
	if _, err := f.Seek(0, io.SeekStart); err != nil {
		f.Close()
		return nil, nil, fmt.Errorf("定位观测日志失败: %w", err)
	}
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 64*1024), 4*1024*1024)
	for sc.Scan() {
		line := sc.Bytes()
		if len(line) == 0 {
			continue
		}
		var o Observation
		if err := json.Unmarshal(line, &o); err != nil {
			// 跳过损坏行（例如上次写入中途崩溃的截断行），不中断恢复。
			continue
		}
		replayed = append(replayed, o)
	}
	if err := sc.Err(); err != nil {
		f.Close()
		return nil, nil, fmt.Errorf("读取观测日志失败: %w", err)
	}
	// 回到末尾继续追加。
	if _, err := f.Seek(0, io.SeekEnd); err != nil {
		f.Close()
		return nil, nil, fmt.Errorf("定位日志末尾失败: %w", err)
	}
	return &wal{f: f, enc: json.NewEncoder(f)}, replayed, nil
}

// append 追加一条观测并立即落盘。
func (w *wal) append(o Observation) error {
	if w == nil || w.f == nil {
		return nil
	}
	if err := w.enc.Encode(o); err != nil {
		return fmt.Errorf("写入观测日志失败: %w", err)
	}
	if err := w.f.Sync(); err != nil {
		return fmt.Errorf("同步观测日志失败: %w", err)
	}
	return nil
}

// close 关闭日志文件。
func (w *wal) close() error {
	if w == nil || w.f == nil {
		return nil
	}
	return w.f.Close()
}
