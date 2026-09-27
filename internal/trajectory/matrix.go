package trajectory

// matrix is a small dense row-major matrix for covariance propagation.
// Only the operations needed by the predictors are implemented.
type matrix struct {
	rows, cols int
	data       []float64
}

func newMatrix(rows, cols int) matrix {
	return matrix{rows: rows, cols: cols, data: make([]float64, rows*cols)}
}

func identity(n int) matrix {
	m := newMatrix(n, n)
	for i := 0; i < n; i++ {
		m.set(i, i, 1)
	}
	return m
}

func diag(v []float64) matrix {
	m := newMatrix(len(v), len(v))
	for i, x := range v {
		m.set(i, i, x)
	}
	return m
}

func (m matrix) at(i, j int) float64     { return m.data[i*m.cols+j] }
func (m matrix) set(i, j int, v float64) { m.data[i*m.cols+j] = v }

// mul returns a*b. Panics on dimension mismatch (programming error).
func mul(a, b matrix) matrix {
	if a.cols != b.rows {
		panic("matrix mul: dimension mismatch")
	}
	out := newMatrix(a.rows, b.cols)
	for i := 0; i < a.rows; i++ {
		for k := 0; k < a.cols; k++ {
			av := a.at(i, k)
			if av == 0 {
				continue
			}
			for j := 0; j < b.cols; j++ {
				out.data[i*out.cols+j] += av * b.at(k, j)
			}
		}
	}
	return out
}

// transpose returns mᵀ.
func transpose(m matrix) matrix {
	out := newMatrix(m.cols, m.rows)
	for i := 0; i < m.rows; i++ {
		for j := 0; j < m.cols; j++ {
			out.set(j, i, m.at(i, j))
		}
	}
	return out
}

// add returns a+b.
func add(a, b matrix) matrix {
	if a.rows != b.rows || a.cols != b.cols {
		panic("matrix add: dimension mismatch")
	}
	out := newMatrix(a.rows, a.cols)
	for i := range out.data {
		out.data[i] = a.data[i] + b.data[i]
	}
	return out
}

// matFromFlat builds an n×n matrix from row-major flat data.
func matFromFlat(n int, flat []float64) matrix {
	m := newMatrix(n, n)
	copy(m.data, flat)
	return m
}
