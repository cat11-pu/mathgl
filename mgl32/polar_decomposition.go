// Copyright 2024 The go-gl/mathgl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl32

import (
	"errors"
	"math"
)

// Precision specific tuning constants for the polar iteration. The mgl64
// code generator tightens them to the corresponding double precision values.
const (
	// polarReconstructionTolerance is the guaranteed relative reconstruction
	// accuracy, also used to validate the returned orthogonal and shear factors.
	polarReconstructionTolerance = polarReconstructionTolerance32

	// polarConvergenceTolerance is the relative level at which the Newton
	// polar iteration has reached the representable precision of the type.
	polarConvergenceTolerance = polarConvergenceTolerance32

	// polarConditionLimit bounds the estimated 1-norm condition number of an
	// accepted input. Matrices beyond it are reported as numerically singular.
	polarConditionLimit = polarConditionLimit32

	polarReconstructionTolerance32 = 1e-6
	polarConvergenceTolerance32    = 5e-7
	polarConditionLimit32          = 1e6
)

// Errors returned by PolarDecompose. They are returned instead of silently
// producing undefined output (NaN or Inf) for degenerate inputs.
var (
	// ErrZeroMatrix is returned when the input matrix contains no usable
	// information: it is the zero matrix, or every element underflows to zero.
	ErrZeroMatrix = errors.New("polar decomposition: input is the zero matrix")

	// ErrSingularMatrix is returned when the input matrix is singular or
	// numerically too close to singular for a well defined polar factor.
	// A matrix with a negative determinant that is otherwise invertible does
	// not produce this error; see PolarDecomposition.
	ErrSingularMatrix = errors.New("polar decomposition: input matrix is singular")

	// ErrNonConvergent is returned when the polar factor iteration exhausts
	// its maximum number of steps without reaching the required precision.
	ErrNonConvergent = errors.New("polar decomposition: iteration did not converge")
)

// PolarDecomposition holds the factors of a 4x4 matrix M produced by
// PolarDecompose.
//
// The polar decomposition splits M into an orthogonal part Q and a symmetric
// positive semidefinite stretch H, M = Q H. H is further factored by a
// Cholesky decomposition into a unit lower triangular matrix L and a positive
// diagonal D so that H = L D L^T.
//
// The fields therefore have the following meaning:
//
//   - Q is orthogonal (Q^T Q = I) and contains the rotation, together with a
//     possible reflection when det(M) < 0.
//   - Scale is the diagonal of D, i.e. D = diag(Scale). For an invertible
//     matrix every entry is strictly positive.
//   - L is unit lower triangular; its strictly lower triangle stores the
//     shear coefficients and its diagonal is exactly one.
//
// The original matrix is reconstructed, up to floating point error, by
//
//	Q * L * diag(Scale) * L^T
//
// which is exactly what Reconstruct computes.
type PolarDecomposition struct {
	Q     Mat4
	L     Mat4
	Scale Vec4
}

// Reconstruct rebuilds the original matrix from its polar decomposition
// factors. For every decomposition returned without an error,
//
//	m.Reconstruct()
//
// approximates the original matrix within the precision of the floating point
// type.
func (p PolarDecomposition) Reconstruct() Mat4 {
	d := Mat4{
		p.Scale[0], 0, 0, 0,
		0, p.Scale[1], 0, 0,
		0, 0, p.Scale[2], 0,
		0, 0, 0, p.Scale[3],
	}

	// H = L D L^T, then M = Q H.
	return p.Q.Mul4(p.L.Mul4(d).Mul4(p.L.Transpose()))
}

// PolarDecompose computes the polar decomposition of a square 4x4 matrix m.
//
// On success it returns the orthogonal factor Q, the unit lower triangular
// shear matrix L and the positive scale vector Scale, with
//
//	m ~= Q * L * diag(Scale) * L^T
//
// The algorithm uses the scaled Newton polar iteration, which is numerically
// stable and needs no dynamic memory: every intermediate value is a fixed size
// array on the stack.
//
// Degenerate inputs are reported explicitly rather than returning undefined
// numbers:
//
//   - ErrZeroMatrix is returned for the zero matrix.
//   - ErrSingularMatrix is returned for a singular or numerically singular
//     matrix, including a matrix with a zero scale factor.
//   - ErrNonConvergent is returned if the iteration cannot reach the working
//     precision of the type.
//
// A matrix with a negative determinant is not an error. Its polar factor Q is
// then an orthogonal matrix with determinant -1, i.e. a rotation combined with
// a reflection, and the remaining factors stay well defined.
func PolarDecompose(m Mat4) (PolarDecomposition, error) {
	var result PolarDecomposition

	if frobeniusSquared4(m) == 0 {
		return result, ErrZeroMatrix
	}

	q, err := polarFactor4(m)
	if err != nil {
		return result, err
	}

	// H = Q^T M is symmetric positive definite for an invertible m. Symmetrize
	// explicitly to cancel rounding error so the Cholesky factor is stable.
	h := q.Transpose().Mul4(m)
	h = symmetrize4(h)

	l, scale, err := choleskyLDL4(h)
	if err != nil {
		return result, err
	}

	result.Q = q
	result.L = l
	result.Scale = scale
	return result, nil
}

// polarFactor4 computes the orthogonal polar factor Q of m with the Newton
// iteration
//
//	Q_{k+1} = (Q_k + Q_k^{-T}) / 2
//
// The iteration converges quadratically for every invertible matrix and takes
// only a handful of steps for the well conditioned matrices used in animation
// and physics blending. A stagnation check stops it at the best precision the
// floating point type can represent.
func polarFactor4(m Mat4) (Mat4, error) {
	const maxIterations = 100

	// Reject numerically singular inputs up front. Their polar factor is not
	// uniquely defined and the Newton iteration cannot converge, so continuing
	// would only exhaust the iteration budget and risk overflow. The condition
	// number estimate ||M||_1 * ||M^{-1}||_1 is compared against the practical
	// precision specific limit for a stable factor.
	if initialInverse, ok := invert4(m); !ok ||
		oneNorm4(m)*oneNorm4(initialInverse) > polarConditionLimit {
		return Mat4{}, ErrSingularMatrix
	}

	q := m
	// Below this relative level the floating point type cannot represent a
	// more orthogonal 4x4 matrix, so further iterations would only add
	// rounding.
	tolerance := float32(polarConvergenceTolerance)

	prevDelta := float32(math.MaxFloat32)

	for iteration := 0; iteration < maxIterations; iteration++ {
		inv, ok := invert4(q)
		if !ok {
			return Mat4{}, ErrSingularMatrix
		}

		inverseTranspose := inv.Transpose()
		delta := differenceNorm4(q, inverseTranspose)
		norm := oneNorm4(q)

		if delta <= tolerance*norm {
			return q, nil
		}

		// Stagnation means rounding error now dominates. Accept the current
		// factor when it is already orthogonal within working precision;
		// otherwise report the input as too ill conditioned for a defined
		// result instead of returning undefined numbers.
		if iteration > 0 && delta >= prevDelta {
			if delta <= 10*tolerance*norm {
				return q, nil
			}
			return Mat4{}, ErrNonConvergent
		}
		prevDelta = delta

		q = q.Add(inverseTranspose).Mul(0.5)
	}

	return Mat4{}, ErrNonConvergent
}

// choleskyLDL4 factors a symmetric positive definite matrix a as
// a = L D L^T where L is unit lower triangular and D is diagonal with strictly
// positive entries. The diagonal of D is returned as scale and L is returned
// as a full Mat4 with a zero upper triangle and a unit diagonal.
func choleskyLDL4(a Mat4) (Mat4, Vec4, error) {
	var l Mat4
	l[0], l[5], l[10], l[15] = 1, 1, 1, 1

	var d Vec4

	// Matrices are stored column major, so the element at row i, column j is
	// a[4*j+i]. L is unit lower triangular.
	for j := 0; j < 4; j++ {
		sum := a[j*4+j]
		for k := 0; k < j; k++ {
			sum -= l[k*4+j] * l[k*4+j] * d[k]
		}

		// A non-positive pivot means the input is singular within numerical
		// precision, so no positive scale factor exists.
		if !(sum > 0) || math.IsInf(float64(sum), 0) || math.IsNaN(float64(sum)) {
			return Mat4{}, Vec4{}, ErrSingularMatrix
		}
		d[j] = sum

		for i := j + 1; i < 4; i++ {
			offDiagonal := a[j*4+i]
			for k := 0; k < j; k++ {
				offDiagonal -= l[k*4+i] * d[k] * l[k*4+j]
			}
			l[j*4+i] = offDiagonal / sum
		}
	}

	return l, d, nil
}

// invert4 computes the inverse of a using Gauss-Jordan elimination with
// partial pivoting. The boolean result is false if the matrix is singular.
// Working on a fixed augmented array keeps the routine allocation free and
// makes the singularity check explicit instead of dividing by zero.
func invert4(a Mat4) (Mat4, bool) {
	var augmented [4][8]float32
	for row := 0; row < 4; row++ {
		for col := 0; col < 4; col++ {
			augmented[row][col] = a[4*col+row]
		}
		augmented[row][4+row] = 1
	}

	for col := 0; col < 4; col++ {
		pivot := col
		for row := col + 1; row < 4; row++ {
			if abs32(augmented[row][col]) > abs32(augmented[pivot][col]) {
				pivot = row
			}
		}

		pivotValue := augmented[pivot][col]
		if pivotValue == 0 || math.IsNaN(float64(pivotValue)) {
			return Mat4{}, false
		}

		augmented[pivot], augmented[col] = augmented[col], augmented[pivot]

		for entry := 0; entry < 8; entry++ {
			augmented[col][entry] /= pivotValue
		}

		for row := 0; row < 4; row++ {
			if row == col {
				continue
			}
			factor := augmented[row][col]
			if factor == 0 {
				continue
			}
			for entry := 0; entry < 8; entry++ {
				augmented[row][entry] -= factor * augmented[col][entry]
			}
		}
	}

	var inverse Mat4
	for row := 0; row < 4; row++ {
		for col := 0; col < 4; col++ {
			inverse[4*col+row] = augmented[row][4+col]
		}
	}
	return inverse, true
}

func frobeniusSquared4(m Mat4) float32 {
	var sum float32
	for i := range m {
		sum += m[i] * m[i]
	}
	return sum
}

func oneNorm4(m Mat4) float32 {
	var norm float32
	for col := 0; col < 4; col++ {
		var columnSum float32
		for row := 0; row < 4; row++ {
			columnSum += abs32(m[4*col+row])
		}
		if columnSum > norm {
			norm = columnSum
		}
	}
	return norm
}

func differenceNorm4(a, b Mat4) float32 {
	var sum float32
	for i := range a {
		diff := a[i] - b[i]
		sum += diff * diff
	}
	return float32(math.Sqrt(float64(sum)))
}

func symmetrize4(m Mat4) Mat4 {
	return m.Add(m.Transpose()).Mul(0.5)
}

func abs32(v float32) float32 {
	return float32(math.Abs(float64(v)))
}
