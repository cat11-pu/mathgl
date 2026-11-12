// Copyright 2024 The go-gl/mathgl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl32

import (
	"math"
	"math/rand"
	"testing"
)

const (
	polarFloatTolerance = polarReconstructionTolerance
	polarRandomCount    = 1000
)

func TestPolarDecomposeRandomBatch(t *testing.T) {
	rng := rand.New(rand.NewSource(0x5eed))

	var successCount int
	var maxRelativeError, maxOrthogonalityError, maxMaxElementError float32
	var maxIterationScales Vec4

	for i := 0; i < polarRandomCount; i++ {
		m := randomInvertibleMat4(rng)

		decomposition, err := PolarDecompose(m)
		if err != nil {
			t.Fatalf("case %d: expected successful decomposition, got error %v", i, err)
		}
		if hasNonFiniteMat4(decomposition.Q) || hasNonFiniteMat4(decomposition.L) ||
			hasNonFiniteVec4(decomposition.Scale) {
			t.Fatalf("case %d: decomposition contains undefined values", i)
		}

		rebuilt := decomposition.Reconstruct()
		relativeError := relativeFrobeniusError(m, rebuilt)
		elementError := maxAbsElementDifference(m, rebuilt)
		orthogonalityError := orthogonalityError4(decomposition.Q)

		if relativeError > polarFloatTolerance {
			t.Fatalf("case %d: relative reconstruction error %g exceeds %g",
				i, relativeError, polarFloatTolerance)
		}
		if orthogonalityError > polarFloatTolerance {
			t.Fatalf("case %d: Q is not orthogonal, error %g", i, orthogonalityError)
		}

		successCount++
		maxRelativeError = maxFloat32(maxRelativeError, relativeError)
		maxMaxElementError = maxFloat32(maxMaxElementError, elementError)
		maxOrthogonalityError = maxFloat32(maxOrthogonalityError, orthogonalityError)
		for axis := 0; axis < 4; axis++ {
			maxIterationScales[axis] = maxFloat32(maxIterationScales[axis],
				abs32(decomposition.Scale[axis]))
		}
	}

	if successCount != polarRandomCount {
		t.Fatalf("only %d of %d matrices decomposed", successCount, polarRandomCount)
	}

	t.Logf("float32: %d/%d matrices decomposed successfully", successCount, polarRandomCount)
	t.Logf("float32: max relative Frobenius reconstruction error = %.3e", maxRelativeError)
	t.Logf("float32: max absolute element reconstruction error  = %.3e", maxMaxElementError)
	t.Logf("float32: max |Q^T Q - I| Frobenius error           = %.3e", maxOrthogonalityError)
	t.Logf("float32: largest recovered scale per axis          = %v", maxIterationScales)
}

func TestPolarDecomposeProperties(t *testing.T) {
	m := Mat4{
		2, 0.3, -0.7, 1.2,
		0, 1.5, 0.4, -0.9,
		0.6, 0, 2.2, 0.5,
		0.1, -1.1, 0.2, 1.8,
	}

	decomposition, err := PolarDecompose(m)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	// L must be unit lower triangular.
	for col := 0; col < 4; col++ {
		for row := 0; row < 4; row++ {
			switch {
			case row < col && decomposition.L[4*col+row] != 0:
				t.Fatalf("L has a nonzero upper triangular entry at (%d,%d)", row, col)
			case row == col && decomposition.L[4*col+row] != 1:
				t.Fatalf("L has diagonal entry %g instead of 1", decomposition.L[4*col+row])
			}
		}
	}

	// Every scale factor of an invertible matrix must be positive.
	for axis := 0; axis < 4; axis++ {
		if decomposition.Scale[axis] <= 0 {
			t.Fatalf("scale factor %d is not positive: %g", axis, decomposition.Scale[axis])
		}
	}

	if relativeFrobeniusError(m, decomposition.Reconstruct()) > polarFloatTolerance {
		t.Fatal("reconstruction does not match the original matrix")
	}
}

func TestPolarDecomposeIdentity(t *testing.T) {
	decomposition, err := PolarDecompose(Ident4())
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	if !decomposition.Q.ApproxEqualThreshold(Ident4(), 1e-6) {
		t.Fatalf("identity polar factor is %v", decomposition.Q)
	}
	expectedScale := Vec4{1, 1, 1, 1}
	for axis := 0; axis < 4; axis++ {
		if abs32(decomposition.Scale[axis]-expectedScale[axis]) > 1e-6 {
			t.Fatalf("identity scale is %v", decomposition.Scale)
		}
	}
}

func TestPolarDecomposeNegativeDeterminant(t *testing.T) {
	// A reflection on the first axis gives det = -1. It must be accepted and
	// reported through an orthogonal factor with determinant -1 rather than
	// treated as an undefined input.
	reflection := Mat4{
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}

	decomposition, err := PolarDecompose(reflection)
	if err != nil {
		t.Fatalf("reflection matrix returned an error: %v", err)
	}

	det := decomposition.Q.Det()
	if abs32(det+1) > 1e-6 {
		t.Fatalf("reflection polar factor has determinant %g, want -1", det)
	}
	if relativeFrobeniusError(reflection, decomposition.Reconstruct()) > polarFloatTolerance {
		t.Fatal("reflection matrix was not reconstructed")
	}
}

func TestPolarDecomposeDegenerateInputs(t *testing.T) {
	tests := []struct {
		name    string
		matrix  Mat4
		wantErr error
	}{
		{
			name:    "zero matrix",
			matrix:  Mat4{},
			wantErr: ErrZeroMatrix,
		},
		{
			name: "rank one matrix",
			matrix: Mat4{
				1, 2, 3, 4,
				2, 4, 6, 8,
				3, 6, 9, 12,
				4, 8, 12, 16,
			},
			wantErr: ErrSingularMatrix,
		},
		{
			name: "zero column projection",
			matrix: Mat4{
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 0,
			},
			wantErr: ErrSingularMatrix,
		},
		{
			name: "duplicate rows",
			matrix: Mat4{
				1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				1, 2, 3, 4,
			},
			wantErr: ErrSingularMatrix,
		},
		{
			name: "near singular matrix",
			matrix: Mat4{
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1e-20,
			},
			wantErr: ErrSingularMatrix,
		},
	}

	for _, test := range tests {
		decomposition, err := PolarDecompose(test.matrix)
		if err != test.wantErr {
			t.Errorf("%s: got error %v, want %v", test.name, err, test.wantErr)
		}
		if err == nil {
			continue
		}
		if hasNonFiniteMat4(decomposition.Q) || hasNonFiniteMat4(decomposition.L) ||
			hasNonFiniteVec4(decomposition.Scale) {
			t.Errorf("%s: error path produced NaN or Inf", test.name)
		}
	}
}

func TestPolarDecomposeRejectsNonFinitePivot(t *testing.T) {
	notANumber := float32(math.NaN())
	m := Ident4()
	m[0] = notANumber

	if _, err := PolarDecompose(m); err == nil {
		t.Fatal("matrix containing NaN must not decompose successfully")
	}
}

func TestPolarDecomposeNoHeapAllocation(t *testing.T) {
	m := Mat4{
		2, 0.3, -0.7, 1.2,
		0, 1.5, 0.4, -0.9,
		0.6, 0, 2.2, 0.5,
		0.1, -1.1, 0.2, 1.8,
	}

	allocs := testing.AllocsPerRun(100, func() {
		if _, err := PolarDecompose(m); err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
	})
	if allocs != 0 {
		t.Fatalf("PolarDecompose performed %v heap allocations per run, want 0", allocs)
	}
}

// randomInvertibleMat4 builds a guaranteed invertible matrix as
// R1 * L * D * R2, optionally composed with a reflection. The rotation,
// positive diagonal scale and unit lower triangular shear are exactly the
// structures the decomposition is meant to recover.
func randomInvertibleMat4(rng *rand.Rand) Mat4 {
	rotationA := randomRotationMat4(rng)
	rotationB := randomRotationMat4(rng)

	shear := Ident4()
	for col := 0; col < 4; col++ {
		for row := col + 1; row < 4; row++ {
			shear[4*col+row] = rng.Float32()*2 - 1
		}
	}

	scale := Ident4()
	for axis := 0; axis < 4; axis++ {
		// Keep scales bounded away from zero so the matrix stays well
		// conditioned, while spanning shrinkage and enlargement.
		scale[4*axis+axis] = 0.25 + rng.Float32()*3.75
	}

	m := rotationA.Mul4(shear).Mul4(scale).Mul4(rotationB)

	if rng.Intn(5) == 0 {
		reflection := Ident4()
		reflection[0] = -1
		m = m.Mul4(reflection)
	}
	return m
}

func randomRotationMat4(rng *rand.Rand) Mat4 {
	angle := rng.Float32() * 2 * float32(math.Pi)
	axis := Vec3{
		rng.Float32()*2 - 1,
		rng.Float32()*2 - 1,
		rng.Float32()*2 - 1,
	}
	length := float32(math.Sqrt(float64(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])))
	if length == 0 {
		axis[2] = 1
		length = 1
	}
	axis = axis.Mul(1 / length)

	rotation3 := HomogRotate3D(angle, axis)
	return rotation3
}

func relativeFrobeniusError(original, rebuilt Mat4) float32 {
	var differenceSquared, originalSquared float32
	for i := range original {
		diff := rebuilt[i] - original[i]
		differenceSquared += diff * diff
		originalSquared += original[i] * original[i]
	}
	if originalSquared == 0 {
		return float32(math.Sqrt(float64(differenceSquared)))
	}
	return float32(math.Sqrt(float64(differenceSquared / originalSquared)))
}

func maxAbsElementDifference(a, b Mat4) float32 {
	var result float32
	for i := range a {
		result = maxFloat32(result, abs32(a[i]-b[i]))
	}
	return result
}

func orthogonalityError4(q Mat4) float32 {
	product := q.Transpose().Mul4(q)
	return relativeFrobeniusError(Ident4(), product)
}

func hasNonFiniteMat4(m Mat4) bool {
	for i := range m {
		if math.IsNaN(float64(m[i])) || math.IsInf(float64(m[i]), 0) {
			return true
		}
	}
	return false
}

func hasNonFiniteVec4(v Vec4) bool {
	for i := range v {
		if math.IsNaN(float64(v[i])) || math.IsInf(float64(v[i]), 0) {
			return true
		}
	}
	return false
}

func maxFloat32(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
