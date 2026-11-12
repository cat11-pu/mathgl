// Copyright 2024 The go-gl/mathgl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl32

import (
	"math/rand"
	"testing"
)

func randMat4(r *rand.Rand) Mat4 {
	var m Mat4
	for i := range m {
		m[i] = r.Float32()*2 - 1
	}
	return m
}

func randMat3(r *rand.Rand) Mat3 {
	var m Mat3
	for i := range m {
		m[i] = r.Float32()*2 - 1
	}
	return m
}

func randVec3s(r *rand.Rand, n int) []Vec3 {
	v := make([]Vec3, n)
	for i := range v {
		v[i] = Vec3{r.Float32()*20 - 10, r.Float32()*20 - 10, r.Float32()*20 - 10}
	}
	return v
}

func randVec4s(r *rand.Rand, n int) []Vec4 {
	v := make([]Vec4, n)
	for i := range v {
		v[i] = Vec4{r.Float32()*20 - 10, r.Float32()*20 - 10, r.Float32()*20 - 10, r.Float32()*2 - 1}
	}
	return v
}

// TestBatchTransformExactness checks every batch entry point against the
// per-vertex scalar reference using bitwise comparison. Any reordering of the
// float32 multiply-add sequence would show up here, so the test pins the
// no-approximation contract.
func TestBatchTransformExactness(t *testing.T) {
	const n = 1000
	r := rand.New(rand.NewSource(42))
	m4 := randMat4(r)
	m3 := randMat3(r)
	src3 := randVec3s(r, n)
	src4 := randVec4s(r, n)

	t.Run("Vec4", func(t *testing.T) {
		got := make([]Vec4, n)
		BatchTransformVec4(got, src4, m4)
		for i := range src4 {
			want := m4.Mul4x1(src4[i])
			if got[i] != want {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], want)
			}
		}
	})

	t.Run("Vec3", func(t *testing.T) {
		got := make([]Vec3, n)
		BatchTransformVec3(got, src3, m3)
		for i := range src3 {
			want := m3.Mul3x1(src3[i])
			if got[i] != want {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], want)
			}
		}
	})

	t.Run("Points", func(t *testing.T) {
		got := make([]Vec3, n)
		BatchTransformPoints(got, src3, m4)
		for i, v := range src3 {
			ref := m4.Mul4x1(v.Vec4(1)).Vec3()
			if got[i] != ref {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], ref)
			}
		}
	})

	t.Run("Directions", func(t *testing.T) {
		got := make([]Vec3, n)
		BatchTransformDirections(got, src3, m4)
		for i, v := range src3 {
			ref := m4.Mul4x1(v.Vec4(0)).Vec3()
			if got[i] != ref {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], ref)
			}
			if ref != TransformNormal(v, m4) {
				t.Fatalf("vertex %d diverges from TransformNormal", i)
			}
		}
	})

	t.Run("ProjectPoints", func(t *testing.T) {
		// A perspective matrix guarantees non-trivial w values.
		proj := m4.Mul4(Perspective(DegToRad(60), 1.6, 0.1, 100))
		got := make([]Vec3, n)
		BatchProjectPoints(got, src3, proj)
		for i, v := range src3 {
			ref := TransformCoordinate(v, proj)
			if got[i] != ref {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], ref)
			}
		}
	})

	t.Run("PointsFromVec3", func(t *testing.T) {
		got := make([]Vec4, n)
		BatchTransformPointsFromVec3(got, src3, m4)
		for i, v := range src3 {
			ref := m4.Mul4x1(v.Vec4(1))
			if got[i] != ref {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], ref)
			}
		}
	})

	t.Run("MVP", func(t *testing.T) {
		model := randMat4(r)
		view := randMat4(r)
		proj := randMat4(r)
		mvp := proj.Mul4(view.Mul4(model))

		got := make([]Vec4, n)
		BatchTransformPointsMVP(got, src3, model, view, proj)
		for i, v := range src3 {
			ref := mvp.Mul4x1(v.Vec4(1))
			if got[i] != ref {
				t.Fatalf("vertex %d: got %v want %v", i, got[i], ref)
			}
		}
	})

	t.Run("BatchTransformer", func(t *testing.T) {
		model := randMat4(r)
		view := randMat4(r)
		proj := randMat4(r)
		mvp := proj.Mul4(view.Mul4(model))

		bt := NewBatchTransformer(n)
		got := make([]Vec4, n)
		bt.Transform(got, src3, model, view, proj)
		gotStaged := make([]Vec4, n)
		bt.TransformStaged(gotStaged, src3, model, view, proj)
		for i, v := range src3 {
			ref := mvp.Mul4x1(v.Vec4(1))
			if got[i] != ref {
				t.Fatalf("Transform vertex %d: got %v want %v", i, got[i], ref)
			}
		}
		// The staged pipeline must exactly match an explicit three-pass
		// scalar loop, including its intermediate float32 rounding.
		for i, v := range src3 {
			ref := proj.Mul4x1(view.Mul4x1(model.Mul4x1(v.Vec4(1))))
			if gotStaged[i] != ref {
				t.Fatalf("TransformStaged vertex %d: got %v want %v", i, gotStaged[i], ref)
			}
		}
	})
}

// TestBatchInPlace verifies aliasing safety when dst and src share storage.
func TestBatchInPlace(t *testing.T) {
	const n = 128
	r := rand.New(rand.NewSource(7))
	m4 := randMat4(r)
	m3 := randMat3(r)

	v4 := randVec4s(r, n)
	want4 := make([]Vec4, n)
	BatchTransformVec4(want4, v4, m4)
	BatchTransformVec4(v4, v4, m4)
	for i := range want4 {
		if v4[i] != want4[i] {
			t.Fatalf("Vec4 in-place vertex %d: got %v want %v", i, v4[i], want4[i])
		}
	}

	v3 := randVec3s(r, n)
	want3 := make([]Vec3, n)
	BatchTransformVec3(want3, v3, m3)
	BatchTransformVec3(v3, v3, m3)
	for i := range want3 {
		if v3[i] != want3[i] {
			t.Fatalf("Vec3 in-place vertex %d: got %v want %v", i, v3[i], want3[i])
		}
	}

	points := randVec3s(r, n)
	wantP := make([]Vec3, n)
	BatchProjectPoints(wantP, points, m4)
	BatchProjectPoints(points, points, m4)
	for i := range wantP {
		if points[i] != wantP[i] {
			t.Fatalf("points in-place vertex %d: got %v want %v", i, points[i], wantP[i])
		}
	}
}

// TestBatchNoAllocations pins the zero-allocation contract for the hot path.
func TestBatchNoAllocations(t *testing.T) {
	const n = 4096
	r := rand.New(rand.NewSource(99))
	m4 := randMat4(r)
	m3 := randMat3(r)
	src3 := randVec3s(r, n)
	src4 := randVec4s(r, n)
	dst3 := make([]Vec3, n)
	dst4 := make([]Vec4, n)
	bt := NewBatchTransformer(n)

	allocs := testing.AllocsPerRun(10, func() {
		BatchTransformVec4(dst4, src4, m4)
		BatchTransformVec3(dst3, src3, m3)
		BatchTransformPoints(dst3, src3, m4)
		BatchTransformDirections(dst3, src3, m4)
		BatchProjectPoints(dst3, src3, m4)
		BatchTransformPointsFromVec3(dst4, src3, m4)
		BatchTransformPointsMVP(dst4, src3, m4, m4, m4)
		bt.Transform(dst4, src3, m4, m4, m4)
		bt.TransformStaged(dst4, src3, m4, m4, m4)
	})
	if allocs != 0 {
		t.Fatalf("batch transforms allocated %v times per run, want 0", allocs)
	}
}

// batchSizes drives the timing curve from tiny batches up to the acceptance
// target of one million vertices.
var batchSizes = []int{1, 10, 100, 1000, 10000, 100000, 1000000}

func makeBenchData(b *testing.B, n int) ([]Vec3, []Vec4, Mat4) {
	r := rand.New(rand.NewSource(1))
	m := randMat4(r)
	src := randVec3s(r, n)
	dst := make([]Vec4, n)
	return src, dst, m
}

func BenchmarkBatchTransformPointsMVP(b *testing.B) {
	for _, n := range batchSizes {
		src, dst, m := makeBenchData(b, n)
		bt := NewBatchTransformer(n)
		b.Run(label(n), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				bt.Transform(dst, src, m, m, m)
			}
		})
	}
}

func BenchmarkBatchTransformPointsFromVec3(b *testing.B) {
	for _, n := range batchSizes {
		src, dst, m := makeBenchData(b, n)
		b.Run(label(n), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				BatchTransformPointsFromVec3(dst, src, m)
			}
		})
	}
}

// BenchmarkScalarReference is the naive per-vertex loop using the existing
// scalar API, kept as the precision baseline and timing comparison point.
func BenchmarkScalarReference(b *testing.B) {
	for _, n := range batchSizes {
		src, dst, m := makeBenchData(b, n)
		b.Run(label(n), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				for j, v := range src {
					dst[j] = m.Mul4x1(v.Vec4(1))
				}
			}
		})
	}
}

func BenchmarkBatchTransformVec4(b *testing.B) {
	for _, n := range batchSizes {
		r := rand.New(rand.NewSource(1))
		src := randVec4s(r, n)
		dst := make([]Vec4, n)
		m := randMat4(r)
		b.Run(label(n), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				BatchTransformVec4(dst, src, m)
			}
		})
	}
}

func BenchmarkBatchTransformVec3(b *testing.B) {
	for _, n := range batchSizes {
		r := rand.New(rand.NewSource(1))
		src := randVec3s(r, n)
		dst := make([]Vec3, n)
		m := randMat3(r)
		b.Run(label(n), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				BatchTransformVec3(dst, src, m)
			}
		})
	}
}

func label(n int) string {
	switch {
	case n >= 1000000:
		return "1M"
	case n >= 1000:
		return itoa(n/1000) + "k"
	default:
		return itoa(n)
	}
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	var buf [16]byte
	i := len(buf)
	for n > 0 {
		i--
		buf[i] = byte('0' + n%10)
		n /= 10
	}
	return string(buf[i:])
}
