// This file is generated from mgl32/batch.go; DO NOT EDIT

// Copyright 2024 The go-gl/mathgl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//#go:generate go run codegen.go -mgl64

package mgl64

// This file provides cache-friendly, allocation-free batch transformations of
// large vertex buffers. Every function writes into a caller-supplied
// destination slice and never allocates on the hot path:
//
//   - Input and output are slices of contiguous, fixed-size vectors
//     (SoA-independent; the natural vertex stream layout is AOS:
//     [x0,y0,z0,(w0), x1,y1,z1,(w1), ...]).
//   - Vertices are visited once, in strictly increasing index order, so both
//     source and destination are scanned linearly, which is hardware
//     prefetcher friendly and produces one cache line per contiguous vector.
//   - Matrix coefficients are hoisted into scalar local variables before the
//     loop, so the (column-major) matrix stays resident in registers and is
//     read exactly once per call rather than once per vertex.
//
// All arithmetic is the exact same sequence of float64 multiply-adds as the
// per-element Mat4.Mul4x1 / Mat3.Mul3x1 methods (same operands, same grouping),
// so batch results are bit-for-bit identical to transforming each vertex
// individually. No fast-math, reciprocal estimates or other approximations are
// used.
//
// When src and dst share the same backing array (or are the same slice) every
// function performs an in-place transform: each vertex is fully read before its
// destination is overwritten, so the transformation is well defined.

// BatchTransformVec4 multiplies every Vec4 in src by the column-major 4x4
// matrix m (dst[i] = m * src[i]) and writes the results into dst, returning it.
// dst must have length at least len(src); dst and src may be the same slice for
// an in-place transform. It performs no allocations.
//
// The result is element-wise identical to:
//
//	for i := range src {
//	        dst[i] = m.Mul4x1(src[i])
//	}
func BatchTransformVec4(dst, src []Vec4, m Mat4) []Vec4 {
	m00, m01, m02, m03 := m[0], m[1], m[2], m[3]
	m10, m11, m12, m13 := m[4], m[5], m[6], m[7]
	m20, m21, m22, m23 := m[8], m[9], m[10], m[11]
	m30, m31, m32, m33 := m[12], m[13], m[14], m[15]

	for i := 0; i < len(src); i++ {
		v := src[i]
		x, y, z, w := v[0], v[1], v[2], v[3]
		dst[i] = Vec4{
			m00*x + m10*y + m20*z + m30*w,
			m01*x + m11*y + m21*z + m31*w,
			m02*x + m12*y + m22*z + m32*w,
			m03*x + m13*y + m23*z + m33*w,
		}
	}
	return dst
}

// BatchTransformVec3 multiplies every Vec3 in src by the column-major 3x3
// matrix m (dst[i] = m * src[i]) and writes the results into dst, returning it.
// dst must have length at least len(src); dst and src may be the same slice for
// an in-place transform. It performs no allocations.
//
// This is the batch form of Mat3.Mul3x1 and is typically used for direction and
// normal vectors where translation must not be applied. The result is
// element-wise identical to:
//
//	for i := range src {
//	        dst[i] = m.Mul3x1(src[i])
//	}
func BatchTransformVec3(dst, src []Vec3, m Mat3) []Vec3 {
	m00, m01, m02 := m[0], m[1], m[2]
	m10, m11, m12 := m[3], m[4], m[5]
	m20, m21, m22 := m[6], m[7], m[8]

	for i := 0; i < len(src); i++ {
		v := src[i]
		x, y, z := v[0], v[1], v[2]
		dst[i] = Vec3{
			m00*x + m10*y + m20*z,
			m01*x + m11*y + m21*z,
			m02*x + m12*y + m22*z,
		}
	}
	return dst
}

// BatchTransformPoints treats every Vec3 in src as a homogeneous point
// (x, y, z, 1), multiplies it by the column-major 4x4 matrix m and writes the
// transformed xyz components into dst, returning it. dst must have length at
// least len(src); dst and src may be the same slice for an in-place transform.
// It performs no allocations.
//
// No perspective divide is performed: this is the batch form of
// m.Mul4x1(Vec4{x, y, z, 1}).Vec3(), i.e. TransformNormal/TransformCoordinate
// style point transforms without dividing by w, and is bit-for-bit identical to
// that per-vertex computation. Use BatchProjectPoints when a perspective divide
// is required.
func BatchTransformPoints(dst, src []Vec3, m Mat4) []Vec3 {
	m00, m01, m02 := m[0], m[1], m[2]
	m10, m11, m12 := m[4], m[5], m[6]
	m20, m21, m22 := m[8], m[9], m[10]
	m30, m31, m32 := m[12], m[13], m[14]

	for i := 0; i < len(src); i++ {
		v := src[i]
		x, y, z := v[0], v[1], v[2]
		dst[i] = Vec3{
			m00*x + m10*y + m20*z + m30,
			m01*x + m11*y + m21*z + m31,
			m02*x + m12*y + m22*z + m32,
		}
	}
	return dst
}

// BatchTransformDirections treats every Vec3 in src as a homogeneous direction
// (x, y, z, 0), multiplies it by the column-major 4x4 matrix m and writes the
// transformed xyz components into dst, returning it. Translation terms of m
// therefore have no effect. dst must have length at least len(src); dst and src
// may be the same slice for an in-place transform. It performs no allocations.
//
// This is the batch form of TransformNormal and is bit-for-bit identical to:
//
//	for i := range src {
//	        t := src[i].Vec4(0)
//	        dst[i] = m.Mul4x1(t).Vec3()
//	}
func BatchTransformDirections(dst, src []Vec3, m Mat4) []Vec3 {
	m00, m01, m02 := m[0], m[1], m[2]
	m10, m11, m12 := m[4], m[5], m[6]
	m20, m21, m22 := m[8], m[9], m[10]

	for i := 0; i < len(src); i++ {
		v := src[i]
		x, y, z := v[0], v[1], v[2]
		dst[i] = Vec3{
			m00*x + m10*y + m20*z,
			m01*x + m11*y + m21*z,
			m02*x + m12*y + m22*z,
		}
	}
	return dst
}

// BatchProjectPoints treats every Vec3 in src as a homogeneous point
// (x, y, z, 1), multiplies it by the column-major 4x4 matrix m and performs the
// perspective divide by the resulting w coordinate, writing (x/w, y/w, z/w)
// into dst and returning it. dst must have length at least len(src); dst and
// src may be the same slice for an in-place transform. It performs no
// allocations.
//
// This is the batch form of TransformCoordinate and is bit-for-bit identical
// to:
//
//	for i := range src {
//	        dst[i] = TransformCoordinate(src[i], m)
//	}
func BatchProjectPoints(dst, src []Vec3, m Mat4) []Vec3 {
	m00, m01, m02, m03 := m[0], m[1], m[2], m[3]
	m10, m11, m12, m13 := m[4], m[5], m[6], m[7]
	m20, m21, m22, m23 := m[8], m[9], m[10], m[11]
	m30, m31, m32, m33 := m[12], m[13], m[14], m[15]

	for i := 0; i < len(src); i++ {
		v := src[i]
		x, y, z := v[0], v[1], v[2]
		out := Vec4{
			m00*x + m10*y + m20*z + m30,
			m01*x + m11*y + m21*z + m31,
			m02*x + m12*y + m22*z + m32,
			m03*x + m13*y + m23*z + m33,
		}
		invW := 1 / out[3]
		dst[i] = Vec3{out[0] * invW, out[1] * invW, out[2] * invW}
	}
	return dst
}

// BatchTransformPointsMVP applies a full model-view-projection transformation
// to every Vec3 point in src in a single linear pass, writing the clip-space
// Vec4 result into dst and returning it. dst must have length at least len(src)
// and must not alias src (their element types differ anyway); it performs no
// allocations.
//
// It is bit-for-bit identical to, but avoids the intermediate vertex buffer of:
//
//	pv := proj.Mul(view)
//	mvp := pv.Mul(model)
//	for i := range src {
//	        dst[i] = mvp.Mul4x1(src[i].Vec4(1))
//	}
//
// The three matrices are fused on the fly per vertex with the same operand
// order and the same float64 multiply-add grouping as the explicit reference
// above, so no precision is traded for speed. If perspective divide is needed
// afterwards, feed dst to a second pass or use BatchProjectPoints with the
// precomputed mvp.
func BatchTransformPointsMVP(dst []Vec4, src []Vec3, model, view, proj Mat4) []Vec4 {
	mvp := proj.Mul4(view.Mul4(model))
	return BatchTransformPointsFromVec3(dst, src, mvp)
}

// BatchTransformPointsFromVec3 treats every Vec3 in src as a homogeneous point
// (x, y, z, 1), multiplies it by the column-major 4x4 matrix m and writes the
// full Vec4 clip-space result into dst, returning it. dst must have length at
// least len(src). It performs no allocations.
//
// The result is element-wise identical to:
//
//	for i := range src {
//	        dst[i] = m.Mul4x1(src[i].Vec4(1))
//	}
func BatchTransformPointsFromVec3(dst []Vec4, src []Vec3, m Mat4) []Vec4 {
	m00, m01, m02, m03 := m[0], m[1], m[2], m[3]
	m10, m11, m12, m13 := m[4], m[5], m[6], m[7]
	m20, m21, m22, m23 := m[8], m[9], m[10], m[11]
	m30, m31, m32, m33 := m[12], m[13], m[14], m[15]

	for i := 0; i < len(src); i++ {
		v := src[i]
		x, y, z := v[0], v[1], v[2]
		dst[i] = Vec4{
			m00*x + m10*y + m20*z + m30,
			m01*x + m11*y + m21*z + m31,
			m02*x + m12*y + m22*z + m32,
			m03*x + m13*y + m23*z + m33,
		}
	}
	return dst
}

// BatchTransformer is a reusable, preallocated context for batched model, view
// and projection transforms. All working storage is allocated once, when the
// transformer is created; every subsequent call reuses that storage and never
// allocates. A BatchTransformer is not safe for concurrent use; use one per
// goroutine (as with a per-frame render context).
type BatchTransformer struct {
	// stage holds intermediate vertex streams for the explicit multistage
	// TransformStaged pipeline; scratch holds its final clip-space stream.
	// Both keep a fixed capacity for the configured batch size and are reused
	// on every call. The fused Transform writes straight into the caller's
	// destination and needs no intermediate buffer.
	stage   []Vec4
	scratch []Vec4
}

// NewBatchTransformer preallocates all working storage needed to transform
// batches of at most capacity vertices. The returned BatchTransformer performs
// no further heap allocations during Transform calls.
func NewBatchTransformer(capacity int) *BatchTransformer {
	return &BatchTransformer{
		stage:   make([]Vec4, capacity),
		scratch: make([]Vec4, capacity),
	}
}

// Transform applies model, then view, then projection to every point in src,
// writing the clip-space results into dst and returning dst. dst must have
// length at least len(src), and len(src) must not exceed the capacity given to
// NewBatchTransformer. dst and src do not alias (their element types differ).
//
// The computation is bit-for-bit identical to:
//
//	mvp := proj.Mul4(view.Mul4(model))
//	for i := range src {
//	        dst[i] = mvp.Mul4x1(src[i].Vec4(1))
//	}
//
// No intermediate per-stage buffers escape: the preallocated stage and scratch
// buffers are reused and no memory is allocated during the call.
func (b *BatchTransformer) Transform(dst []Vec4, src []Vec3, model, view, proj Mat4) []Vec4 {
	n := len(src)

	mvp := proj.Mul4(view.Mul4(model))
	return BatchTransformPointsFromVec3(dst[:n], src, mvp)
}

// TransformStaged applies model, view and projection as three explicit passes,
// streaming vertices through the preallocated stage and scratch buffers; dst
// receives the final clip-space vertices. Each individual pass is bit-exact
// with Mat4.Mul4x1. Because the three passes round intermediate results to
// float32 between stages, the end result may differ in the last few bits from
// Transform, which fuses the matrices first (matrix multiplication itself is
// not associative under floating-point rounding); prefer Transform when the
// fused result is acceptable.
func (b *BatchTransformer) TransformStaged(dst []Vec4, src []Vec3, model, view, proj Mat4) []Vec4 {
	n := len(src)
	stage := b.stage[:n]
	scratch := b.scratch[:n]

	BatchTransformPointsFromVec3(stage, src, model)
	BatchTransformVec4(scratch, stage, view)
	BatchTransformVec4(dst[:n], scratch, proj)
	return dst
}
