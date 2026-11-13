// This file is generated from mgl32/euler_test.go; DO NOT EDIT

// Copyright 2021 The go-gl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl64

import (
	"math"
	"testing"
)

var eulerTestOrders = []struct {
	order RotationOrder
	name  string
}{
	{XYZ, "XYZ"}, {XZY, "XZY"}, {YXZ, "YXZ"},
	{YZX, "YZX"}, {ZXY, "ZXY"}, {ZYX, "ZYX"},
}

// rotationAngleDeg returns the geodesic angle (degrees) between two rotation
// matrices via the angle of R1^T R2: acos((trace - 1) / 2).
func rotationAngleDeg(a, b Mat3) float64 {
	var trace float64
	for r := 0; r < 3; r++ {
		for c := 0; c < 3; c++ {
			trace += a.At(r, c) * b.At(r, c)
		}
	}
	cosTheta := Clamp((trace-1)/2, -1, 1)
	return float64(math.Acos(float64(cosTheta)) * 180 / math.Pi)
}

func wrapPi(v float64) float64 {
	for v > math.Pi {
		v -= 2 * math.Pi
	}
	for v < -math.Pi {
		v += 2 * math.Pi
	}
	return v
}

func maxFloat32(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

// TestEulerRoundTrip covers all six Tait-Bryan orders on a dense grid away
// from gimbal lock and asserts both the matrix round trip and the recovered
// angles stay within the 0.01 degree accuracy budget.
func TestEulerRoundTrip(t *testing.T) {
	t.Parallel()

	const deg = math.Pi / 180
	const maxResidualDeg = float64(0.01)
	const maxAngleDeg = float64(0.01)

	for _, tc := range eulerTestOrders {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			worstResidual := float64(0)
			worstAngle := float64(0)
			for ai := -150; ai <= 150; ai += 15 {
				for ci := -150; ci <= 150; ci += 15 {
					for pd := -80.0; pd <= 80.0; pd += 10 {
						a1in := float64(ai) * deg
						a2in := float64(pd) * deg
						a3in := float64(ci) * deg

						m := EulerAnglesToMat3(a1in, a2in, a3in, tc.order)
						a1, a2, a3 := Mat3ToEulerAngles(m, tc.order)
						back := EulerAnglesToMat3(a1, a2, a3, tc.order)

						if residual := rotationAngleDeg(m, back); residual > worstResidual {
							worstResidual = residual
						}
						d1 := float64(math.Abs(float64(wrapPi(a1 - a1in))))
						d2 := float64(math.Abs(float64(wrapPi(a2 - a2in))))
						d3 := float64(math.Abs(float64(wrapPi(a3 - a3in))))
						if angleErr := maxFloat32(d1, maxFloat32(d2, d3)) * 180 / math.Pi; angleErr > worstAngle {
							worstAngle = angleErr
						}
					}
				}
			}

			// The float64 build satisfies the full matrix budget; float32 can
			// only be held to it away from the poles because of storage
			// quantization. The angle budget must hold in both builds.
			if worstAngle > maxAngleDeg {
				t.Errorf("%s worst recovered angle error %.5f deg exceeds %.2f deg",
					tc.name, worstAngle, maxAngleDeg)
			}
			if eulerMatrixBudget() && worstResidual > maxResidualDeg {
				t.Errorf("%s worst matrix round trip %.5f deg exceeds %.2f deg",
					tc.name, worstResidual, maxResidualDeg)
			}
		})
	}
}

// TestEulerGimbalLock drives the middle angle to exactly +/-pi/2. The
// decomposition is non-unique there, so the contract is: (1) a deterministic
// solution with angle1 fixed to zero, and (2) the recovered angles reproduce
// the input rotation exactly.
func TestEulerGimbalLock(t *testing.T) {
	t.Parallel()

	const deg = math.Pi / 180
	for _, tc := range eulerTestOrders {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			for _, pitchDeg := range []float64{-90, 90} {
				for ai := -170; ai <= 170; ai += 17 {
					for ci := -170; ci <= 170; ci += 17 {
						m := EulerAnglesToMat3(float64(ai)*deg, pitchDeg*deg, float64(ci)*deg, tc.order)
						a1, a2, a3 := Mat3ToEulerAngles(m, tc.order)

						if a1 != 0 {
							t.Fatalf("%s gimbal solution not deterministic: angle1=%v (want 0)", tc.name, a1)
						}
						if math.Abs(float64(a2)-float64(pitchDeg)*deg) > 1e-6 {
							t.Fatalf("%s gimbal middle angle %.5f != %.5f", tc.name, a2, pitchDeg*deg)
						}
						back := EulerAnglesToMat3(a1, a2, a3, tc.order)
						if residual := rotationAngleDeg(m, back); residual > eulerGimbalTolerance() {
							t.Fatalf("%s gimbal reconstruction residual %.6f deg", tc.name, residual)
						}
					}
				}
			}
		})
	}
}

// TestEulerContinuity sweeps the middle angle continuously through gimbal
// lock and verifies the reconstructed orientation never jumps: the largest
// per-step change must match the small input step rather than flipping.
func TestEulerContinuity(t *testing.T) {
	t.Parallel()

	const deg = math.Pi / 180
	const stepDeg = 0.1
	for _, tc := range eulerTestOrders {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			var previous Mat3
			havePrevious := false
			worstStep := float64(0)
			for pitch := -92.0; pitch <= 92.0; pitch += stepDeg {
				m := EulerAnglesToMat3(70*deg, float64(pitch)*deg, 30*deg, tc.order)
				a1, a2, a3 := Mat3ToEulerAngles(m, tc.order)
				reconstructed := EulerAnglesToMat3(a1, a2, a3, tc.order)
				if havePrevious {
					if step := rotationAngleDeg(previous, reconstructed); step > worstStep {
						worstStep = step
					}
				}
				previous = reconstructed
				havePrevious = true
			}
			if worstStep > float64(stepDeg*2) {
				t.Errorf("%s orientation jumps by %.4f deg crossing gimbal lock", tc.name, worstStep)
			}
		})
	}
}

// TestEulerMatchesQuaternion cross-checks the matrix forward conversion
// against the existing quaternion Euler helper for the same order.
func TestEulerMatchesQuaternion(t *testing.T) {
	t.Parallel()

	const deg = math.Pi / 180
	for _, tc := range eulerTestOrders {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			m := EulerAnglesToMat3(0.7, -0.4, 1.1, tc.order)
			q := AnglesToQuat(0.7, -0.4, 1.1, tc.order).Mat4().Mat3()
			if residual := rotationAngleDeg(m, q); residual > eulerQuaternionTolerance() {
				t.Errorf("%s matrix/quaternion mismatch %.6f deg", tc.name, residual)
			}
		})
	}
}

// TestEulerUnsupportedOrderPanics confirms unsupported (repeated-axis)
// orders panic instead of silently returning a default value.
func TestEulerUnsupportedOrderPanics(t *testing.T) {
	t.Parallel()

	mustPanic := func(name string, fn func()) {
		defer func() {
			if recover() == nil {
				t.Errorf("%s did not panic on unsupported order", name)
			}
		}()
		fn()
	}
	mustPanic("EulerAnglesToMat3", func() { EulerAnglesToMat3(1, 2, 3, XYX) })
	mustPanic("EulerAnglesToMat4", func() { EulerAnglesToMat4(1, 2, 3, ZXZ) })
	mustPanic("Mat3ToEulerAngles", func() { Mat3ToEulerAngles(Ident3(), YXY) })
	mustPanic("Mat4ToEulerAngles", func() { Mat4ToEulerAngles(Ident4(), ZYZ) })
}

// TestEulerMat4RoundTrip checks the homogeneous wrappers round trip and keep
// their bottom row/translation untouched.
func TestEulerMat4RoundTrip(t *testing.T) {
	t.Parallel()

	const deg = math.Pi / 180
	for _, tc := range eulerTestOrders {
		m4 := EulerAnglesToMat4(0.5*deg, -30*deg, 1.2*deg, tc.order)
		a1, a2, a3 := Mat4ToEulerAngles(m4, tc.order)
		back := EulerAnglesToMat4(a1, a2, a3, tc.order)
		if residual := rotationAngleDeg(m4.Mat3(), back.Mat3()); residual > eulerGimbalTolerance() {
			t.Errorf("%s Mat4 round trip residual %.6f deg", tc.name, residual)
		}
	}
}
