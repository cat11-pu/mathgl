// This file is generated from mgl32/euler.go; DO NOT EDIT

// Copyright 2021 The go-gl Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package mgl64

import "math"

// eulerSingularityEpsilon is compared against 1-sin(angle2)^2 =
// cos(angle2)^2 to decide whether a Tait-Bryan triple is at gimbal lock
// (middle angle +/- pi/2). The float32 package uses 1e-7 and the float64
// package tightens this to 1e-12 (mgl64/euler.go overrides this declaration
// after code generation). At these values the generic atan2 branch stays
// numerically safe above the threshold, while the deterministic singularity
// branch is only entered within a few thousandths of a degree of the pole,
// far inside the 0.01 degree accuracy target.
const eulerSingularityEpsilon = 1e-12

// eulerMatrixBudget reports whether this package's floating-point storage can
// hold the full 0.01 degree matrix round-trip budget. float64 can; float32
// matrix storage cannot near the poles, so float32 tests assert angle accuracy
// instead and check matrix reconstruction only at gimbal lock.
func eulerMatrixBudget() bool { return true }

// eulerGimbalTolerance is the accepted reconstruction residual (degrees) at an
// exact gimbal-lock matrix, scaled to the package precision.
func eulerGimbalTolerance() float64 { return 1e-5 }

// eulerQuaternionTolerance is the accepted matrix/quaternion agreement
// (degrees), scaled to the package precision.
func eulerQuaternionTolerance() float64 { return 1e-5 }

// The six Tait-Bryan (proper) rotation orders supported by the Euler angle
// helpers: XYZ, XZY, YXZ, YZX, ZXY and ZYX. The remaining RotationOrder
// values (the repeated-axis "proper Euler" orders such as XYX) are not
// supported and make the helpers panic, matching the convention used by
// AnglesToQuat for unknown orders.

// rot3DX returns the 3x3 rotation matrix for an angle about the X axis.
func rot3DX(angle float64) Mat3 {
	sin, cos := math.Sincos(float64(angle))
	return Mat3{
		1, 0, 0,
		0, float64(cos), float64(sin),
		0, float64(-sin), float64(cos),
	}
}

// rot3DY returns the 3x3 rotation matrix for an angle about the Y axis.
func rot3DY(angle float64) Mat3 {
	sin, cos := math.Sincos(float64(angle))
	return Mat3{
		float64(cos), 0, float64(-sin),
		0, 1, 0,
		float64(sin), 0, float64(cos),
	}
}

// rot3DZ returns the 3x3 rotation matrix for an angle about the Z axis.
func rot3DZ(angle float64) Mat3 {
	sin, cos := math.Sincos(float64(angle))
	return Mat3{
		float64(cos), float64(sin), 0,
		float64(-sin), float64(cos), 0,
		0, 0, 1,
	}
}

// EulerAnglesToMat3 converts three Euler angles into a 3x3 rotation matrix
// using one of the six supported Tait-Bryan rotation orders. The angles are
// in radians and the order describes the intrinsic rotations applied to the
// moving frame; for example ZYX with angle1=yaw, angle2=pitch and angle3=roll
// produces the common aerospace R = Rz(yaw) * Ry(pitch) * Rx(roll) matrix.
//
// The result is identical to composing Rotate3DX/Y/Z in the given order and
// is consistent with AnglesToQuat for the same order. Passing an unsupported
// order panics instead of silently returning a default matrix.
func EulerAnglesToMat3(angle1, angle2, angle3 float64, order RotationOrder) Mat3 {
	switch order {
	case XYZ:
		return rot3DX(angle1).Mul3(rot3DY(angle2)).Mul3(rot3DZ(angle3))
	case XZY:
		return rot3DX(angle1).Mul3(rot3DZ(angle2)).Mul3(rot3DY(angle3))
	case YXZ:
		return rot3DY(angle1).Mul3(rot3DX(angle2)).Mul3(rot3DZ(angle3))
	case YZX:
		return rot3DY(angle1).Mul3(rot3DZ(angle2)).Mul3(rot3DX(angle3))
	case ZXY:
		return rot3DZ(angle1).Mul3(rot3DX(angle2)).Mul3(rot3DY(angle3))
	case ZYX:
		return rot3DZ(angle1).Mul3(rot3DY(angle2)).Mul3(rot3DX(angle3))
	default:
		panic("Unsupported rotation order: only the six Tait-Bryan orders XYZ, XZY, YXZ, YZX, ZXY and ZYX are supported")
	}
}

// EulerAnglesToMat4 is EulerAnglesToMat3 with the rotation embedded in a
// homogeneous 4x4 matrix.
func EulerAnglesToMat4(angle1, angle2, angle3 float64, order RotationOrder) Mat4 {
	return EulerAnglesToMat3(angle1, angle2, angle3, order).Mat4()
}

// Mat3ToEulerAngles is the inverse of EulerAnglesToMat3: it extracts three
// Euler angles in radians that reproduce the given rotation matrix under the
// requested order. The first returned value corresponds to angle1 and so on.
//
// mathgl stores rotations in the row-vector convention (its Rotate3DX/Y/Z
// matrices are the transpose of the conventional column-vector form), so the
// matrix is first transposed into standard convention. All extraction
// arithmetic is carried out in float64 even for the float32 package, which
// keeps near-gimbal round-off at the storage-format limit rather than
// amplifying float32 error through the small cosine divisor.
//
// Away from gimbal lock the canonical ranges are: angle1 and angle3 in
// [-pi, pi] and the middle angle in [-pi/2, pi/2]. At gimbal lock the
// decomposition is not unique, so a deterministic solution is returned:
// angle1 is fixed to zero and the full rotation is assigned to angle3.
// Because the extracted angles always reconstruct the exact matrix passed
// in, interpolating the underlying rotations (e.g. via quaternion slerp)
// never jumps or flips when the middle angle sweeps through +/- pi/2.
//
// Precision note: the extraction arithmetic runs in float64, so the returned
// angles are accurate to roughly 1e-5 degrees even in this package. The
// round trip through a float32 matrix is nevertheless limited by float32
// matrix storage itself (~0.03 degrees near the poles); applications that
// need the full 0.01 degree matrix round-trip accuracy close to gimbal lock
// should use the mgl64 package, whose round-trip error is below 1e-5 degrees.
// Passing an unsupported order panics.
func Mat3ToEulerAngles(m Mat3, order RotationOrder) (angle1, angle2, angle3 float64) {
	// R holds the rotation in standard column-vector convention:
	// R[row][col] = mathgl m[col][row] = m[col*3 + row], widened to float64.
	var R [3][3]float64
	for r := 0; r < 3; r++ {
		for c := 0; c < 3; c++ {
			R[r][c] = float64(m[3*c+r])
		}
	}
	a1, a2, a3 := eulerAnglesFromStandard(R, order, eulerSingularityEpsilon)
	return float64(a1), float64(a2), float64(a3)
}

// eulerAnglesFromStandard extracts Tait-Bryan angles from a rotation matrix
// given in the standard column-vector convention R = R0(a1) R1(a2) R2(a3).
// The formulas were derived algebraically for each of the six orders and
// checked numerically across a dense grid including exact +/-pi/2
// gimbal-lock matrices.
func eulerAnglesFromStandard(R [3][3]float64, order RotationOrder, epsilon float64) (angle1, angle2, angle3 float64) {
	atan2 := math.Atan2
	asin := func(v float64) float64 {
		return math.Asin(math.Max(-1, math.Min(1, v)))
	}

	switch order {
	case XYZ:
		s2 := math.Max(-1, math.Min(1, R[0][2]))
		angle2 = asin(s2)
		if math.Abs(1-s2*s2) < epsilon {
			angle1 = 0
			angle3 = atan2(R[1][0], R[1][1])
			return
		}
		angle1 = atan2(-R[1][2], R[2][2])
		angle3 = atan2(-R[0][1], R[0][0])

	case XZY:
		s2 := math.Max(-1, math.Min(1, -R[0][1]))
		angle2 = asin(s2)
		if math.Abs(1-s2*s2) < epsilon {
			angle1 = 0
			if s2 > 0 {
				angle3 = atan2(R[1][2], R[1][0])
			} else {
				angle3 = atan2(-R[1][2], -R[1][0])
			}
			return
		}
		angle1 = atan2(R[2][1], R[1][1])
		angle3 = atan2(R[0][2], R[0][0])

	case YXZ:
		s2 := math.Max(-1, math.Min(1, -R[1][2]))
		angle2 = asin(s2)
		if math.Abs(1-s2*s2) < epsilon {
			angle1 = 0
			angle3 = atan2(-R[0][1], R[0][0])
			return
		}
		angle1 = atan2(R[0][2], R[2][2])
		angle3 = atan2(R[1][0], R[1][1])

	case YZX:
		s2 := math.Max(-1, math.Min(1, R[1][0]))
		angle2 = asin(s2)
		if math.Abs(1-s2*s2) < epsilon {
			angle1 = 0
			if s2 > 0 {
				angle3 = atan2(R[0][2], -R[0][1])
			} else {
				angle3 = atan2(-R[0][2], R[0][1])
			}
			return
		}
		angle1 = atan2(-R[2][0], R[0][0])
		angle3 = atan2(-R[1][2], R[1][1])

	case ZXY:
		s2 := math.Max(-1, math.Min(1, R[2][1]))
		angle2 = asin(s2)
		if math.Abs(1-s2*s2) < epsilon {
			angle1 = 0
			angle3 = atan2(R[0][2], R[0][0])
			return
		}
		angle1 = atan2(-R[0][1], R[1][1])
		angle3 = atan2(-R[2][0], R[2][2])

	case ZYX:
		s2 := math.Max(-1, math.Min(1, -R[2][0]))
		angle2 = asin(s2)
		if math.Abs(1-s2*s2) < epsilon {
			angle1 = 0
			if s2 > 0 {
				angle3 = atan2(R[0][1], R[0][2])
			} else {
				angle3 = atan2(-R[0][1], -R[0][2])
			}
			return
		}
		angle1 = atan2(R[1][0], R[0][0])
		angle3 = atan2(R[2][1], R[2][2])

	default:
		panic("Unsupported rotation order: only the six Tait-Bryan orders XYZ, XZY, YXZ, YZX, ZXY and ZYX are supported")
	}

	return angle1, angle2, angle3
}

// Mat4ToEulerAngles is Mat3ToEulerAngles for the rotational part of a
// homogeneous 4x4 matrix. Translation and the bottom row are ignored.
func Mat4ToEulerAngles(m Mat4, order RotationOrder) (angle1, angle2, angle3 float64) {
	return Mat3ToEulerAngles(m.Mat3(), order)
}
