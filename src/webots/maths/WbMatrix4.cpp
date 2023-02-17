// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbMatrix4.hpp"

#include "WbMatrix3.hpp"
#include "WbRotation.hpp"

static const double IDENTITY[16] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

void WbMatrix4::setIdentity() {
  memcpy(mM, IDENTITY, sizeof(mM));
}

// Constructors

WbMatrix4::WbMatrix4(const WbVector3 &trans, const WbRotation &rot) {
  setTranslation(trans);
  setRotation(rot);
  mM[12] = 0.0;
  mM[13] = 0.0;
  mM[14] = 0.0;
  mM[15] = 1.0;
}

WbMatrix4::WbMatrix4(double tx, double ty, double tz, double rx, double ry, double rz, double angle) {
  setTranslation(tx, ty, tz);
  setRotation(rx, ry, rz, angle);
  mM[12] = 0.0;
  mM[13] = 0.0;
  mM[14] = 0.0;
  mM[15] = 1.0;
}

WbMatrix4::WbMatrix4(double tx, double ty, double tz, double rx, double ry, double rz, double angle, double sx, double sy,
                     double sz) {
  fromVrml(tx, ty, tz, rx, ry, rz, angle, sx, sy, sz);
}

WbMatrix4::WbMatrix4(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20,
                     double m21, double m22, double m23, double m30, double m31, double m32, double m33) {
  mM[0] = m00;
  mM[1] = m01;
  mM[2] = m02;
  mM[3] = m03;
  mM[4] = m10;
  mM[5] = m11;
  mM[6] = m12;
  mM[7] = m13;
  mM[8] = m20;
  mM[9] = m21;
  mM[10] = m22;
  mM[11] = m23;
  mM[12] = m30;
  mM[13] = m31;
  mM[14] = m32;
  mM[15] = m33;
}

void WbMatrix4::setRotation(const WbRotation &r) {
  setRotation(r.x(), r.y(), r.z(), r.angle());
}

void WbMatrix4::fromVrml(const WbVector3 &t, const WbRotation &r, const WbVector3 &s) {
  fromVrml(t.x(), t.y(), t.z(), r.x(), r.y(), r.z(), r.angle(), s.x(), s.y(), s.z());
}

// Multiplications

WbMatrix4 WbMatrix4::operator*(const WbMatrix4 &m) const {
  return WbMatrix4(mM[0] * m.mM[0] + mM[1] * m.mM[4] + mM[2] * m.mM[8] + mM[3] * m.mM[12],
                   mM[0] * m.mM[1] + mM[1] * m.mM[5] + mM[2] * m.mM[9] + mM[3] * m.mM[13],
                   mM[0] * m.mM[2] + mM[1] * m.mM[6] + mM[2] * m.mM[10] + mM[3] * m.mM[14],
                   mM[0] * m.mM[3] + mM[1] * m.mM[7] + mM[2] * m.mM[11] + mM[3] * m.mM[15],

                   mM[4] * m.mM[0] + mM[5] * m.mM[4] + mM[6] * m.mM[8] + mM[7] * m.mM[12],
                   mM[4] * m.mM[1] + mM[5] * m.mM[5] + mM[6] * m.mM[9] + mM[7] * m.mM[13],
                   mM[4] * m.mM[2] + mM[5] * m.mM[6] + mM[6] * m.mM[10] + mM[7] * m.mM[14],
                   mM[4] * m.mM[3] + mM[5] * m.mM[7] + mM[6] * m.mM[11] + mM[7] * m.mM[15],

                   mM[8] * m.mM[0] + mM[9] * m.mM[4] + mM[10] * m.mM[8] + mM[11] * m.mM[12],
                   mM[8] * m.mM[1] + mM[9] * m.mM[5] + mM[10] * m.mM[9] + mM[11] * m.mM[13],
                   mM[8] * m.mM[2] + mM[9] * m.mM[6] + mM[10] * m.mM[10] + mM[11] * m.mM[14],
                   mM[8] * m.mM[3] + mM[9] * m.mM[7] + mM[10] * m.mM[11] + mM[11] * m.mM[15],

                   mM[12] * m.mM[0] + mM[13] * m.mM[4] + mM[14] * m.mM[8] + mM[15] * m.mM[12],
                   mM[12] * m.mM[1] + mM[13] * m.mM[5] + mM[14] * m.mM[9] + mM[15] * m.mM[13],
                   mM[12] * m.mM[2] + mM[13] * m.mM[6] + mM[14] * m.mM[10] + mM[15] * m.mM[14],
                   mM[12] * m.mM[3] + mM[13] * m.mM[7] + mM[14] * m.mM[11] + mM[15] * m.mM[15]);
}

WbMatrix4 WbMatrix4::dotTransposed(const WbMatrix4 &m) const {
  return WbMatrix4(mM[0] * m.mM[0] + mM[1] * m.mM[1] + mM[2] * m.mM[2] + mM[3] * m.mM[3],
                   mM[0] * m.mM[4] + mM[1] * m.mM[5] + mM[2] * m.mM[6] + mM[3] * m.mM[7],
                   mM[0] * m.mM[8] + mM[1] * m.mM[9] + mM[2] * m.mM[10] + mM[3] * m.mM[11],
                   mM[0] * m.mM[12] + mM[1] * m.mM[13] + mM[2] * m.mM[14] + mM[3] * m.mM[15],

                   mM[4] * m.mM[0] + mM[5] * m.mM[1] + mM[6] * m.mM[2] + mM[7] * m.mM[3],
                   mM[4] * m.mM[4] + mM[5] * m.mM[5] + mM[6] * m.mM[6] + mM[7] * m.mM[7],
                   mM[4] * m.mM[8] + mM[5] * m.mM[9] + mM[6] * m.mM[10] + mM[7] * m.mM[11],
                   mM[4] * m.mM[12] + mM[5] * m.mM[13] + mM[6] * m.mM[14] + mM[7] * m.mM[15],

                   mM[8] * m.mM[0] + mM[9] * m.mM[1] + mM[10] * m.mM[2] + mM[11] * m.mM[3],
                   mM[8] * m.mM[4] + mM[9] * m.mM[5] + mM[10] * m.mM[6] + mM[11] * m.mM[7],
                   mM[8] * m.mM[8] + mM[9] * m.mM[9] + mM[10] * m.mM[10] + mM[11] * m.mM[11],
                   mM[8] * m.mM[12] + mM[9] * m.mM[13] + mM[10] * m.mM[14] + mM[11] * m.mM[15],

                   mM[12] * m.mM[0] + mM[13] * m.mM[1] + mM[14] * m.mM[2] + mM[15] * m.mM[3],
                   mM[12] * m.mM[4] + mM[13] * m.mM[5] + mM[14] * m.mM[6] + mM[15] * m.mM[7],
                   mM[12] * m.mM[8] + mM[13] * m.mM[9] + mM[14] * m.mM[10] + mM[15] * m.mM[11],
                   mM[12] * m.mM[12] + mM[13] * m.mM[13] + mM[14] * m.mM[14] + mM[15] * m.mM[15]);
}

WbVector4 WbMatrix4::operator*(const WbVector4 &v) const {
  return WbVector4(mM[0] * v.x() + mM[1] * v.y() + mM[2] * v.z() + mM[3] * v.w(),
                   mM[4] * v.x() + mM[5] * v.y() + mM[6] * v.z() + mM[7] * v.w(),
                   mM[8] * v.x() + mM[9] * v.y() + mM[10] * v.z() + mM[11] * v.w(),
                   mM[12] * v.x() + mM[13] * v.y() + mM[14] * v.z() + mM[15] * v.w());
}

// because we are not interested in four-dimensional coordinates, the bottom row of the matrix
// is always [0 0 0 1] and the last value in the vector is always 1
// for explanations see book: "Game Physics Engine Development"
WbVector3 WbMatrix4::operator*(const WbVector3 &v) const {
  return WbVector3(mM[0] * v.x() + mM[1] * v.y() + mM[2] * v.z() + mM[3], mM[4] * v.x() + mM[5] * v.y() + mM[6] * v.z() + mM[7],
                   mM[8] * v.x() + mM[9] * v.y() + mM[10] * v.z() + mM[11]);
}

// rotate (assuming the rotation vector is normalized)
void WbMatrix4::setRotation(double rx, double ry, double rz, double angle) {
  double c = cos(angle);
  double s = sin(angle);

  double t1 = 1.0 - c;
  double t2 = rx * rz * t1;
  double t3 = rx * ry * t1;
  double t4 = ry * rz * t1;

  mM[0] = rx * rx * t1 + c;
  mM[1] = t3 - rz * s;
  mM[2] = t2 + ry * s;

  mM[4] = t3 + rz * s;
  mM[5] = ry * ry * t1 + c;
  mM[6] = t4 - rx * s;

  mM[8] = t2 - ry * s;
  mM[9] = t4 + rx * s;
  mM[10] = rz * rz * t1 + c;
}

void WbMatrix4::transpose() {
  double t[16];
  memcpy(t, mM, sizeof(t));
  mM[1] = t[4];
  mM[2] = t[8];
  mM[3] = t[12];
  mM[4] = t[1];
  mM[6] = t[9];
  mM[7] = t[13];
  mM[8] = t[2];
  mM[9] = t[6];
  mM[11] = t[14];
  mM[12] = t[3];
  mM[13] = t[7];
  mM[14] = t[11];
}

WbMatrix4 WbMatrix4::transposed() const {
  return WbMatrix4(mM[0], mM[4], mM[8], mM[12], mM[1], mM[5], mM[9], mM[13], mM[2], mM[6], mM[10], mM[14], mM[3], mM[7], mM[11],
                   mM[15]);
}

// Source: https://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
bool WbMatrix4::inverse() {
  double inv[16], det;

  inv[0] = mM[5] * mM[10] * mM[15] - mM[5] * mM[11] * mM[14] - mM[9] * mM[6] * mM[15] + mM[9] * mM[7] * mM[14] +
           mM[13] * mM[6] * mM[11] - mM[13] * mM[7] * mM[10];

  inv[4] = -mM[4] * mM[10] * mM[15] + mM[4] * mM[11] * mM[14] + mM[8] * mM[6] * mM[15] - mM[8] * mM[7] * mM[14] -
           mM[12] * mM[6] * mM[11] + mM[12] * mM[7] * mM[10];

  inv[8] = mM[4] * mM[9] * mM[15] - mM[4] * mM[11] * mM[13] - mM[8] * mM[5] * mM[15] + mM[8] * mM[7] * mM[13] +
           mM[12] * mM[5] * mM[11] - mM[12] * mM[7] * mM[9];

  inv[12] = -mM[4] * mM[9] * mM[14] + mM[4] * mM[10] * mM[13] + mM[8] * mM[5] * mM[14] - mM[8] * mM[6] * mM[13] -
            mM[12] * mM[5] * mM[10] + mM[12] * mM[6] * mM[9];

  inv[1] = -mM[1] * mM[10] * mM[15] + mM[1] * mM[11] * mM[14] + mM[9] * mM[2] * mM[15] - mM[9] * mM[3] * mM[14] -
           mM[13] * mM[2] * mM[11] + mM[13] * mM[3] * mM[10];

  inv[5] = mM[0] * mM[10] * mM[15] - mM[0] * mM[11] * mM[14] - mM[8] * mM[2] * mM[15] + mM[8] * mM[3] * mM[14] +
           mM[12] * mM[2] * mM[11] - mM[12] * mM[3] * mM[10];

  inv[9] = -mM[0] * mM[9] * mM[15] + mM[0] * mM[11] * mM[13] + mM[8] * mM[1] * mM[15] - mM[8] * mM[3] * mM[13] -
           mM[12] * mM[1] * mM[11] + mM[12] * mM[3] * mM[9];

  inv[13] = mM[0] * mM[9] * mM[14] - mM[0] * mM[10] * mM[13] - mM[8] * mM[1] * mM[14] + mM[8] * mM[2] * mM[13] +
            mM[12] * mM[1] * mM[10] - mM[12] * mM[2] * mM[9];

  inv[2] = mM[1] * mM[6] * mM[15] - mM[1] * mM[7] * mM[14] - mM[5] * mM[2] * mM[15] + mM[5] * mM[3] * mM[14] +
           mM[13] * mM[2] * mM[7] - mM[13] * mM[3] * mM[6];

  inv[6] = -mM[0] * mM[6] * mM[15] + mM[0] * mM[7] * mM[14] + mM[4] * mM[2] * mM[15] - mM[4] * mM[3] * mM[14] -
           mM[12] * mM[2] * mM[7] + mM[12] * mM[3] * mM[6];

  inv[10] = mM[0] * mM[5] * mM[15] - mM[0] * mM[7] * mM[13] - mM[4] * mM[1] * mM[15] + mM[4] * mM[3] * mM[13] +
            mM[12] * mM[1] * mM[7] - mM[12] * mM[3] * mM[5];

  inv[14] = -mM[0] * mM[5] * mM[14] + mM[0] * mM[6] * mM[13] + mM[4] * mM[1] * mM[14] - mM[4] * mM[2] * mM[13] -
            mM[12] * mM[1] * mM[6] + mM[12] * mM[2] * mM[5];

  inv[3] = -mM[1] * mM[6] * mM[11] + mM[1] * mM[7] * mM[10] + mM[5] * mM[2] * mM[11] - mM[5] * mM[3] * mM[10] -
           mM[9] * mM[2] * mM[7] + mM[9] * mM[3] * mM[6];

  inv[7] = mM[0] * mM[6] * mM[11] - mM[0] * mM[7] * mM[10] - mM[4] * mM[2] * mM[11] + mM[4] * mM[3] * mM[10] +
           mM[8] * mM[2] * mM[7] - mM[8] * mM[3] * mM[6];

  inv[11] = -mM[0] * mM[5] * mM[11] + mM[0] * mM[7] * mM[9] + mM[4] * mM[1] * mM[11] - mM[4] * mM[3] * mM[9] -
            mM[8] * mM[1] * mM[7] + mM[8] * mM[3] * mM[5];

  inv[15] = mM[0] * mM[5] * mM[10] - mM[0] * mM[6] * mM[9] - mM[4] * mM[1] * mM[10] + mM[4] * mM[2] * mM[9] +
            mM[8] * mM[1] * mM[6] - mM[8] * mM[2] * mM[5];

  det = mM[0] * inv[0] + mM[1] * inv[4] + mM[2] * inv[8] + mM[3] * inv[12];

  if (det == 0)
    return false;

  det = 1.0 / det;

  for (int i = 0; i < 16; ++i)
    mM[i] = inv[i] * det;

  return true;
}

// These methods compute the inverse matrix provided that the original 4-by-4 matrix corresponds to a scale-free transform
// matrix
void WbMatrix4::pseudoInverse() {
  double t[16];
  memcpy(t, mM, sizeof(t));
  mM[1] = t[4];
  mM[2] = t[8];
  mM[3] = -(t[0] * t[3] + t[4] * t[7] + t[8] * t[11]);
  mM[4] = t[1];
  mM[6] = t[9];
  mM[7] = -(t[1] * t[3] + t[5] * t[7] + t[9] * t[11]);
  mM[8] = t[2];
  mM[9] = t[6];
  mM[11] = -(t[2] * t[3] + t[6] * t[7] + t[10] * t[11]);
}

WbMatrix4 WbMatrix4::pseudoInversed() const {
  return WbMatrix4(mM[0], mM[4], mM[8], -(mM[0] * mM[3] + mM[4] * mM[7] + mM[8] * mM[11]), mM[1], mM[5], mM[9],
                   -(mM[1] * mM[3] + mM[5] * mM[7] + mM[9] * mM[11]), mM[2], mM[6], mM[10],
                   -(mM[2] * mM[3] + mM[6] * mM[7] + mM[10] * mM[11]), 0.0, 0.0, 0.0, 1);
}

// Compute the product of the pseudo inverse of the matrix by v (augmented by 1)
WbVector3 WbMatrix4::pseudoInversed(const WbVector3 &v) const {
  return WbVector3(mM[0] * v.x() + mM[4] * v.y() + mM[8] * v.z() - (mM[0] * mM[3] + mM[4] * mM[7] + mM[8] * mM[11]),
                   mM[1] * v.x() + mM[5] * v.y() + mM[9] * v.z() - (mM[1] * mM[3] + mM[5] * mM[7] + mM[9] * mM[11]),
                   mM[2] * v.x() + mM[6] * v.y() + mM[10] * v.z() - (mM[2] * mM[3] + mM[6] * mM[7] + mM[10] * mM[11]));
}

void WbMatrix4::inverseTransform(double sx, double sy, double sz) {
  // We assume that this matrix is transform matrix (3D-rotation * 3D-scale + 3D-translation)
  const double x = 1.0 / (sx * sx);
  const double y = 1.0 / (sy * sy);
  const double z = 1.0 / (sz * sz);
  scale(x, y, z);
  pseudoInverse();
}

WbMatrix4 WbMatrix4::inversedTransform(double sx, double sy, double sz) const {
  // We assume that this matrix is transform matrix (3D-rotation * 3D-scale + 3D-translation)
  WbMatrix4 m(*this);
  m.inverseTransform(sx, sy, sz);
  return m;
}

WbMatrix4 WbMatrix4::inversedTransform(double scale) {
  // We assume that this matrix is transform matrix (3D-rotation * 3D-scale + 3D-translation)
  WbMatrix4 m(*this);
  if (scale != 0.0)
    m.inversedTransform(scale, scale, scale);
  else {
    // Computes scale from the matrix
    const double x = 1.0 / (mM[0] * mM[0] + mM[1] * mM[1] + mM[2] * mM[2]);
    const double y = 1.0 / (mM[4] * mM[4] + mM[5] * mM[5] + mM[6] * mM[6]);
    const double z = 1.0 / (mM[8] * mM[8] + mM[9] * mM[9] + mM[10] * mM[10]);
    m.pseudoInverse();
    m.scale(x, y, z);
  }
  return m;
}

void WbMatrix4::fromOpenGlMatrix(const double m[16]) {
  mM[0] = m[0];
  mM[1] = m[4];
  mM[2] = m[8];
  mM[3] = m[12];

  mM[4] = m[1];
  mM[5] = m[5];
  mM[6] = m[9];
  mM[7] = m[13];

  mM[8] = m[2];
  mM[9] = m[6];
  mM[10] = m[10];
  mM[11] = m[14];

  mM[12] = m[3];
  mM[13] = m[7];
  mM[14] = m[11];
  mM[15] = m[15];
}

void WbMatrix4::toOpenGlMatrix(double m[16]) const {
  m[0] = mM[0];
  m[1] = mM[4];
  m[2] = mM[8];
  m[3] = mM[12];

  m[4] = mM[1];
  m[5] = mM[5];
  m[6] = mM[9];
  m[7] = mM[13];

  m[8] = mM[2];
  m[9] = mM[6];
  m[10] = mM[10];
  m[11] = mM[14];

  m[12] = mM[3];
  m[13] = mM[7];
  m[14] = mM[11];
  m[15] = mM[15];
}

WbMatrix3 WbMatrix4::extracted3x3Matrix() const {
  return WbMatrix3(mM[0], mM[1], mM[2], mM[4], mM[5], mM[6], mM[8], mM[9], mM[10]);
}

WbVector3 WbMatrix4::sub3x3MatrixDot(const WbVector3 &v) const {
  return WbVector3(mM[0] * v.x() + mM[1] * v.y() + mM[2] * v.z(), mM[4] * v.x() + mM[5] * v.y() + mM[6] * v.z(),
                   mM[8] * v.x() + mM[9] * v.y() + mM[10] * v.z());
}

void WbMatrix4::extract3x4Matrix(double m[12]) const {
  memcpy(m, mM, 11 * sizeof(double));
  m[3] = 0.0;
  m[7] = 0.0;
  m[11] = 0.0;
}

void WbMatrix4::extract3x4Matrix(double m[12], double scale) const {
  memcpy(m, mM, 11 * sizeof(double));
  m[0] *= scale;
  m[1] *= scale;
  m[2] *= scale;
  m[4] *= scale;
  m[5] *= scale;
  m[6] *= scale;
  m[8] *= scale;
  m[9] *= scale;
  m[10] *= scale;
  m[3] = 0.0;
  m[7] = 0.0;
  m[11] = 0.0;
}

WbQuaternion WbMatrix4::extractedQuaternion(double x, double y, double z) const {
  // Rescale the 3-by-3 submatrix
  const double m0 = mM[0] * x;
  const double m4 = mM[4] * x;
  const double m8 = mM[8] * x;

  const double m1 = mM[1] * y;
  const double m5 = mM[5] * y;
  const double m9 = mM[9] * y;

  const double m2 = mM[2] * z;
  const double m6 = mM[6] * z;
  const double m10 = mM[10] * z;

  // Now, the 3-by-3 submatrix should be orthogonal (otherwise there is no guarantee to obtain a normalized quaternion)
  if (m0 == m5 && m5 == m10 && m10 == 1.0) {
    // zero angle case
    return WbQuaternion();
  }

  double s = 2.0, invS = 1.0;
  const double trace = m0 + m5 + m10;
  if (trace >= 0.0) {      //
    s *= sqrt(trace + 1);  // we divide by s = 4w, which is large enough
    invS = 1.0 / s;
    return WbQuaternion(0.25 * s, (m9 - m6) * invS, (m2 - m8) * invS, (m4 - m1) * invS);
  }

  if (m0 > m5 && m0 > m10) {         // M[0] (equivalently x) is the largest
    s *= sqrt(1.0 + m0 - m5 - m10);  // s = 4x
    invS = 1.0 / s;
    return WbQuaternion((m9 - m6) * invS, 0.25 * s, (m1 + m4) * invS, (m8 + m2) * invS);
  } else if (m5 > m10) {             // M[5] (equivalently y) is the largest
    s *= sqrt(1.0 - m0 + m5 - m10);  // s = 4y
    invS = 1.0 / s;
    return WbQuaternion((m2 - m8) * invS, (m1 + m4) * invS, 0.25 * s, (m6 + m9) * invS);
  } else {                           // M[10] (equivalently z) is the largest
    s *= sqrt(1.0 - m0 - m5 + m10);  // s = 4z
    invS = 1.0 / s;
    return WbQuaternion((m4 - m1) * invS, (m8 + m2) * invS, (m6 + m9) * invS, 0.25 * s);
  }
}
