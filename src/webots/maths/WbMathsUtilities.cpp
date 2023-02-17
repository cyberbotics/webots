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

#include "WbMathsUtilities.hpp"
#include "WbPolygon.hpp"
#include "WbVector2.hpp"
#include "WbVector3.hpp"

using namespace std;

namespace {

/////////////////////////////////////////////////////////////////////////////////
//  Graham scan algorithm: find the convex hull of set of points in the plane  //
/////////////////////////////////////////////////////////////////////////////////

// There are two phases:
//(1) Sort
//(2) Backtracking to remove all right turns

// (1) The indices of a list of points are sorted with respect to both their polar angles and distances to the first point
// (anchor) Although quadratic on average, insertion sort is the fastest algorithms for sorting very small arrays.

// The following macro describes the order used on points in the plane during the sort phasis of the scan: first we compare the
// angles, then the distances to the anchor The threshold is important as it avoids false negatives when points are closed to be
// aligned; false positives can be removed in a second step
#define ORDER(k, l)                                                   \
  ((cosinus.at(indices.at(k)) < cosinus.at(l) - COSINUS_THRESHOLD) || \
   (cosinus.at(indices.at(k)) < cosinus.at(l) + COSINUS_THRESHOLD && distance.at(indices.at(k)) > distance.at(l)))

  void straightInsertionSort(QVector<int> &indices, const QVector<double> &cosinus, const QVector<double> &distance) {
    static const double COSINUS_THRESHOLD = 1e-6;
    const int size = indices.size();
    int i;
    for (i = 2; i < size; ++i) {
      int j = i;
      int temp = indices[i];
      while (j > 1 && ORDER(j - 1, temp)) {
        indices[j] = indices.at(j - 1);
        --j;
      }
      indices[j] = temp;
    }
  }

  // Tests whether the elbow [A, B] union [B, C] turns right or not, i.e., whether the oriented angle (BC, BA) is positive or
  // not
  bool rightTurn(const WbVector2 &A, const WbVector2 &B, const WbVector2 &C) {
    const double determinant_BC_BA = (C.x() - B.x()) * (A.y() - B.y()) - (C.y() - B.y()) * (A.x() - B.x());
    if (determinant_BC_BA > 0.0)
      return false;  // it turns left

    return true;  // it turns right or the three points are aligned
  }
};  // namespace

bool WbMathsUtilities::isPowerOf2(unsigned int n) {
  return (n != 0) && ((n & (n - 1)) == 0);
}

unsigned int WbMathsUtilities::nextPowerOf2(unsigned int n) {
  n--;
  n |= n >> 1;
  n |= n >> 2;
  n |= n >> 4;
  n |= n >> 8;
  n |= n >> 16;
  n++;
  return n;
}

int WbMathsUtilities::round(double n) {
  return floor(n + 0.5);
}

//////////////////////////////////////////////////
// Graham scan algorithm adapted from Sedgewick //
//////////////////////////////////////////////////

// The method extracts a list of indices corresponding to the extremal points of the convex hull of the provided list (the
// y-coordinate is ignored) and returns the size of the convex hull. Note: we suppose that points.size() >= 4;
int WbMathsUtilities::convexHull(const QVector<WbVector2> &points, QVector<int> &hullIndices) {
  const int size = points.size();  // size >= 4
  double minY = points.at(0).y();
  int i = 0, iMin = 0;

  for (i = 1; i < size; ++i) {
    if (points.at(i).y() < minY) {
      minY = points.at(i).y();
      iMin = i;
    }
  }

  for (i = 0; i < size; ++i) {
    if (points.at(i).y() == minY && points.at(i).x() >= points.at(iMin).x())
      iMin = i;
  }

  QVector<int> index(size);
  for (i = 0; i < size; ++i)
    index[i] = i;

  index[0] = iMin;
  index[iMin] = 0;

  QVector<double> cosinus(size);
  QVector<double> distance(size);
  // Computes the cosinus of the angles (anchor, points[i], positive x-axis unit vector)
  for (i = 1; i < size; ++i) {
    const double x = points.at(index.at(i)).x() - points.at(index.at(0)).x();
    const double y = points.at(index.at(i)).y() - points.at(index.at(0)).y();
    distance[index.at(i)] = sqrt(x * x + y * y);
    cosinus[index.at(i)] = x / distance.at(index.at(i));
  }
  // Sorts all points with respect to increasing angles
  straightInsertionSort(index, cosinus, distance);
  hullIndices[0] = index.at(0);
  int k = 2;
  const int sizeMinusOne = size - 1;
  // Finds the first three non-aligned points in the convex hull (including points[index[1]])
  while (k < sizeMinusOne && cosinus.at(index.at(1)) == cosinus.at(index.at(k)))
    ++k;
  // All points are aligned: exits
  if (k == sizeMinusOne) {
    hullIndices[1] = index.at(k);
    return 2;
  }
  hullIndices[1] = index.at(k - 1);
  hullIndices[2] = index.at(k);
  int M = 2;

  for (i = k + 1; i < size; ++i) {
    while (M >= 1 && rightTurn(points.at(hullIndices.at(M - 1)), points.at(hullIndices.at(M)), points.at(index.at(i))))
      --M;

    ++M;
    hullIndices[M] = index.at(i);
  }
  return M + 1;
}

int WbMathsUtilities::twoStepsConvexHull(const QVector<WbVector2> &points, QVector<int> &hullIndices) {
  QVector<int> filteredIndices(points.size());
  // A first 'filtering' Graham scan
  const int size = convexHull(points, filteredIndices);
  if (size < 5) {
    for (int i = 0; i < size; ++i)
      hullIndices[i] = filteredIndices.at(i);
    return size;
  }

  QVector<WbVector2> rotatedPoints(size);
  // We rotate the convex hull of +pi/2 before performing another Graham scan
  for (int i = 0; i < size; ++i) {
    const WbVector2 &v = points.at(filteredIndices.at(i));
    rotatedPoints[i].setXy(v.y(), -v.x());
  }
  QVector<int> indices(size);
  const int hullSize = convexHull(rotatedPoints, indices);
  for (int i = 0; i < hullSize; ++i)
    hullIndices[i] = filteredIndices.at(indices.at(i));

  return hullSize;
}

bool WbMathsUtilities::isConvex(const WbPolygon &p) {
  const int sizeMinusOne = p.actualSize() - 1;
  if (sizeMinusOne < 3)
    return true;
  for (int i = 1; i < sizeMinusOne; ++i) {
    if (rightTurn(p.at(i - 1), p.at(i), p.at(i + 1)))
      return false;
  }
  if (rightTurn(p.at(sizeMinusOne - 1), p.at(sizeMinusOne), p.at(0)))
    return false;
  return true;
}

/////////////////
// orthoBasis  //
/////////////////

// Returns an orhtonormal basis (b[X], b[Y] = vY.normalized(), b[Z])
void WbMathsUtilities::orthoBasis(const WbVector3 &vY, WbVector3 b[3]) {
  b[Y] = vY.normalized();
  const double x = fabs(vY.x());
  const double y = fabs(vY.y());
  const double z = fabs(vY.z());
  // We choose the two largest coordinates of vY to build an orthogonal unit vector
  if (x > y) {
    if (y > z)
      b[Z].setXyz(vY.y(), -vY.x(), 0.0);  // x > y > z
    else
      b[Z].setXyz(vY.z(), 0.0, -vY.x());  // x > z > y or z > x > y
  } else if (x < z)                       // x <= y
    b[Z].setXyz(0.0, vY.z(), -vY.y());    // y >= z > x or z > y >= x
  else
    b[Z].setXyz(vY.y(), -vY.x(), 0.0);  // y >= x >= z

  b[Z].normalize();
  b[X] = b[Y].cross(b[Z]);
}

/////////////////////
// Zero angle test //
/////////////////////

bool WbMathsUtilities::isZeroAngle(double angle) {
  static const double TWO_PI = 2.0 * M_PI;
  static const double ZERO_ANGLE_THRESHOLD = 1e-10;
  double ratio;
  const double reminder = modf(angle / TWO_PI, &ratio);
  if (fabs(reminder) < ZERO_ANGLE_THRESHOLD)
    return true;

  return false;
}

/////////////////////
// Clamping angles //
/////////////////////

void WbMathsUtilities::clampAngles(double &min, double &max) {
  normalizeAngle(min);
  normalizeAngle(max);
  if (min > max) {  // swap
    const double m = max;
    max = min;
    min = m;
  }
}

/////////////
// Vectors //
/////////////

bool WbMathsUtilities::isZeroVector3(const double *v) {
  return v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0;
}

void WbMathsUtilities::printVector3(const char *str, const double *v) {
  printf("%s %g %g %g\n", str, v[0], v[1], v[2]);
}

void WbMathsUtilities::printMatrix3x4(const char *str, const double *m) {
  printf("%s:\n", str);
  printf("%g %g %g %g\n", m[0], m[1], m[2], m[3]);
  printf("%g %g %g %g\n", m[4], m[5], m[6], m[7]);
  printf("%g %g %g %g\n", m[8], m[9], m[10], m[11]);
}

////////////////////////////////////////////////////////////////////////
// find rational approximation to given real number
// David Eppstein / UC Irvine / 8 Aug 1993
//
// With corrections from Arno Formella, May 2008
//
// based on the theory of continued fractions
// if x = a1 + 1/(a2 + 1/(a3 + 1/(a4 + ...)))
// then best approximation is found by truncating this series
// (with some adjustments in the last term).
//
// Note the fraction can be recovered as the first column of the matrix
//  ( a1 1 ) ( a2 1 ) ( a3 1 ) ...
//  ( 1  0 ) ( 1  0 ) ( 1  0 )
// Instead of keeping the sequence of continued fraction terms,
// we just keep the last partial product of these matrices.
////////////////////////////////////////////////////////////////////////
bool WbMathsUtilities::computeRationalApproximation(double value, int maxDenominator, int &numerator, int &denominator) {
  double x = value;
  long ai = 0;

  // initialize matrix
  long m[2][2];
  m[0][0] = m[1][1] = 1;
  m[0][1] = m[1][0] = 0;

  // loop finding terms until denominator gets too big
  while ((m[1][0] * (ai = (long)x) + m[1][1]) <= maxDenominator) {
    long t = m[0][0] * ai + m[0][1];
    m[0][1] = m[0][0];
    m[0][0] = t;

    t = m[1][0] * ai + m[1][1];
    m[1][1] = m[1][0];
    m[1][0] = t;

    // detect division by zero
    if (x == (double)ai)
      break;

    x = 1 / (x - (double)ai);

    // detect failure;
    if (x > 0x7FFFFFFF)
      break;
  }

  // approximate remaining to 0
  double error = value - ((double)m[0][0]) / ((double)m[1][0]);
  if (error < 1e-8) {
    numerator = m[0][0];
    denominator = m[1][0];
    return true;
  }

  // approximate remaining to 1/ai
  ai = (maxDenominator - m[1][1]) / m[1][0];
  m[0][0] = m[0][0] * ai + m[0][1];
  m[1][0] = m[1][0] * ai + m[1][1];
  error = value - ((double)m[0][0]) / ((double)m[1][0]);
  if (error < 1e-8) {
    numerator = m[0][0];
    denominator = m[1][0];
    return true;
  }

  return false;
}
