/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Math-related util functions.
 */

//#pragma once
#ifndef COMMON__MATH__MATH_UTILS_HPP_
#define COMMON__MATH__MATH_UTILS_HPP_

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>
#include <array>

#include "vec2d.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace common {
namespace math {

double Sqr(const double x);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Calculate the difference between angle from and to
 * @param from the start angle
 * @param from the end angle
 * @return The difference between from and to. The range is between [-PI, PI).
 */
double AngleDiff(const double from, const double to);

/**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

// Gaussian
double Gaussian(const double u, const double std, const double x);

inline double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

// Rotate a 2d vector counter-clockwise by theta
Vec2d RotateVector2d(const Vec2d &v_in, const double theta);

inline std::pair<double, double> RFUToFLU(const double x, const double y) {
  return std::make_pair(y, -x);
}

inline std::pair<double, double> FLUToRFU(const double x, const double y) {
  return std::make_pair(-y, x);
}

inline void L2Norm(int feat_dim, float *feat_data) {
  if (feat_dim == 0) {
    return;
  }
  // feature normalization
  float l2norm = 0.0f;
  for (int i = 0; i < feat_dim; ++i) {
    l2norm += feat_data[i] * feat_data[i];
  }
  if (l2norm == 0) {
    float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] = val;
    }
  } else {
    l2norm = std::sqrt(l2norm);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] /= l2norm;
    }
  }
}

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

inline double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  // unless the result is subnormal
  return std::fabs(x - y) <=
         std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
         std::fabs(x - y) < std::numeric_limits<T>::min();
}

std::vector<double> ToContinuousAngle(const std::vector<double> &angle);


template<int N>
inline std::array<double, N> LinSpaced(double start, double end) {
  std::array<double, N> res;
  double step = (end - start) / (N - 1);

  for(int i = 0; i < N; i++) {
    res[i] = start + step * i;
  }

  return res;
}

inline std::vector<double> LinSpaced(double start, double end, int count) {
  std::vector<double> res(count, 0);
  double step = (end - start) / (count - 1);

  for(int i = 0; i < count; i++) {
    res[i] = start + step * i;
  }

  return res;
}

inline double Interpolate1d(const std::vector<double> &x, const std::vector<double> &y, double t) {
  if(t >= x.back()) {
    return y.back();
  }

  auto index = std::distance(x.begin(), std::lower_bound(x.begin(), x.end(), t));

  if(index == 0) {
    return y.front();
  }

  return lerp(y[index - 1], x[index - 1], y[index], x[index], t);
}

inline std::vector<double> Interpolate1d(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &t) {
  std::vector<double> result(t.size());
  result.reserve(t.size());
  for(size_t i = 0; i < t.size(); i++) {
    result[i] = Interpolate1d(x, y, t[i]);
  }
  return result;
}

inline std::vector<double> ARange(double start, double end, double step) {
  std::vector<double> res;

  for(int i = 0; ; i++) {
    auto val = start + step * i;
    if(val > end) {
      break;
    }
    res.push_back(val);
  }

  return res;
}

/**
 * @brief Compute cubic B-spline control points for interpolation.
 * Given n+1 data points y[0..n], returns n+3 control points.
 * The curve is C² continuous and passes through all data points.
 *
 * Control point layout (size n+3): P[0]=P_{-1}, P[1]=P_0, ..., P[n+1]=P_n, P[n+2]=P_{n+1}
 * At integer t=k: C(k) = (P[k] + 4*P[k+1] + P[k+2]) / 6 = D_k = y[k]
 */
inline std::vector<double> CubicBSplineControlPoints(const std::vector<double>& y) {
  int n = static_cast<int>(y.size()) - 1;
  if (n < 0) return y;

  std::vector<double> P(n + 3);

  // Boundary: P_0 = D_0, P_n = D_n (from natural BC)
  P[1] = y[0];      // P_0
  P[n + 1] = y[n];  // P_n

  int m = n - 1;  // number of unknowns: P_1 ... P_{n-1}
  if (m <= 0) {
    // n <= 1: only 1-2 data points, no interior unknowns
    if (n == 0) {
      P[0] = y[0]; P[2] = y[0];
    } else {  // n == 1
      P[2] = y[1];
      P[0] = 2.0 * P[1] - P[2];       // P_{-1}
      P[3] = 2.0 * P[2] - P[1];       // P_{2}
    }
    return P;
  }

  // Thomas algorithm for tridiagonal system A*x = d of size m
  // a[j]=1, b[j]=4, c[j]=1
  // x[j] = P[j+2] (i.e., x[0]=P[2]=P_1, ..., x[m-1]=P[m+1]=P_{n-1})
  // rhs[j]: 6*y[1]-y[0] for j=0; 6*y[j+1] for j=1..m-2; 6*y[n-1]-y[n] for j=m-1
  std::vector<double> cp(m);  // c'_j
  std::vector<double> dp(m);  // d'_j

  cp[0] = 1.0 / 4.0;
  dp[0] = (6.0 * y[1] - y[0]) / 4.0;

  for (int j = 1; j < m - 1; j++) {
    double w = 1.0 / (4.0 - cp[j - 1]);
    cp[j] = w;
    dp[j] = (6.0 * y[j + 1] - dp[j - 1]) * w;
  }

  if (m > 1) {
    double w = 1.0 / (4.0 - cp[m - 2]);
    cp[m - 1] = 0.0;
    dp[m - 1] = (6.0 * y[n - 1] - y[n] - dp[m - 2]) * w;
  } else {
    // m == 1: single equation is both first and last
    dp[0] = (6.0 * y[1] - y[0] - y[n]) / 4.0;
  }

  // Backward substitution
  P[n] = dp[m - 1];  // P_{n-1} = x[m-1]
  for (int j = m - 2; j >= 0; j--) {
    P[j + 2] = dp[j] - cp[j] * P[j + 3];
  }

  // Natural boundary extrapolation
  P[0] = 2.0 * P[1] - P[2];           // P_{-1}
  P[n + 2] = 2.0 * P[n + 1] - P[n];   // P_{n+1}

  return P;
}

/**
 * @brief Evaluate cubic B-spline at parameter t.
 * @param t Parameter value in [0, n] where n = control_points.size() - 3.
 * @param P Control points including boundary points (size n+3).
 * @return Spline value at t.
 */
inline double EvaluateCubicBSpline(double t, const std::vector<double>& P) {
  int n = static_cast<int>(P.size()) - 3;  // number of segments
  if (n <= 0) return P[0];

  t = std::max(0.0, std::min(static_cast<double>(n), t));
  int i = static_cast<int>(std::floor(t));
  if (i >= n) i = n - 1;
  double u = t - i;

  double u2 = u * u;
  double u3 = u2 * u;
  double one_u = 1.0 - u;
  double one_u2 = one_u * one_u;
  double one_u3 = one_u2 * one_u;

  // N[-1] = (1-u)³/6, N[0] = (3u³-6u²+4)/6, N[1] = (-3u³+3u²+3u+1)/6, N[2] = u³/6
  return (one_u3 * P[i] +
          (3.0 * u3 - 6.0 * u2 + 4.0) * P[i + 1] +
          (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) * P[i + 2] +
          u3 * P[i + 3]) / 6.0;
}

/**
 * @brief Evaluate cubic B-spline first derivative at parameter t.
 */
inline double EvaluateCubicBSplineDerivative(double t, const std::vector<double>& P) {
  int n = static_cast<int>(P.size()) - 3;
  if (n <= 0) return 0.0;

  t = std::max(0.0, std::min(static_cast<double>(n), t));
  int i = static_cast<int>(std::floor(t));
  if (i >= n) i = n - 1;
  double u = t - i;

  double u2 = u * u;
  double one_u = 1.0 - u;
  double one_u2 = one_u * one_u;

  // N'[-1] = -(1-u)²/2, N'[0] = (3u²-4u)/2, N'[1] = (-3u²+2u+1)/2, N'[2] = u²/2
  return (-one_u2 * P[i] +
          (3.0 * u2 - 4.0 * u) * P[i + 1] +
          (-3.0 * u2 + 2.0 * u + 1.0) * P[i + 2] +
          u2 * P[i + 3]) / 2.0;
}

/**
 * @brief Evaluate cubic B-spline at multiple parameter values.
 * @param t_vals Query parameter values (each in [0, n]).
 * @param P Control points from CubicBSplineControlPoints.
 * @return Interpolated values at each t_val.
 */
inline std::vector<double> EvaluateCubicBSpline(
    const std::vector<double>& t_vals, const std::vector<double>& P) {
  std::vector<double> result;
  result.reserve(t_vals.size());
  for (double t : t_vals) {
    result.push_back(EvaluateCubicBSpline(t, P));
  }
  return result;
}

}  // namespace math
}  // namespace common
#endif
