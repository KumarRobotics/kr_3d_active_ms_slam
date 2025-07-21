// Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace eigen_checks {
namespace internal {
constexpr double kDefaultPrecision = 1e-10;

template <typename LeftMat, typename RightMat>
::testing::AssertionResult MatricesEqual(const LeftMat &A,
                                         const std::string &name_lhs,
                                         const RightMat &B,
                                         const std::string &name_rhs,
                                         double threshold,
                                         const std::string &name_threshold) {
  if (A.rows() != B.rows() || A.cols() != B.cols()) {
    return ::testing::AssertionFailure()
           << "Matrix size mismatch: " << A.rows() << "x" << A.cols() << " ("
           << name_lhs << ") != " << B.rows() << "x" << B.cols() << " ("
           << name_rhs << ")";
  }

  bool success = true;
  std::string message;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      const auto Aij = A(i, j);
      const auto Bij = B(i, j);
      if (std::abs(Aij - Bij) > threshold) {
        success = false;
        message += "\n  Mismatch at [" + std::to_string(i) + "," +
                   std::to_string(j) + "] : " + std::to_string(Aij) +
                   " != " + std::to_string(Bij) + " (" + name_threshold +
                   " = " + std::to_string(threshold) + ")";
      }
    }
  }

  return success ? ::testing::AssertionSuccess()
                 : ::testing::AssertionFailure() << message << std::endl;
}

template <typename LeftMat, typename RightMat, class BinaryOp>
::testing::AssertionResult MatricesRelation(
    const LeftMat &A, const std::string &name_lhs, const RightMat &B,
    const std::string &name_rhs, BinaryOp op, const std::string &name_op) {
  if (A.rows() != B.rows() || A.cols() != B.cols()) {
    return ::testing::AssertionFailure()
           << "Matrix size mismatch: " << A.rows() << "x" << A.cols() << " ("
           << name_lhs << ") != " << B.rows() << "x" << B.cols() << " ("
           << name_rhs << ")";
  }

  bool success = true;
  std::string message;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      const auto Aij = A(i, j);
      const auto Bij = B(i, j);
      if (!op(Aij, Bij)) {
        success = false;
        message += "\n  Mismatch at [" + std::to_string(i) + "," +
                   std::to_string(j) + "] : " + std::to_string(Aij) + " " +
                   name_op + " " + std::to_string(Bij);
      }
    }
  }

  return success ? ::testing::AssertionSuccess()
                 : ::testing::AssertionFailure() << message << std::endl;
}

template <typename LeftMat, typename RightMat>
::testing::AssertionResult MatricesNear(
    const Eigen::MatrixBase<LeftMat> &lhs, const std::string &name_lhs,
    const Eigen::MatrixBase<RightMat> &rhs, const std::string &name_rhs,
    typename Eigen::MatrixBase<LeftMat>::Scalar tolerance,
    const std::string &name_tolerance) {
  if (lhs.rows() != rhs.rows()) {
    ::testing::AssertionResult failure_reason(false);
    failure_reason << "The matrices have a different number of rows: "
                   << name_lhs << " has " << lhs.rows() << " rows while "
                   << name_rhs << " has " << rhs.rows() << " rows.\n";
    return failure_reason;
  }
  if (lhs.cols() != rhs.cols()) {
    ::testing::AssertionResult failure_reason(false);
    failure_reason << "The matrices have a different number of cols: "
                   << name_lhs << " has " << lhs.cols() << " cols while "
                   << name_rhs << " cols " << rhs.cols() << " cols.\n";
    failure_reason << name_lhs << ":\n" << lhs << "\n";
    failure_reason << name_rhs << ":\n" << rhs << "\n";
    return failure_reason;
  }

  // Early exit for dynamic-sized matrices where one dimension is zero. No need
  // to check values...
  if (rhs.rows() == 0 || rhs.cols() == 0 || lhs.rows() == 0 ||
      lhs.cols() == 0) {
    return ::testing::AssertionSuccess();
  }

  using Scalar = typename Eigen::MatrixBase<LeftMat>::Scalar;
  const Scalar max_diff = (lhs - rhs).cwiseAbs().maxCoeff();

  if (max_diff <= tolerance) {
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult failure_reason(false);
  failure_reason << "The matrices are different. The maximum difference "
                 << "between " << name_lhs << " and " << name_rhs << " is "
                 << max_diff << ", which exceeds " << tolerance << ", where\n";
  for (int i = 0; i < lhs.rows(); ++i) {
    for (int j = 0; j < lhs.cols(); ++j) {
      const Scalar lij = lhs(i, j);
      const Scalar rij = rhs(i, j);
      const auto diff = std::abs(lij - rij);
      if (!std::isfinite(lij) || !std::isfinite(rij) || diff > tolerance) {
        if (lhs.rows() == 1) {
          failure_reason << "\nposition " << j << " evaluates to " << lij
                         << " and " << rij;
        } else if (lhs.cols() == 1) {
          failure_reason << "\nposition " << i << " evaluates to " << lij
                         << " and " << rij;
        } else {
          failure_reason << "\nposition " << i << "," << j << " evaluates to "
                         << lij << " and " << rij;
        }
        failure_reason << " with a tolerance of " << name_tolerance << ".\n";
      }
    }
  }
  failure_reason << name_lhs << ":\n" << lhs << "\n";
  failure_reason << name_rhs << ":\n" << rhs << "\n";
  failure_reason << "Difference:\n" << (lhs - rhs) << "\n";
  return failure_reason;
}

template <typename Matrix>
::testing::AssertionResult MatrixZero(
    const Eigen::MatrixBase<Matrix> &lhs, const std::string &name_lhs,
    typename Eigen::MatrixBase<Matrix>::Scalar tolerance,
    const std::string &name_tolerance) {
  if (lhs.isZero(tolerance)) {
    return ::testing::AssertionSuccess();
  }
  // Make a copy to get the same size matrix even in the dynamic size case.
  Eigen::Matrix<typename Eigen::MatrixBase<Matrix>::Scalar, Eigen::Dynamic,
                Eigen::Dynamic>
      zero;
  zero.setZero(lhs.rows(), lhs.cols());
  ::testing::AssertionResult failure_reason =
      MatricesNear(lhs, name_lhs, zero, "Zero", tolerance, name_tolerance);
  CHECK_EQ(false, static_cast<bool>(failure_reason));
  return failure_reason;
}

}  // namespace internal
}  // namespace eigen_checks

// -----------------------------------------------------------------------------
#include <limits>
#include <type_traits>

#define EIGEN_MATRIX_EQUAL(MatrixA, MatrixB)                                \
  eigen_checks::internal::MatricesEqual(                                    \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                 \
      static_cast<                                                          \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>( \
          0.0),                                                             \
      "0.0")

#define EIGEN_MATRIX_LT(MatrixA, MatrixB)                                     \
  eigen_checks::internal::MatricesRelation(                                   \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                   \
      std::less<                                                              \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>(), \
      "<")

#define EIGEN_MATRIX_LE(MatrixA, MatrixB)                                     \
  eigen_checks::internal::MatricesRelation(                                   \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                   \
      std::less_equal<                                                        \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>(), \
      "<=")

#define EIGEN_MATRIX_GT(MatrixA, MatrixB)                                     \
  eigen_checks::internal::MatricesRelation(                                   \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                   \
      std::greater<                                                           \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>(), \
      ">")

#define EIGEN_MATRIX_GE(MatrixA, MatrixB)                                     \
  eigen_checks::internal::MatricesRelation(                                   \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                   \
      std::greater_equal<                                                     \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>(), \
      ">=")

#define EIGEN_MATRIX_EQUAL_DOUBLE(MatrixA, MatrixB)                         \
  eigen_checks::internal::MatricesEqual(                                    \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                 \
      static_cast<                                                          \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>( \
          eigen_checks::internal::kDefaultPrecision),                       \
      "Floating point precision")

#define EIGEN_MATRIX_NEAR(MatrixA, MatrixB, Precision)                       \
  eigen_checks::internal::MatricesNear(MatrixA, #MatrixA, MatrixB, #MatrixB, \
                                       Precision, #Precision)

#define EIGEN_MATRIX_ZERO(MatrixA, Precision) \
  eigen_checks::internal::MatrixZero(MatrixA, #MatrixA, Precision, #Precision)

// -----------------------------------------------------------------------------
// To allow the user to append custom messages we store the result of the
// verification inside a unique local variable to be used in the check and for
// outputting the message.
#define INTERNAL_EIGEN_CHECKS_LOCAL_UNIQUE_VAR(var, line) var##line
#define INTERNAL_EIGEN_CHECKS_MAKE_VAR(var, line) \
  INTERNAL_EIGEN_CHECKS_LOCAL_UNIQUE_VAR(var, line)

#define INTERNAL_EIGEN_CHECKS_MAKE_GLOG_CHECK(X)                              \
  ::testing::AssertionResult INTERNAL_EIGEN_CHECKS_MAKE_VAR(result_,          \
                                                            __LINE__) = (X);  \
  CHECK(static_cast<bool>(INTERNAL_EIGEN_CHECKS_MAKE_VAR(result_, __LINE__))) \
      << (INTERNAL_EIGEN_CHECKS_MAKE_VAR(result_, __LINE__)).message()

#define CHECK_EIGEN_MATRIX_EQUAL(MatrixA, MatrixB)                            \
  INTERNAL_EIGEN_CHECKS_MAKE_GLOG_CHECK(eigen_checks::internal::MatricesNear( \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                   \
      static_cast<                                                            \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>(   \
          0.0),                                                               \
      "0.0"))

#define CHECK_EIGEN_MATRIX_EQUAL_DOUBLE(MatrixA, MatrixB)                     \
  INTERNAL_EIGEN_CHECKS_MAKE_GLOG_CHECK(eigen_checks::internal::MatricesNear( \
      MatrixA, #MatrixA, MatrixB, #MatrixB,                                   \
      static_cast<                                                            \
          typename std::remove_reference<decltype(MatrixA)>::type::Scalar>(   \
          eigen_checks::internal::kDefaultPrecision),                         \
      "Floating point precision"))

#define CHECK_EIGEN_MATRIX_NEAR(MatrixA, MatrixB, Precision)                  \
  INTERNAL_EIGEN_CHECKS_MAKE_GLOG_CHECK(eigen_checks::internal::MatricesNear( \
      MatrixA, #MatrixA, MatrixB, #MatrixB, Precision, #Precision))

#define CHECK_EIGEN_MATRIX_ZERO(MatrixA, Precision)                         \
  INTERNAL_EIGEN_CHECKS_MAKE_GLOG_CHECK(eigen_checks::internal::MatrixZero( \
      MatrixA, #MatrixA, Precision, #Precision))
