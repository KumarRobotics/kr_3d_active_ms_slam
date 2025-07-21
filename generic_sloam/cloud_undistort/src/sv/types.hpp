#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <array>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <memory>
#include <type_traits>
#include <vector>

namespace sv {

// Real value
using real_t = double;

// Id type
using id_t = std::size_t;

// optional
template <typename T>
using Optional = boost::optional<T>;
using boost::make_optional;

// scoped_ptr
using boost::scoped_ptr;

// template <typename T>
// using Eye_t = typename Eigen::MatrixBase<T>::IdentityReturnType;
// template <typename T>
// using Const_t = typename Eigen::MatrixBase<T>::ConstantReturnType;

//------------------------------------------------------------------------------
// Typedefs of commonly used Eigen matrices and vectors.

// MatrixMN, MatrixN = MatrixNN, I_NxN, and Z_NxN, for M,N=1..9.
#define SV_MAKE_EIGEN_MATRIX_TYPEDEFS(SIZE, SUFFIX)                      \
  using Matrix##SUFFIX = Eigen::Matrix<real_t, SIZE, SIZE>;              \
  using Matrix1##SUFFIX = Eigen::Matrix<real_t, 1, SIZE>;                \
  using Matrix2##SUFFIX = Eigen::Matrix<real_t, 2, SIZE>;                \
  using Matrix3##SUFFIX = Eigen::Matrix<real_t, 3, SIZE>;                \
  using Matrix4##SUFFIX = Eigen::Matrix<real_t, 4, SIZE>;                \
  using Matrix5##SUFFIX = Eigen::Matrix<real_t, 5, SIZE>;                \
  using Matrix6##SUFFIX = Eigen::Matrix<real_t, 6, SIZE>;                \
  using Matrix7##SUFFIX = Eigen::Matrix<real_t, 7, SIZE>;                \
  using Matrix8##SUFFIX = Eigen::Matrix<real_t, 8, SIZE>;                \
  using Matrix9##SUFFIX = Eigen::Matrix<real_t, 9, SIZE>;                \
  using Matrix##SUFFIX##X = Eigen::Matrix<real_t, SIZE, Eigen::Dynamic>; \
  using MatrixX##SUFFIX = Eigen::Matrix<real_t, Eigen::Dynamic, SIZE>;   \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::IdentityReturnType     \
      I_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Identity();                \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::ConstantReturnType     \
      Z_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Zero()

SV_MAKE_EIGEN_MATRIX_TYPEDEFS(1, 1);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(2, 2);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(3, 3);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(4, 4);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(5, 5);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(6, 6);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(7, 7);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(8, 8);
SV_MAKE_EIGEN_MATRIX_TYPEDEFS(9, 9);

// Dynamic length vectors and matrices
using VectorX = Eigen::Matrix<real_t, Eigen::Dynamic, 1>;
using RowVecX = Eigen::Matrix<real_t, 1, Eigen::Dynamic>;
using VectorXi = Eigen::VectorXi;
using MatrixXXi = Eigen::MatrixXi;
using MatrixXX = Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic>;

template <int N>
using MatrixNX = Eigen::Matrix<real_t, N, Eigen::Dynamic>;
template <int N>
using MatrixXN = Eigen::Matrix<real_t, Eigen::Dynamic, N>;
template <int N>
using MatrixN = Eigen::Matrix<real_t, N, N>;
template <int M, int N>
using MatrixMN = Eigen::Matrix<real_t, M, N>;
template <int N>
using VectorN = Eigen::Matrix<real_t, N, 1>;

template <typename T>
using MatrixXXT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
template <typename T, int N>
using VectorNT = Eigen::Matrix<T, N, 1>;
template <typename T>
using VectorXT = Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Commonly used fixed size vectors.
using Vector1 = Eigen::Matrix<real_t, 1, 1>;
using Vector2 = Eigen::Matrix<real_t, 2, 1>;
using Vector3 = Eigen::Matrix<real_t, 3, 1>;
using Vector4 = Eigen::Matrix<real_t, 4, 1>;
using Vector5 = Eigen::Matrix<real_t, 5, 1>;
using Vector6 = Eigen::Matrix<real_t, 6, 1>;
using Vector7 = Eigen::Matrix<real_t, 7, 1>;
using Vector8 = Eigen::Matrix<real_t, 8, 1>;
using Vector9 = Eigen::Matrix<real_t, 9, 1>;
using Vector2i = Eigen::Vector2i;

using RowVec1 = Eigen::Matrix<real_t, 1, 1>;
using RowVec2 = Eigen::Matrix<real_t, 1, 2>;
using RowVec3 = Eigen::Matrix<real_t, 1, 3>;
using RowVec4 = Eigen::Matrix<real_t, 1, 4>;
using RowVec5 = Eigen::Matrix<real_t, 1, 5>;
using RowVec6 = Eigen::Matrix<real_t, 1, 6>;
using RowVec7 = Eigen::Matrix<real_t, 1, 7>;
using RowVec8 = Eigen::Matrix<real_t, 1, 8>;
using RowVec9 = Eigen::Matrix<real_t, 1, 9>;

// Aligned std::vector
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

// Eigen Ref and Map
template <typename T>
using EigenRef = Eigen::Ref<T>;
template <typename T>
using EigenCRef = Eigen::Ref<const T>;
template <typename T>
using EigenMap = Eigen::Map<T>;
template <typename T>
using EigenCMap = Eigen::Map<const T>;

/// usage: auto arr =  make_array<20>('z');
template <size_t N, typename T>
std::array<T, N> make_array(const T &v) {
  std::array<T, N> arr;
  arr.fill(v);
  return arr;
}

}  // namespace sv
