#ifndef _COP_TYPEDEFS_H_
#define _COP_TYPEDEFS_H_

#include <limits>
#include <vector>

namespace coplibrary {

	template <typename T = double>
	using VecT = std::vector <T>;
	using VecDbl = VecT<double>;
	using VecUint = VecT<size_t>;

	template <typename T = double>
	using MatT =  std::vector <VecT <T> >;
	using MatDbl =  MatT<double>;
	using MatUint =  MatT<size_t>;

	double constexpr kInf = std::numeric_limits<double>::infinity();
	size_t constexpr kNIL = std::numeric_limits<size_t>::max();

} /* namespace coplibrary */

#endif /* _COP_TYPEDEFS_H_ */
