#ifndef STD_EXPERIMENTAL_FUTURES_V1_BITS_UTILITY_HPP
#define STD_EXPERIMENTAL_FUTURES_V1_BITS_UTILITY_HPP

#include <type_traits>

namespace std {

// Put this in for now, since it will be in 20, but my standard library implementation doesn't have it yet
template<class T>
struct remove_cvref {
  using type = std::remove_cv_t<std::remove_reference_t<T>>;
};

template<class T>
using remove_cvref_t = typename remove_cvref<T>::type;

} // end namespace std

#endif //STD_EXPERIMENTAL_FUTURES_V1_BITS_UTILITY_HPP
