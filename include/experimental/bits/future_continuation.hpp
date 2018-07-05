//
// Created by David S Hollman on 6/28/18.
//

#ifndef STDFUTURES_EXPERIMENTAL_BITS_FUTURE_CONTINUATION_HPP
#define STDFUTURES_EXPERIMENTAL_BITS_FUTURE_CONTINUATION_HPP

#include <experimental/execution>

#include <experimental/type_traits> // detection idiom
#include <cassert>

namespace std::experimental {
inline namespace executors_v1 {
namespace execution {

namespace __future_continuation_impl {

// TODO needs a version that takes _Ty by const reference
// Type-erased future continuation
template <typename _Ret, typename _Ty>
struct __poly_future_continuation_base {

  virtual _Ret __set_value(_Ty&&) = 0;
  virtual _Ret __set_exception(exception_ptr const&) = 0;

  virtual ~poly_future_continuation_base() noexcept = default;
};

template <typename FutureContinuation, typename _Ty>
using __val_return_detect_archetype = decltype(std::declval<FutureContinuation>()(
  std::declval<_Ty>()
));

template <typename FutureContinuation>
using __exc_return_detect_archetype = decltype(std::declval<FutureContinuation>()(
  exception_arg, std::declval<exception_ptr>()
));

// TODO check the validity of the future continuation elsewhere
template <typename FutureContinuation, typename _Ty>
using __return_detect_t = std::conditional_t<
  is_future_value_continuation_v<FutureContinuation, _Ty>,
  detected_t<__val_return_detect_archetype, FutureContinuation, _Ty>,
  // MUST be a future_exception_continuation
  detected_t<__exc_return_detect_archetype, FutureContinuation>
>;


// TODO @paper we need constraints on return types for future continuation concept
template <typename FutureContinuation, typename _Ret, typename _Ty>
struct __poly_future_continuation_impl
  : __poly_future_continuation_base<_Ret, _Ty>
{
  FutureContinuation fc_;

  template <typename FutureContinuationDeduced>
  __poly_future_continuation_impl(std::in_place_t,
    FutureContinuationDeduced&& fcd
  ) : fc_(std::forward<FutureContinuationDeduced>(fcd))
  { }

  _Ret __do_set_value(_Ty&& val, std::true_type) {
    return std::move(fc_)(std::move(val));
  }
  _Ret __do_set_value(_Ty&& val, std::false_type) { assert(false); /* should be unreachable */ }

  _Ret __set_value(_Ty&& val) override {
    return __do_set_value(std::move(val), is_future_value_continuation<FutureContinuation>{});
  }


  _Ret __do_set_exception(exception_ptr const& exc, std::true_type) {
    return std::move(fc_)(exception_arg, exc);
  }
  _Ret __do_set_exception(exception_ptr const&, std::false_type) { assert(false); /* should be unreachable */ }

  _Ret __set_exception(exception_ptr const& exc) override {
    return __do_set_exception(exc, is_future_exception_continuation<FutureContinuation>{});
  }

  ~__poly_future_continuation_impl() noexcept override = default;

};

// TODO add allocators once they're in the paper
template <typename _Ret, typename _Ty>
class __poly_future_continuation {
  private:
    using impl_ptr_t = std::unique_ptr<__poly_future_continuation_base<_Ret, _Ty>>;
    impl_ptr_t impl_;
    // Cache these for efficiency
    // Obviously, these could be compacted into a bitfield...
    const bool handle_value_detected;
    const bool handle_exception_detected;
  public:
    template <typename FutureContinuation>
    explicit __poly_future_continuation(
      FutureContinuation&& fc,
      std::enable_if_t<
        is_future_continuation_v<remove_cv_t<remove_reference_t<FutureContinuation>>, _Ty>
          && !is_same_v<remove_cv_t<remove_reference_t<FutureContinuation>>, __poly_future_continuation>,
        int
      > = 0
    ) : impl_(std::make_unique<__poly_future_continuation_impl<remove_cv_t<remove_reference_t<FutureContinuation>>, _Ret, _Ty>>(
          std::in_place, std::forward<FutureContinuation>(fc)
        )),
        handle_value_detected(is_future_value_continuation_v<remove_cv_t<remove_reference_t<FutureContinuation>>, _Ty>),
        handle_exception_detected(is_future_exception_continuation_v<remove_cv_t<remove_reference_t<FutureContinuation>>>),
    { }

    __poly_future_continuation() noexcept = delete;
    __poly_future_continuation(__poly_future_continuation const&) = delete;
    __poly_future_continuation(__poly_future_continuation&&) noexcept = default;
    __poly_future_continuation& operator=(__poly_future_continuation const&) = delete;
    __poly_future_continuation& operator=(__poly_future_continuation&&) noexcept = default;

    template <typename TDeduced>
      _Ret operator()(TDeduced&& val)
    {
      if(handle_value_detected) {
        auto tmp = impl_ptr_t(std::move(impl_));
        return tmp->set_value(std::forward<TDeduced>(val));
      }
      else {
        return std::forward<TDeduced>(val);
      }
    }

    _Ret operator()(exception_arg_t, exception_ptr exc)
    {
      if(handle_exception_detected) {
        auto tmp = impl_ptr_t(std::move(impl_));
        return tmp->set_exception(exc);
      }
      else {
        std::rethrow_exception(exc);
      }
    }
};


} // end namespace __future_continuation_impl

template <typename _Ret, typename _Ty>
struct is_future_continuation<
  __future_continuation_impl::__poly_future_continuation<_Ret, _Ty>, _Ty
> : std::true_type { };

} // end namespace execution
} // end namespace executors_v1
} // end namespace std::experimental

#endif //STDFUTURES_EXPERIMENTAL_BITS_FUTURE_CONTINUATION_HPP
