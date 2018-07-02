#ifndef STDFUTURES_EXPERIMENTAL_BITS_STANDARD_FUTURE_HPP
#define STDFUTURES_EXPERIMENTAL_BITS_STANDARD_FUTURE_HPP

#include "future_completion_token.hpp"
#include "future_continuation.hpp"
#include "executors_helpers.hpp"
#include "standard_promise_shared_state.hpp"

#include <experimental/execution>

#include <any>
#include <chrono>
#include <type_traits>
#include <tuple>
#include <variant>

namespace std {
namespace experimental {
inline namespace futures_v1 {
namespace execution {

namespace __continuable_future_impl {

template <typename T, typename Executor, typename Ret=any>
struct __cf_impl_base {

  using poly_future_continuation_t = __future_continuation_impl::__poly_future_continuation<
    Ret, T
  >;

  virtual
  executors_v1::execution::executor_future_t<Executor, Ret>
  __invoke_then(poly_future_continuation_t) && = 0;

};

template <typename ContinuableFuture, typename T, typename Executor, typename Ret=any>
struct __cf_erase_impl : __cf_impl_base<T, Executor, Ret> {
  ContinuableFuture f_;

  explicit __cf_erase_impl(ContinuableFuture&& f) : f_(std::move(f)) { }

  executors_v1::execution::executor_future_t<Executor, Ret>
  __invoke_then(typename base_t::poly_future_continuation_t cont) && override {
    return f_.then(std::move(cont));
  }
};

template <typename ContinuableFuture, typename T, typename NewExecutor, typename Ret=any>
struct __cf_via_erase_impl : __cf_impl_base<T, NewExecutor, Ret> {
  ContinuableFuture f_;
  NewExecutor new_ex_;

  explicit __cf_erase_impl(ContinuableFuture&& f, NewExecutor new_ex)
    : f_(std::move(f)), new_ex_(new_ex)
  { }

  executors_v1::execution::executor_future_t<NewExecutor, Ret>
  __invoke_then(typename base_t::poly_future_continuation_t cont) && override {
    auto [p, ft] = execution::make_promise_contract<T>(new_ex_);
    executors_v1::execution::require(new_ex_, executors_v1::execution::then).then_execute(
      std::move(cont), std::move(ft)
    );
    std::move(f_).then(execution::on_variant([pp=std::move(p)](auto&& var){
      if(auto* var_val = std::get_if<T>(&var)) {
        std::move(pp).set_value(*var_val);
      }
      else {
        auto* ee_val = std::get_if<exception_ptr>(&var);
        std::move(pp).set_exception(*ee_val);
      }
    }));
  }
};

template <typename T, typename OneWayExecutor, typename Ret>
struct __cf_one_way_impl : __cf_impl_base<T, OneWayExecutor, Ret> {
  using base_t = __cf_impl_base<T, OneWayExecutor, Ret>;
  std::shared_ptr<detail::promise_shared_state<T>> core_;
  // TODO Implement shared-state executor here.

  executors_v1::execution::executor_future_t<OneWayExecutor, Ret>
  __invoke_then(typename base_t::poly_future_continuation_t cont) && override {
    // Prepare next in chain
    auto chainedCore =
      std::make_shared<detail::no_executor_promise_shared_state<T>>();
    auto nextFuture = standard_future<Ret, OneWayExecutor>(chainedCore, executor_);

    // Add task
    core_->set_task(
    [chainedCore,
    executor = std::move(executor_),
      continuation = std::move(continuation)](T&& val) mutable {
      executor.execute(
        [chainedCore,
          val = std::move(val),
          continuation = std::move(continuation)]() mutable {
          auto nextVal = continuation(std::move(val));
          chainedCore->set_value(std::move(nextVal));
        });
    });
  }
};

struct __nat { };

template <typename T>
inline constexpr auto __g = execution::on_value_or_error(
  [](T const&) { return 42; },
  [](exception_ptr const&) { return 42; }
);

template <typename ContinuableFuture, typename T, typename Executor>
using __is_cf_archetype = std::tuple<
  decltype(Executor(std::declval<ContinuableFuture>().get_executor())),
  decltype(executors_v1::execution::executor_future_t<Executor, int>(std::declval<ContinuableFuture>().then(__g<T>)))
>;

template <typename ContinuableFuture, typename T, typename Executor>
inline constexpr bool __is_cf_v = is_detected_v<__is_cf_archetype, ContinuableFuture, T, Executor>;

} // __continuable_future_impl


template<class T, class Executor>
class continuable_future {
  private:

    Executor ex_;
    std::unique_ptr<__continuable_future_impl::__cf_impl_base<T, Executor>> impl_;

    template <typename ContinuableFuture>
    continuable_future(
      ContinuableFuture&& f,
      Executor ex
    ) : ex_(ex),
        impl_(std::make_unique<__continuable_future_impl::__cf_via_erase_impl<
          decay_t<ContinuableFuture>, T, Executor>
        >(std::forward<ContinuableFuture>(f), ex))
    { }

    template<class, class>
    friend class continuable_future;

  public:

    using value_type = T;
    using executor_type = Executor;

    continuable_future() = delete;
    continuable_future(continuable_future const&) = delete;
    continuable_future(continuable_future&&) noexcept = default;
    continuable_future& operator=(continuable_future const&) = delete;
    continuable_future& operator=(continuable_future&&) noexcept = default;
    ~continuable_future() noexcept = default;

    // TODO @paper add this type-erasing constructor
    template <typename ContinuableFuture>
    continuable_future(
      ContinuableFuture&& cf,
      std::enable_if_t<
        __continuable_future_impl::__is_cf_v<ContinuableFuture, T, Executor>,
        __continuable_future_impl::__nat
      > = { }
    ) : ex_(cf.get_executor()),
        impl_(
          std::make_unique<__continuable_future_impl:::_cf_erase_impl<
            remove_cv_t<remove_reference_t<ContinuableFuture>>, T, Executor
          >>(std::forward<ContinuableFuture>(cf))
        )
    { }

    template <typename FutureContinuation>
    auto then(FutureContinuation&& fc) && {
      using _Ret = __future_continuation_impl::__return_detect_t<FutureContinuation, T>;
      executors_v1::execution::require(ex_,
        executors_v1::execution::relationship.continuation
      ).then_execute(
        [](any&& ret) { return std::any_cast<_Ret>(ret); },
        impl_->__invoke_then(std::forward<FutureContinuation>(fc))
      );
    }

    template <typename NewExecutor>
    auto via(NewExecutor other) && {
      return continuable_future<T, NewExecutor>(std::move(*this), other);
    }

    auto get_executor() const {
      return ex_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////

    void wait() {
      detail::CVStruct cv;

      // Callback does not consume the value from this core so try_get will still
      // be valid
      core_->set_task([&cv](T&& /* not used */) mutable {
        cv.ready = true;
        cv.cv.notify_one();
      });

      std::unique_lock<std::mutex> lck{cv.cv_mutex};
      cv.cv.wait(lck, [&cv](){return cv.ready == true;});
    }

    template<class Clock, class Duration>
    void wait_until(const chrono::time_point<Clock, Duration>& abs_time) {
      // condition variable heap allocated because we will return from this scope
      // if waiting on CV times out and it must stay alive to be satisfied later
      // by already enqueued callback.
      auto cv = std::make_shared<detail::CVStruct>();

      // Create a new future in the chain that will be in a callback free state
      // again.
      auto chainedCore =
        std::make_shared<detail::no_executor_promise_shared_state<T>>();
      auto nextSemiFuture = standard_semi_future<T>(chainedCore);

      // Callback does not consume the value from this core so try_get will still
      // be valid
      core_->set_task(
        [cv, chainedCore = std::move(chainedCore)](T&& val) mutable {
          cv->ready = true;
          cv->cv.notify_one();
          chainedCore->set_value(std::move(val));
        });

      std::unique_lock<std::mutex> lck{cv->cv_mutex};
      cv->cv.wait_until(lck, abs_time, [cv](){return cv->ready == true;});

      // Replace this future with sf so that timeout still gives us a valid state
      *this = std::move(nextSemiFuture);
    }

    T get() {
      // Signalling pattern for shared state is owned by future
      if(auto* value = core_->try_get()) {
        // If the core has already completed, then there should be a value
        return std::move(*value);
      }
      // Otherwise wait and return value
      wait();
      auto* value = core_->try_get();
      return std::move(*value);
    }

    // Then with one_way executor
    template<class F, class Exec = Executor>
    auto then(
      F&& continuation,
      typename enable_if<
        experimental::execution::is_oneway_executor_v<Exec>>::type* = 0) &&
    -> standard_future<std::result_of_t<F(T&&)>, Exec>;

    // Then with then_executor
    template<class F, class Exec = Executor>
    auto then(
      F&& continuation,
      typename enable_if<
        experimental::execution::is_then_executor_v<Exec>>::type* = 0,
      int a = 0) &&
    -> decltype(std::declval<Exec>().then_execute(
      std::declval<F>(),
      std::move(*this)));

    // Allow via to extract future type from then_executor
    //
    // If the executors are the same, the future type will not change so there
    // is no need to enqueue the cost (and recursion risk) of calling then
    // under the hood.
    // If the executor is one-way, then the future type cannot change.
    template<class NextExecutor>
    auto via(
      NextExecutor&& exec,
      typename enable_if<
        experimental::execution::is_oneway_executor_v<NextExecutor> &&
          !experimental::execution::is_then_executor_v<NextExecutor>>::type* = 0
    ) && -> standard_future<T, NextExecutor>;

    // Allow via to extract future type from then_executor
    //
    // If the executor types are different and the executor is a then_executor,
    // the future type might change.
    template<class NextExecutor>
    auto via(
      NextExecutor&& exec,
      typename enable_if<
        experimental::execution::is_then_executor_v<NextExecutor>>::type* = 0,
      int a = 0) &&
    -> decltype(std::declval<std::decay_t<NextExecutor>>().then_execute(
      std::declval<HelperF>(),
      std::move(*this)));

    // Should be called only by executor implementations
    // Callback should perform only trivial work to let the executor know
    // how to proceed.
    void set_callback(
      std::experimental::execution::future_completion_token<T>&& token){
      core_->set_task(std::move(token));
    }

  private:
    template<class PromiseType>
    friend class standard_promise;
    template<class SemiFutureType>
    friend class standard_semi_future;
    template<class FutureType, class ExecutorType>
    friend class standard_future;

    standard_future() = delete;
    standard_future(
      std::shared_ptr<detail::promise_shared_state<T>> core,
      Executor ex) :
      core_{std::move(core)},
      executor_{std::move(ex)} {
    }






    std::shared_ptr<detail::promise_shared_state<T>> core_;
    Executor executor_;
};


} // end namespace execution
} // fuures_v1
} // experimental
} // std

#endif //STDFUTURES_STANDARD_FUTURE_HPP
