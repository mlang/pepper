#if !defined(AUXTASK_H_INCLUDED)
#define AUXTASK_H_INCLUDED
#include <cstddef>
#include <AuxTaskNonRT.h>
#include <boost/lockfree/spsc_queue.hpp>

// In C++17 we can use std::apply(memfun, std::tuple_cat(std::tuple(instance), args))
namespace detail {

template<typename F, typename C, typename Tuple, size_t ...S>
decltype(auto)
apply_tuple_impl(C&& cl, F&& fn, Tuple&& t, std::index_sequence<S...>) {
  return (std::forward<C>(cl)->*std::forward<F>(fn))
         (std::get<S>(std::forward<Tuple>(t))...);
}
template<typename C, typename F, typename Tuple>
decltype(auto) apply_from_tuple(C&& cl, F&& fn, Tuple&& t) {
  std::size_t constexpr tSize =
    std::tuple_size<std::remove_reference_t<Tuple>>::value;
  return apply_tuple_impl(std::forward<C>(cl), std::forward<F>(fn),
                          std::forward<Tuple>(t), std::make_index_sequence<tSize>());
} 

}

enum TaskKind { RT, NonRT };
template<enum TaskKind, typename Callback> class AuxTask;
template<typename Class, typename... Args>
class AuxTask<NonRT, void (Class::*)(Args...)> : AuxTaskNonRT {
  void (Class::* const member_function)(Args...);
  Class * const instance;
  boost::lockfree::spsc_queue<std::tuple<Args...>, boost::lockfree::capacity<10>>
  arguments;
  static void call(void *ptr, int) {
    auto task = static_cast<AuxTask *>(ptr);
    task->arguments.consume_one([task](std::tuple<Args...> &args) {
      detail::apply_from_tuple(task->instance, task->member_function, args);
    });
  }
public:
  AuxTask(const char *name, void (Class::*callback)(Args...), Class *instance)
  : AuxTaskNonRT(), member_function(callback), instance(instance) {
    create(name, call);
  }
  void operator()(Args... args) {
    arguments.push(std::tuple<Args...>(args...));
    schedule(this, sizeof(Class));
  }
};
#endif // AUXTASK_H_INCLUDED
