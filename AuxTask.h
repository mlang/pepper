#if !defined(AUXTASK_H_INCLUDED)
#define AUXTASK_H_INCLUDED
// AuxTask runs a void function (with arguments) in a Bela auxiliary task.
//
// NOTE: Only specialisations for NonRT tasks are implemented right now.
//
// ## Usage
//
// ### Pointer to (non overloaded) member function
//
// class Class {
//   void doTask(...) { ... }
// public:
//   Class() : task("task", &Class::doTask, this) {}
//   AuxTask<NonRT, &Class::doTask> task;
// };
// Class instance;
// instance.task(...);
//
// (you need to pass the type of an overloaded member function explicitly.)
//
// ### Pointer to (non overloaded) function
//
// void doTask(...) { ... }
// AuxTask<NonRT, decltype(&doTask)> task("task", &doTask);
// task(...);
// ----------------------------------------------------------------------------------
#include <cstddef>
#include <AuxTaskNonRT.h>
#include <boost/lockfree/spsc_queue.hpp>

// In C++17 we can use std::apply(memfun, std::tuple_cat(std::tuple(instance), args))
namespace compat {

template<typename MF, typename C, typename T, size_t... I>
decltype(auto) apply_impl(MF&& mf, C *c, T&& t, std::index_sequence<I...>) {
  return (c->*std::forward<MF>(mf))(std::get<I>(std::forward<T>(t))...);
}
template<typename F, typename T, size_t... I>
decltype(auto) apply_impl(F&& f, T&& t, std::index_sequence<I...>) {
  return std::forward<F>(f)(std::get<I>(std::forward<T>(t))...);
}

template<typename MF, typename C, typename T>
decltype(auto) apply(MF&& mf, C&& c, T&& t) {
  std::size_t constexpr tSize = std::tuple_size<std::remove_reference_t<T>>::value;
  return apply_impl(std::forward<MF>(mf), std::forward<C>(c), std::forward<T>(t),
                    std::make_index_sequence<tSize>());
} 
template<typename F, typename T>
decltype(auto) apply(F&& fn, T&& t) {
  std::size_t constexpr tSize = std::tuple_size<std::remove_reference_t<T>>::value;
  return apply_impl(std::forward<F>(fn), std::forward<T>(t),
                    std::make_index_sequence<tSize>());
}

} // namespace compat

enum TaskKind { RT, NonRT };
template<enum TaskKind, typename Callback> class AuxTask;
template<typename Class, typename... Args>
class AuxTask<NonRT, void (Class::*)(Args...)> : AuxTaskNonRT {
  using function_type = void (Class::*)(Args...);
  function_type const member_function;
  Class * const instance;
  boost::lockfree::spsc_queue<std::tuple<Args...>, boost::lockfree::capacity<10>>
  arguments;
  static void call(void *ptr) {
    auto task = static_cast<AuxTask *>(ptr);
    task->arguments.consume_one([task](std::tuple<Args...> &args) {
      compat::apply(task->member_function, task->instance, args);
    });
  }
public:
  AuxTask(const char *name, function_type callback, Class *instance)
  : AuxTaskNonRT(), member_function(callback), instance(instance) {
    create(name, call, this);
  }
  ~AuxTask() { cleanup(); }
  void operator()(Args... args) {
    arguments.push(std::tuple<Args...>(args...));
    schedule();
  }
};
template<typename... Args>
class AuxTask<NonRT, void (*)(Args...)> : AuxTaskNonRT {
  using function_type = void (*)(Args...);
  function_type const function;
  boost::lockfree::spsc_queue<std::tuple<Args...>, boost::lockfree::capacity<10>>
  arguments;
  static void call(void *ptr) {
    auto task = static_cast<AuxTask *>(ptr);
    task->arguments.consume_one([task](std::tuple<Args...> &args) {
      compat::apply(task->function, args);
    });
  }
public:
  AuxTask(const char *name, function_type callback)
  : AuxTaskNonRT(), function(callback) {
    create(name, call, this);
  }
  ~AuxTask() { cleanup(); }
  void operator()(Args... args) {
    arguments.push(std::tuple<Args...>(args...));
    schedule();
  }
};
#endif // AUXTASK_H_INCLUDED
