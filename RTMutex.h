#ifndef RTMUTEX_H_DEFINED
#define RTMUTEX_H_DEFINED

#include <iostream>
#include <native/mutex.h>

class RTMutex {
  RT_MUTEX mutex;
public:
  RTMutex(char const *name) {
    if (rt_mutex_create(&mutex, name) < 0) {
      std::cerr << "Failed to create mutex " << name << std::endl;
      throw std::bad_alloc();
    }
  }
  ~RTMutex() { rt_mutex_delete(&mutex); }
  void lock() { rt_mutex_acquire(&mutex, TM_INFINITE); }
  void unlock() { rt_mutex_release(&mutex); }
};

#endif // RTMUTEX_H_DEFINED
