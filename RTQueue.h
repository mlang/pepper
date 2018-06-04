#ifndef RTQUEUE_H_DEFINED
#define RTQUEUE_H_DEFINED

#include <native/queue.h>
#include <iostream>

class RTQueue {
  RT_QUEUE queue;
public:
  RTQueue(char const *name, size_t pool = 0x10000) {
    int ret = rt_queue_create(&queue, name, pool, TM_INFINITE, Q_FIFO);
    if (ret < 0) {
      throw std::system_error(-ret, std::system_category(), name);
    }
  }
  ~RTQueue() { rt_queue_delete(&queue); }
  void *alloc(size_t size) {
    return rt_queue_alloc(&queue, size);
  }
  void free(void *buf) {
    rt_queue_free(&queue, buf);
  }
  template<typename T, typename... Args> int send(Args&&... args) {
    return rt_queue_send(&queue,
      new (alloc(sizeof(T))) T(std::forward<Args>(args)...), sizeof(T),
      Q_NORMAL
    );
  }
  template<typename T> int write(T const &obj) {
    static_assert(std::is_trivially_copyable<T>::value,
		  "objects written to a queue need to be trivially copyable");
    return rt_queue_write(&queue, &obj, sizeof(T), Q_NORMAL);
  }
  ssize_t receive(void *&buf) {
    return rt_queue_receive(&queue, &buf, TM_NONBLOCK);
  }
};

#endif
