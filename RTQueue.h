#ifndef RTQUEUE_H_DEFINED
#define RTQUEUE_H_DEFINED

#include <iostream>

class RTQueue {
  char const *name;
  mqd_t queue;
public:
  RTQueue(char const *name, size_t msgSize) : name(name) {
    std::string qname = "/q_";
    qname += name;
    mq_attr attr;
    attr.mq_maxmsg = 4;
    attr.mq_msgsize = msgSize;
    queue = __wrap_mq_open(qname.c_str(), O_CREAT | O_RDWR | O_NONBLOCK, 0644, &attr);
    if (queue < 0) {
      throw std::system_error(errno, std::system_category(), name);
    }
  }
  ~RTQueue() {
    __wrap_mq_close(queue);
    std::string qname = "/q_";
    qname += name;
    __wrap_mq_unlink(qname.c_str());
  }
  template<typename T> void write(T const &obj) {
    static_assert(std::is_trivially_copyable<T>::value,
		  "objects written to a queue need to be trivially copyable");
    if (__wrap_mq_send(queue, (char*)&obj, sizeof(T), 0)) {
      rt_fprintf(stderr, "Error while sending to queue %s: %s\n", name, strerror(errno));
    }
  }
  ssize_t receive(void *buf, size_t size) {
    ssize_t ret = __wrap_mq_receive(queue, (char *)buf, size, nullptr);
    if (ret < 0 && errno != EAGAIN) {
      rt_fprintf(stderr, "Error while receiving from queue %s: %s\n",
		 name, strerror(errno));
    }
    return ret;
  }
};

#endif
