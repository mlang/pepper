#ifndef RTPIPE_H_DEFINED
#define RTPIPE_H_DEFINED

#include <fcntl.h>
#include <Bela.h> // rt_fprintf
#include <pthread.h>
#include <xenomai_wraps.h>
#include <iostream>
#include <system_error>

class RTPipe {
  char const *name;
  int fd = -1, socket;
public:
  RTPipe(char const *name, size_t poolSize = 0x10000 * 10) : name(name) {
    socket = createXenomaiPipe(name, poolSize);
    if (socket < 0) {
      throw std::system_error(errno, std::system_category(), name);
    }
  }
  ~RTPipe() {
    if (fd != -1)
      close(fd);
  }
  int open() {
    std::string path = "/proc/xenomai/registry/rtipc/xddp/";
    path += name;
    fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) {
      throw std::system_error(errno, std::system_category(), path);
    }
    return fd;
  }
  int fileDescriptor() const { return fd; }
  template<typename T> void write(T const &obj) {
    static_assert(std::is_trivially_copyable<T>::value,
		  "objects written to a pipe need to be trivially copyable");
    int ret = __wrap_sendto(socket, &obj, sizeof(T), 0, nullptr, 0);
    if (ret < 0) {
      rt_fprintf(stderr, "Failed to write to pipe %s\n", name);
    }
  }
};

#endif // RTPIPE_H_DEFINED
