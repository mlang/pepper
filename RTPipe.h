#ifndef RTPIPE_H_DEFINED
#define RTPIPE_H_DEFINED

#include <fcntl.h>
#include <Bela.h> // rt_fprintf
#include <native/pipe.h>
#include <iostream>
#include <system_error>

class RTPipe {
  char const *name;
  RT_PIPE pipe;
  int fd = -1;
public:
  RTPipe(char const *name, size_t pool = 0) : name(name) {
    int ret = rt_pipe_create(&pipe, name, P_MINOR_AUTO, pool);
    if (ret < 0) {
      throw std::system_error(-ret, std::system_category(), name);
    }
  }
  ~RTPipe() {
    rt_pipe_delete(&pipe);
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
  template<typename T> void write(T const &obj, int mode = P_NORMAL) {
    static_assert(std::is_trivially_copyable<T>::value,
		  "objects written to a pipe need to be trivially copyable");
    int ret = rt_pipe_write(&pipe, &obj, sizeof(T), mode);
    if (ret < 0) {
      rt_fprintf(stderr, "Failed to write to pipe %s\n", name);
    }
  }
};

#endif // RTPIPE_H_DEFINED
