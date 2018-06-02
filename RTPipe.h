#ifndef RTPIPE_H_DEFINED
#define RTPIPE_H_DEFINED

#include <fcntl.h>
#include <Bela.h> // rt_fprintf
#include <native/pipe.h>

class RTPipe {
  char const *name;
  RT_PIPE pipe;
  int fd = -1;
public:
  RTPipe(char const *name, size_t pool = 0) : name(name) {
    int ret = rt_pipe_create(&pipe, name, P_MINOR_AUTO, pool);
    if (ret < 0) {
      std::cerr << "Failed to create pipe " << name << std::endl;
      throw std::bad_alloc();
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
      std::cerr << "Failed to open pipe " << name << std::endl;
      throw std::bad_alloc();
    }
    return fd;
  }
  int fileDescriptor() const { return fd; }
  void write(void *buf, size_t size, int mode = P_NORMAL) {
    int ret = rt_pipe_write(&pipe, buf, size, mode);
    if (ret < 0) {
      rt_fprintf(stderr, "Failed to write to pipe %s\n", name);
    }
  }
};

#endif // RTPIPE_H_DEFINED
