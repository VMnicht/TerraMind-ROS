#include "terramind_transport/port_discovery.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <glob.h>
#include <set>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace terramind::transport
{
std::vector<std::string> serial_candidates(const std::vector<std::string> & patterns)
{
  std::vector<std::string> paths;
  std::set<dev_t> devices;
  for (const auto & pattern : patterns) {
    glob_t expanded{};
    const int result = glob(pattern.c_str(), 0, nullptr, &expanded);
    if (result != 0 && result != GLOB_NOMATCH) {
      globfree(&expanded);
      throw std::runtime_error("cannot enumerate serial pattern: " + pattern);
    }
    for (size_t i = 0; i < expanded.gl_pathc; ++i) {
      struct stat info{};
      const std::string path = expanded.gl_pathv[i];
      if (stat(path.c_str(), &info) == 0 && S_ISCHR(info.st_mode) &&
        devices.insert(info.st_rdev).second)
      {
        paths.push_back(path);
      }
    }
    globfree(&expanded);
  }
  return paths;
}

DiscoveryLock::DiscoveryLock()
{
  const auto path = "/tmp/terramind-serial-discovery-" + std::to_string(geteuid()) + ".lock";
  const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (fd < 0) {throw std::runtime_error("open discovery lock: " + std::string(strerror(errno)));}
  struct stat info{};
  if (fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_uid != geteuid() ||
    (info.st_mode & 0077) != 0)
  {
    ::close(fd);
    throw std::runtime_error("unsafe discovery lock file");
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    const int error = errno;
    ::close(fd);
    if (error == EWOULDBLOCK || error == EAGAIN) {return;}
    throw std::runtime_error("lock discovery: " + std::string(strerror(error)));
  }
  fd_ = fd;
}
DiscoveryLock::~DiscoveryLock()
{
  // 不删除锁文件，否则等待者可能锁住不同 inode。
  if (fd_ >= 0) {::close(fd_);}
}
}
