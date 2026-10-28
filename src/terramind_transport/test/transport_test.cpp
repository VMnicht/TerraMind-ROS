// 仅使用 PTY 验证字节读写和设备锁，不访问物理串口。
#include "terramind_transport/serial_port.hpp"
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>
#include <poll.h>
TEST(Transport, PtyReadWriteAndOwnership) {
  int master, slave;
  char path[256];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  terramind::transport::SerialPort p;
  p.open(path);
  terramind::transport::SerialPort second;
  EXPECT_THROW(second.open(path), std::runtime_error);
  p.write_all({0xfc, 0, 0xff});
  pollfd fd{master, POLLIN, 0};
  ASSERT_GT(poll(&fd, 1, 100), 0);
  uint8_t b[3];
  ASSERT_EQ(read(master, b, 3), 3);
  EXPECT_EQ(b[2], 255);
  ASSERT_EQ(write(master, b, 3), 3);
  auto out = p.read_some(100);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], 0xfc);
  p.close();
  EXPECT_NO_THROW(second.open(path));
  second.close();
  close(master);
  close(slave);
}
TEST(Transport, InvalidDeviceAndBaud) {
  terramind::transport::SerialPort p;
  EXPECT_THROW(p.open("/not/a/serial/port"), std::runtime_error);
  EXPECT_THROW(p.open("/dev/null", 9600), std::invalid_argument);
}
TEST(Transport, NavigationBaudAndOwnership) {
  int master, slave;
  char path[256];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  terramind::transport::SerialPort p;
  ASSERT_NO_THROW(p.open(path, 460800));
  termios settings{};
  ASSERT_EQ(tcgetattr(slave, &settings), 0);
  EXPECT_EQ(cfgetispeed(&settings), B460800);
  EXPECT_EQ(cfgetospeed(&settings), B460800);
  terramind::transport::SerialPort control;
  EXPECT_THROW(control.open(path, 115200), std::runtime_error);
  const uint8_t bytes[]{0xaa, 0x55, 0, 0xff};
  ASSERT_EQ(write(master, bytes, sizeof(bytes)), 4);
  EXPECT_EQ(p.read_some(100), (std::vector<uint8_t>{0xaa, 0x55, 0, 0xff}));
  p.close(); close(master); close(slave);
}

#include "terramind_transport/port_discovery.hpp"
#include <sys/wait.h>
#include <chrono>
#include <thread>

namespace {
std::unique_ptr<terramind::transport::DiscoveryLock> acquire_scan_lock() {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (true) {
    auto lock = std::make_unique<terramind::transport::DiscoveryLock>();
    if (lock->acquired() || std::chrono::steady_clock::now() >= deadline) {return lock;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}

TEST(Transport, DiscoveryLockExcludesOtherProcessesAndReleases) {
  {
    auto owner = acquire_scan_lock();
    ASSERT_TRUE(owner->acquired());
    const auto child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
      terramind::transport::DiscoveryLock competitor;
      _exit(competitor.acquired() ? 1 : 0);
    }
    int status = 0;
    ASSERT_EQ(waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status)); EXPECT_EQ(WEXITSTATUS(status), 0);
  }
  auto next = acquire_scan_lock();
  EXPECT_TRUE(next->acquired());
}
TEST(Transport, MoveKeepsOwnershipAndClosesPreviousHandle) {
  int master, slave, other_master, other_slave;
  char path[256], other_path[256];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  ASSERT_EQ(openpty(&other_master, &other_slave, other_path, nullptr, nullptr), 0);
  {
    terramind::transport::SerialPort source, destination, check;
    source.open(path, 460800); destination.open(other_path);
    destination = std::move(source);
    EXPECT_FALSE(source.is_open()); EXPECT_TRUE(destination.is_open());
    EXPECT_THROW(check.open(path), std::runtime_error);
    EXPECT_NO_THROW(check.open(other_path));
    terramind::transport::SerialPort moved(std::move(destination));
    EXPECT_FALSE(destination.is_open());
    EXPECT_THROW(check.open(path), std::runtime_error);
  }
  close(master); close(slave); close(other_master); close(other_slave);
}
