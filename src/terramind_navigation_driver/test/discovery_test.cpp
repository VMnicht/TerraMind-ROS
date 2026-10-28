// 私有 PTY 测试导航身份、歧义、别名、占用、取消和零发送；不访问硬件。
#include "terramind_navigation_driver/port_discovery.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fcntl.h>
#include <pty.h>
#include <thread>
#include <unistd.h>

namespace nav = terramind::navigation;
namespace transport = terramind::transport;
namespace fs = std::filesystem;
namespace
{
// 其他包可同时运行真实扫描测试；忙时只重试锁竞争，不重试识别失败。
auto discover(const std::vector<std::string> & patterns, double seconds,
  const std::function<bool()> & running)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (true) {
    auto result = nav::discover_navigation_board(patterns, seconds, running);
    if (result.errors != std::vector<std::string>{"serial discovery busy; retrying"} ||
      std::chrono::steady_clock::now() >= deadline) {return result;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
struct Directory
{
  std::string path;
  Directory() {
    char name[] = "/tmp/terramind-nav-discovery-XXXXXX";
    const auto p = mkdtemp(name);
    if (!p) {throw std::runtime_error("mkdtemp");}
    path = p;
  }
  ~Directory() {fs::remove_all(path);}
};
class Peer
{
public:
  enum Kind {BOARD, BAD_CRC, WRONG_VERSION, DUPLICATES, QUIET, SILENT_AFTER_MATCH, SLOW_BOARD, SLOW_SILENT};
  std::string path;
  std::atomic<size_t> received{0};
  Peer(const std::string & link, Kind kind) {
    char device[256]{};
    termios settings{};
    settings.c_cflag = CS8 | CLOCAL | CREAD;
    if (openpty(&master_, &slave_, device, &settings, nullptr) < 0) {
      throw std::runtime_error("openpty");
    }
    path = device;
    fs::create_symlink(path, link);
    fcntl(master_, F_SETFL, O_NONBLOCK);
    thread_ = std::thread([this, kind] {
      uint32_t sequence = 0;
      auto next_send = std::chrono::steady_clock::now();
      while (running_) {
        uint8_t input[1024];
        const auto n = read(master_, input, sizeof(input));
        if (n > 0) {received += static_cast<size_t>(n);}
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_send && kind != QUIET &&
          !(kind == SILENT_AFTER_MATCH && sequence >= 8) &&
          !(kind == SLOW_SILENT && sequence >= 3)) {
          next_send = now + std::chrono::milliseconds(
            kind == SLOW_BOARD || kind == SLOW_SILENT ? 1000 : 10);
          if (kind != DUPLICATES || sequence < 8) {++sequence;}
          // NAV_VALID=false、全零姿态/时间仍是合法初始化报文。
          std::array<uint8_t, 80> bytes{};
          bytes[0] = 0xaa; bytes[1] = 0x55; bytes[2] = kind == WRONG_VERSION ? 2 : 1;
          for (int i = 0; i < 4; ++i) {bytes[4+i] = sequence >> (8*i);}
          auto crc = nav::crc16(bytes.data()+2, 76);
          bytes[78] = crc & 255; bytes[79] = crc >> 8;
          if (kind == BAD_CRC) {bytes[30] ^= 1;}
          const auto unused = write(master_, bytes.data(), bytes.size());
          (void)unused;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }
  ~Peer() {
    running_ = false;
    thread_.join();
    close(master_); close(slave_);
  }
private:
  int master_ = -1, slave_ = -1;
  std::atomic<bool> running_{true};
  std::thread thread_;
};
}

TEST(NavigationDiscovery, ThreeAdvancingFramesAndWrapWithoutNavigationFix)
{
  nav::NavigationProbe probe;
  nav::Frame frame;
  frame.sequence = 0xfffffffeu;
  EXPECT_FALSE(probe.observe(frame));
  EXPECT_FALSE(probe.observe(frame));
  frame.sequence++;
  EXPECT_FALSE(probe.observe(frame));
  frame.sequence++;
  EXPECT_TRUE(probe.observe(frame));
  EXPECT_FALSE(probe.observe(frame));
  frame.sequence = 0xffffffffu;  // backwards, restart evidence
  EXPECT_FALSE(probe.observe(frame));
  EXPECT_FALSE(probe.matched());
  frame.sequence = 0; EXPECT_FALSE(probe.observe(frame));
  frame.sequence = 1; EXPECT_TRUE(probe.observe(frame));
}

TEST(NavigationDiscovery, UniqueInvalidNavigationBoardAndAliasesNoTransmission)
{
  Directory dir;
  Peer board(dir.path+"/board", Peer::BOARD);
  Peer crc(dir.path+"/crc", Peer::BAD_CRC);
  Peer version(dir.path+"/version", Peer::WRONG_VERSION);
  Peer quiet(dir.path+"/quiet", Peer::QUIET);
  fs::create_symlink(board.path, dir.path+"/alias");
  auto result = discover({dir.path+"/board", dir.path+"/*"}, .25, [] {return true;});
  ASSERT_TRUE(result.port);
  EXPECT_EQ(result.device, dir.path+"/board");
  EXPECT_EQ(result.candidate_count, 4u);
  EXPECT_EQ(result.matches.size(), 1u);
  EXPECT_EQ(board.received, 0u); EXPECT_EQ(crc.received, 0u);
  EXPECT_EQ(version.received, 0u); EXPECT_EQ(quiet.received, 0u);
  transport::SerialPort other;
  EXPECT_THROW(other.open(board.path), std::runtime_error);
}

TEST(NavigationDiscovery, AmbiguousBoardsReleaseAllHandles)
{
  Directory dir;
  Peer a(dir.path+"/a", Peer::BOARD), b(dir.path+"/b", Peer::BOARD);
  auto result = discover({dir.path+"/*"}, .25, [] {return true;});
  EXPECT_FALSE(result.port); EXPECT_EQ(result.matches.size(), 2u);
  transport::SerialPort owner;
  EXPECT_NO_THROW(owner.open(a.path));
  EXPECT_EQ(a.received, 0u); EXPECT_EQ(b.received, 0u);
}

TEST(NavigationDiscovery, DuplicateAndSilentStreamsDoNotRemainMatched)
{
  Directory dir;
  Peer duplicate(dir.path+"/duplicate", Peer::DUPLICATES);
  Peer silent(dir.path+"/silent", Peer::SILENT_AFTER_MATCH);
  auto result = discover({dir.path+"/*"}, .4, [] {return true;});
  EXPECT_FALSE(result.port); EXPECT_TRUE(result.matches.empty());
}

TEST(NavigationDiscovery, LockedDevicesCancellationAndEmptyScan)
{
  Directory dir;
  Peer a(dir.path+"/a", Peer::BOARD);
  transport::SerialPort owner;
  owner.open(a.path);
  auto result = discover({dir.path+"/*"}, .25, [] {return true;});
  EXPECT_FALSE(result.port); EXPECT_EQ(result.errors.size(), 1u);
  auto empty = discover({dir.path+"/missing*"}, .25, [] {return true;});
  EXPECT_EQ(empty.candidate_count, 0u);
  owner.close();
  const auto start = std::chrono::steady_clock::now();
  auto cancelled = discover({dir.path+"/*"}, 10, [&] {
    return std::chrono::steady_clock::now()-start < std::chrono::milliseconds(80);
  });
  EXPECT_FALSE(cancelled.port);
  EXPECT_LT(std::chrono::steady_clock::now()-start, std::chrono::milliseconds(200));
  EXPECT_NO_THROW(owner.open(a.path));
  EXPECT_THROW(discover({}, 0., [] {return true;}), std::invalid_argument);
}

TEST(NavigationDiscovery, OneHertzInitializationHeartbeatWithoutFix)
{
  Directory dir;
  Peer board(dir.path+"/initializing", Peer::SLOW_BOARD);
  auto result = discover({dir.path+"/*"}, 3.5, [] {return true;});
  ASSERT_TRUE(result.port);
  EXPECT_EQ(result.device, dir.path+"/initializing");
  EXPECT_EQ(board.received, 0u);
}

TEST(NavigationDiscovery, SilentOneHertzHeartbeatExpires)
{
  Directory dir;
  Peer board(dir.path+"/silent-initializing", Peer::SLOW_SILENT);
  auto result = discover({dir.path+"/*"}, 4.2, [] {return true;});
  EXPECT_FALSE(result.port);
  EXPECT_TRUE(result.matches.empty());
}
