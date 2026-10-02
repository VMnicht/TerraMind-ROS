// 被动发现回归：临时 PTY 模拟设备；验证歧义拒绝、别名去重和探测期间零发送。
#include "terramind_mcu/port_discovery.hpp"
#include "terramind_protocol/codec.hpp"
#include "terramind_protocol/frame_parser.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <pty.h>
#include <thread>
#include <unistd.h>

namespace mcu = terramind::mcu;
namespace tp = terramind::protocol;
namespace fs = std::filesystem;

namespace
{
struct Directory
{
  std::string path;
  Directory()
  {
    char name[] = "/tmp/terramind-discovery-XXXXXX";
    auto result = mkdtemp(name);
    if (!result) {throw std::runtime_error("mkdtemp failed");}
    path = result;
  }
  ~Directory() {fs::remove_all(path);}
};

class Peer
{
public:
  enum Kind {BOARD, TEXT, BAD_CRC, DUPLICATES, QUIET};
  int master = -1, slave = -1;
  std::string path;
  std::atomic<size_t> received{0};
  Peer(const std::string & link, Kind kind)
  {
    char device[256]{};
    termios settings{};
    settings.c_cflag = CS8 | CLOCAL | CREAD;
    if (openpty(&master, &slave, device, &settings, nullptr) < 0) {
      throw std::runtime_error("openpty failed");
    }
    path = device;
    fcntl(master, F_SETFL, O_NONBLOCK);
    fs::create_symlink(path, link);
    thread_ = std::thread([this, kind] {
      uint16_t sequence = 0;
      tp::State state;
      const auto start = std::chrono::steady_clock::now();
      auto next = start;
      while (running_) {
        uint8_t bytes[1024];
        const auto n = read(master, bytes, sizeof(bytes));
        if (n > 0) {received += static_cast<size_t>(n);}
        const auto now = std::chrono::steady_clock::now();
        if (now >= next) {
          tp::Bytes packet;
          if (kind == TEXT) {
            const std::string text = "$GNGGA,not-a-control-board\r\n";
            packet.assign(text.begin(), text.end());
          } else if (kind != QUIET) {
            if (kind != DUPLICATES || sequence < 4) {
              ++sequence;
              state.system.uptime_ms = 1 + std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            }
            packet = tp::encode_state(state, sequence);
            if (kind == BAD_CRC) {packet[packet.size() - 4] ^= 1;}
          }
          if (!packet.empty()) {
            const auto unused = write(master, packet.data(), packet.size());
            (void)unused;
          }
          next = now + std::chrono::milliseconds(25);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });
  }
  ~Peer()
  {
    running_ = false;
    thread_.join();
    close(master);
    close(slave);
  }

private:
  std::atomic<bool> running_{true};
  std::thread thread_;
};

tp::Frame state_frame(uint16_t sequence, uint32_t uptime)
{
  tp::State state;
  state.system.uptime_ms = uptime;
  tp::FrameParser parser;
  return parser.feed(tp::encode_state(state, sequence)).at(0);
}
}

TEST(Discovery, DeduplicatesAliasesAndIgnoresNonDevices)
{
  Directory directory;
  Peer peer(directory.path + "/usb", Peer::QUIET);
  fs::create_symlink(peer.path, directory.path + "/preferred");
  fs::create_symlink("/missing", directory.path + "/broken");
  std::ofstream(directory.path + "/regular") << "not a serial port";
  auto paths = mcu::serial_candidates({directory.path + "/preferred", directory.path + "/*"});
  ASSERT_EQ(paths.size(), 1u);
  EXPECT_EQ(paths.front(), directory.path + "/preferred");
}

TEST(Discovery, RequiresFullProgressingStateAndHandlesWrap)
{
  mcu::StatusProbe probe;
  tp::FrameParser parser;
  EXPECT_FALSE(probe.observe(parser.feed(tp::encode_control({}, 0)).at(0)));
  auto malformed = state_frame(0, 0);
  malformed.payload.resize(3);
  EXPECT_FALSE(probe.observe(malformed));
  EXPECT_FALSE(probe.observe(state_frame(7, 100)));
  EXPECT_FALSE(probe.observe(state_frame(7, 150)));
  EXPECT_TRUE(probe.observe(state_frame(8, 150)));
  EXPECT_FALSE(probe.observe(state_frame(8, 150)));  // No freshness renewal.
  EXPECT_FALSE(probe.observe(state_frame(9, 100)));  // Uptime moved backwards.
  EXPECT_FALSE(probe.matched());
  EXPECT_TRUE(probe.observe(state_frame(10, 150)));
  mcu::StatusProbe wrap;
  EXPECT_FALSE(wrap.observe(state_frame(65535, 0xfffffff0u)));
  EXPECT_TRUE(wrap.observe(state_frame(0, 34)));
}

TEST(Discovery, SelectsUniqueBoardWithoutTransmittingToAnyCandidate)
{
  Directory directory;
  Peer text(directory.path + "/navigation", Peer::TEXT);
  Peer corrupt(directory.path + "/bad-crc", Peer::BAD_CRC);
  Peer quiet(directory.path + "/silent", Peer::QUIET);
  Peer board(directory.path + "/control", Peer::BOARD);
  auto found = mcu::discover_control_board({directory.path + "/*"}, .25, [] {return true;});
  ASSERT_TRUE(found.port);
  ASSERT_EQ(found.matches.size(), 1u);
  EXPECT_EQ(found.device, directory.path + "/control");
  EXPECT_EQ(found.candidate_count, 4u);
  EXPECT_EQ(text.received, 0u);
  EXPECT_EQ(corrupt.received, 0u);
  EXPECT_EQ(quiet.received, 0u);
  EXPECT_EQ(board.received, 0u);
  // 识别成功后原句柄持续持锁，从探测到使用之间不能重新打开设备。
  terramind::transport::SerialPort second;
  EXPECT_THROW(second.open(board.path), std::runtime_error);
}

TEST(Discovery, MultipleBoardsAreAmbiguousAndReceiveNoControl)
{
  Directory directory;
  Peer one(directory.path + "/one", Peer::BOARD);
  Peer two(directory.path + "/two", Peer::BOARD);
  auto found = mcu::discover_control_board({directory.path + "/*"}, .25, [] {return true;});
  EXPECT_FALSE(found.port);
  EXPECT_EQ(found.matches.size(), 2u);
  EXPECT_EQ(one.received, 0u);
  EXPECT_EQ(two.received, 0u);
}

TEST(Discovery, RepeatedFramesCannotKeepAnOldIdentificationAlive)
{
  Directory directory;
  Peer duplicate(directory.path + "/duplicate", Peer::DUPLICATES);
  auto found = mcu::discover_control_board({directory.path + "/*"}, .35, [] {return true;});
  EXPECT_FALSE(found.port);
  EXPECT_TRUE(found.matches.empty());
  EXPECT_EQ(duplicate.received, 0u);
}

TEST(Discovery, SkipsLockedPortsAndCancelsPromptly)
{
  Directory directory;
  Peer board(directory.path + "/busy", Peer::BOARD);
  terramind::transport::SerialPort owner;
  owner.open(board.path);
  auto found = mcu::discover_control_board({directory.path + "/*"}, .25, [] {return true;});
  EXPECT_FALSE(found.port);
  EXPECT_EQ(found.errors.size(), 1u);
  EXPECT_EQ(board.received, 0u);
  const auto start = std::chrono::steady_clock::now();
  auto cancelled = mcu::discover_control_board({directory.path + "/*"}, 10, [] {return false;});
  EXPECT_FALSE(cancelled.port);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
}
