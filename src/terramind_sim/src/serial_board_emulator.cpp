#include "terramind_sim/board_model.hpp"
#include "terramind_protocol/frame_parser.hpp"
#include "terramind_control/command_guard.hpp"
#include <pty.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <cerrno>
#include <cstring>
namespace tp = terramind::protocol;
namespace ts = terramind::sim;
namespace tc = terramind::control;
volatile std::sig_atomic_t running = 1;
void stop(int)
{
  running = 0;
}
int main(int argc, char ** argv)
{
  std::string link;
  int corrupt_every = 0, fragment = 117, capabilities = tp::CURRENT_CAPABILITIES;
  ts::Geometry geometry;
  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (i + 1 >= argc) {
        throw std::invalid_argument("missing argument value");
      }
      std::string v = argv[++i];
      if (arg == "--link") {
        link = v;
      } else if (arg == "--wheel-diameter") {
        geometry.wheel_diameter = std::stod(v);
      } else if (arg == "--wheel-separation") {
        geometry.wheel_separation = std::stod(v);
      } else if (arg == "--corrupt-every") {
        corrupt_every = std::stoi(v);
      } else if (arg == "--fragment") {
        fragment = std::stoi(v);
      } else if (arg == "--capabilities") {
        size_t consumed = 0;
        capabilities = std::stoi(v, &consumed);
        if (consumed != v.size()) {
          throw std::invalid_argument("capabilities must be an integer");
        }
      } else {
        throw std::invalid_argument("unknown argument " + arg);
      }
    }
    if (link.empty() || fragment < 1 || corrupt_every < 0 || capabilities < 0 || capabilities > 63) {
      throw std::invalid_argument(
              "usage: serial_board_emulator --link PATH [--fragment N] [--corrupt-every N] [--capabilities 0..63]");
    }
    ts::BoardModel model(geometry, static_cast<uint16_t>(capabilities));
    int master = -1, slave = -1;
    char path[256]{};
    if (openpty(&master, &slave, path, nullptr, nullptr) < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    termios t{};
    tcgetattr(slave, &t);
    cfmakeraw(&t);
    tcsetattr(slave, TCSANOW, &t);
    fcntl(master, F_SETFL, O_NONBLOCK);
    if (symlink(path, link.c_str()) < 0) {
      close(master);
      close(slave);
      throw std::runtime_error(
              "cannot create PTY link (must not already exist)");
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::cout << "PTY_READY " << link << " -> " << path << std::endl;
    double now = tc::steady_seconds();
    model.reset(now);
    double next_status = now;
    tp::FrameParser parser;
    uint16_t seq = 0;
    uint64_t old_errors = 0;
    while (running) {
      pollfd p{master, POLLIN, 0};
      poll(&p, 1, 5);
      uint8_t buffer[1024];
      auto n = read(master, buffer, sizeof(buffer));
      now = tc::steady_seconds();
      if (n > 0) {
        for (const auto & frame:parser.feed(buffer, n)) {
          model.receive(frame, now);
        }
      }
      auto errors = parser.errors();
      model.add_rx_errors(errors - old_errors);
      old_errors = errors;
      model.step(now);
      if (now >= next_status) {
        auto bytes = tp::encode_state(model.state(), seq++);
        if (corrupt_every && seq % corrupt_every == 0) {
          bytes[bytes.size() - 4] ^= 1;
        }
        size_t offset = 0;
        double deadline = tc::steady_seconds() + 0.020;
        while (offset < bytes.size() && running && tc::steady_seconds() < deadline) {
          auto count =
            write(master, bytes.data() + offset, std::min<size_t>(fragment, bytes.size() - offset));
          if (count > 0) {
            offset += count;
          } else {
            pollfd out{master, POLLOUT, 0};
            poll(&out, 1, 1);
          }
        }
        next_status = now + 0.050;
      }
    }
    if (std::filesystem::is_symlink(link) && std::filesystem::read_symlink(link) == path) {
      unlink(link.c_str());
    }
    close(master);
    close(slave);
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "serial_board_emulator: " << e.what() << std::endl;
    return 1;
  }
}
