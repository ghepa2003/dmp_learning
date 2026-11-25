// Unit tests of include/real_replay/satellite_udp.hpp: layout, pack/unpack, LatestValue under concurrency.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "real_replay/satellite_udp.hpp"

using namespace real_replay::sat;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

int main() {
  std::printf("state datagram\n");
  static_assert(kStatePacketSize == 120, "size");
  StatePacket p;
  p.size = kStatePacketSize;
  p.seq = 42;
  p.state = static_cast<std::uint32_t>(SatState::kRunning);
  p.t_mono_ns = 123456789012345ull;
  p.theta = -1.25;
  p.omega_meas = std::nan("");
  p.theta_q7 = -1.2501;
  p.q7 = 0.5;
  p.omega_cmd = -0.0349;
  p.center[0] = 0.0;
  p.center[1] = 0.72;
  p.center[2] = 0.70;
  p.faults = kFaultPhaseMismatch | kFaultStopCommand;
  p.flags = kFlagOmegaValid;
  std::uint8_t buf[kStatePacketSize];
  packState(p, buf);
  std::uint32_t magic = 0;
  std::memcpy(&magic, buf, 4);
  check(std::memcmp(buf, "SATS", 4) == 0 && magic == kStateMagic, "magic bytes 'SATS' at offset 0");
  double th = 0.0;
  std::memcpy(&th, buf + 24, 8);
  check(th == -1.25, "theta at offset 24 (little-endian double)");
  std::uint32_t faults = 0;
  std::memcpy(&faults, buf + 112, 4);
  check(faults == p.faults, "faults at offset 112");
  std::uint32_t flags = 0;
  std::memcpy(&flags, buf + 116, 4);
  check(flags == kFlagOmegaValid, "flags at offset 116 (bit 0 = omega_valid)");
  StatePacket q;
  check(unpackState(buf, sizeof buf, q), "unpack ok");
  check(q.seq == 42 && q.t_mono_ns == p.t_mono_ns && q.theta == p.theta && std::isnan(q.omega_meas) && q.center[1] == 0.72 && q.faults == p.faults && q.flags == p.flags,
        "round trip of every field (NaN omega preserved)");
  check(!unpackState(buf, sizeof buf - 1, q), "wrong length refused");
  std::uint8_t bad[kStatePacketSize];
  std::memcpy(bad, buf, sizeof bad);
  bad[0] = 'X';
  check(!unpackState(bad, sizeof bad, q), "wrong magic refused");
  std::memcpy(bad, buf, sizeof bad);
  bad[4] = 1;
  check(!unpackState(bad, sizeof bad, q), "old version 1 refused (v2 added flags)");
  std::uint16_t ver = 0;
  std::memcpy(&ver, buf + 4, 2);
  check(ver == 2 && kProtocolVersion == 2, "protocol version 2");

  std::printf("command datagram\n");
  CommandPacket c;
  c.type = static_cast<std::uint16_t>(CommandType::kStart);
  c.seq = 7;
  std::uint8_t cb[kCommandPacketSize];
  packCommand(c, cb);
  CommandPacket d;
  check(std::memcmp(cb, "SATC", 4) == 0 && unpackCommand(cb, sizeof cb, d) && d.type == 2 && d.seq == 7, "START round trip");
  c.type = 9;
  packCommand(c, cb);
  check(!unpackCommand(cb, sizeof cb, d), "unknown type refused");

  std::printf("LatestValue (1 writer, 1 reader, 2 s)\n");
  LatestValue<StatePacket> lv;
  StatePacket out;
  check(!lv.load(out), "empty -> false");
  std::atomic<bool> quit{false};
  std::atomic<long> torn{0}, reads{0};
  std::thread reader([&] {
    while (!quit.load()) {
      StatePacket r;
      if (lv.load(r)) {
        ++reads;
        // writer keeps every field equal to seq: a torn copy would mix two values
        const double s = static_cast<double>(r.seq);
        if (r.theta != s || r.q7 != s || r.center[2] != s || r.t_mono_ns != r.seq) ++torn;
      }
    }
  });
  const auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::uint32_t k = 0;
  while (std::chrono::steady_clock::now() < t_end) {
    StatePacket w;
    w.seq = ++k;
    w.t_mono_ns = k;
    w.theta = w.q7 = w.center[2] = static_cast<double>(k);
    lv.store(w);
  }
  quit.store(true);
  reader.join();
  check(reads.load() > 1000 && torn.load() == 0, "no torn read in " + std::to_string(reads.load()) + " reads, " + std::to_string(k) + " writes");
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
