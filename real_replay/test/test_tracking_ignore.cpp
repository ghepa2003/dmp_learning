// dry_run.ignore_tracking_error: true is accepted only in dry-run.
//   test_tracking_ignore <config.yaml>
#include <cstdio>
#include <string>

#include "real_replay/config.hpp"

using namespace real_replay;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_tracking_ignore <config.yaml>\n");
    return 2;
  }
  Config cfg = loadConfig(argv[1]);
  int fail = 0;
  auto check = [&fail](bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++fail;
  };
  std::string msg;

  cfg.dry_run.ignore_tracking_error = true;
  check(!trackingIgnoreAllowed(cfg, /*dry_run=*/false, &msg), "ignore=true, robot mode: refused");
  check(msg == "ignore_tracking_error e consentito solo in dry-run", "message text");
  msg.clear();
  check(trackingIgnoreAllowed(cfg, /*dry_run=*/true, &msg) && msg.empty(), "ignore=true, dry-run: allowed");

  cfg.dry_run.ignore_tracking_error = false;
  check(trackingIgnoreAllowed(cfg, false, &msg), "ignore=false, robot mode: allowed");
  check(trackingIgnoreAllowed(cfg, true, &msg), "ignore=false, dry-run: allowed");
  check(!trackingIgnoreAllowed(Config{[&] { Config c = cfg; c.dry_run.ignore_tracking_error = true; return c; }()}, false, nullptr),
        "nullptr message accepted");
  std::printf("%s\n", fail ? "FAILED" : "all passed");
  return fail ? 1 : 0;
}
