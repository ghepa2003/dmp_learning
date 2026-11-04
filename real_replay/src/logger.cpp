#include "real_replay/logger.hpp"

#include <cmath>
#include <cstdio>

namespace real_replay {

static_assert(sizeof(LogRecord) % sizeof(double) == 0, "LogRecord must be a flat array of doubles");

Logger::Logger(std::size_t capacity) : buf_(capacity) {}  // value-initialised: zero-filled

bool Logger::allFinite() const noexcept {
  for (std::size_t i = 0; i < size_; ++i) {
    const double* d = reinterpret_cast<const double*>(&buf_[i]);
    for (std::size_t k = 0; k < kNumFields; ++k) {
      if (!std::isfinite(d[k])) return false;
    }
  }
  return true;
}

namespace {

std::string header() {
  std::string h = "t,period,cb_us,success_rate,gain_alpha,phase,flags";
  auto add7 = [&h](const char* base) {
    for (int i = 1; i <= 7; ++i) h += std::string(",") + base + std::to_string(i);
  };
  add7("q");
  add7("dq");
  add7("tau_J");
  add7("tau_cmd");
  h += ",p_des_x,p_des_y,p_des_z";
  h += ",q_des_w,q_des_x,q_des_y,q_des_z";
  h += ",p_meas_x,p_meas_y,p_meas_z";
  h += ",q_meas_w,q_meas_x,q_meas_y,q_meas_z";
  h += ",e_pos_x,e_pos_y,e_pos_z";
  h += ",e_rot_x,e_rot_y,e_rot_z";
  return h;
}

}  // namespace

bool Logger::writeCsv(const std::string& path, std::string* error) const {
  std::FILE* f = std::fopen(path.c_str(), "wx")  /* exclusive: never overwrite */;
  if (!f) {
    if (error) *error = "cannot create '" + path + "' (exists already, or not writable)";
    return false;
  }
  std::fprintf(f, "%s\n", header().c_str());
  char line[Logger::kNumFields * 32 + 8];
  for (std::size_t i = 0; i < size_; ++i) {
    const double* d = reinterpret_cast<const double*>(&buf_[i]);
    int n = 0;
    for (std::size_t k = 0; k < kNumFields; ++k) {
      n += std::snprintf(line + n, sizeof(line) - static_cast<std::size_t>(n), k == 0 ? "%.9g" : ",%.9g",
                         d[k]);
    }
    line[n++] = '\n';
    if (std::fwrite(line, 1, static_cast<std::size_t>(n), f) != static_cast<std::size_t>(n)) {
      if (error) *error = "write error on '" + path + "'";
      std::fclose(f);
      return false;
    }
  }
  if (std::fclose(f) != 0) {
    if (error) *error = "close error on '" + path + "'";
    return false;
  }
  return true;
}

}  // namespace real_replay
