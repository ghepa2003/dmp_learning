#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace haptic_dmp_learning {
namespace core {
namespace target_spawn {

/// Parameters of the free_target_object spawn built by demo_replay_sync_orchestrator_node.
struct TargetSpawnParams {
  std::string name;
  double x = 0.0, y = 0.0, z = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  double wx = 0.0, wy = 0.0, wz = 0.0;  // BODY frame (free_target_object.launch.py)
  double size_x = 0.0, size_y = 0.0, size_z = 0.0;
  double mass = 0.0;
  bool with_collar = true;
  bool collar_collision = true;
  double roll = 0.0;   // [rad]; forwarded only when != 0
  double pitch = 0.0;  // [rad]; forwarded only when != 0
  double yaw = 0.0;    // [rad]; always forwarded
};

/// "key:=value" with std::to_string (6 decimals): the format the orchestrator has always used.
inline std::string numArg(const std::string& key, double v) {
  return key + ":=" + std::to_string(v);
}

/**
 * @brief Argument vector of `ros2 launch free_target_object free_target_object.launch.py ...`.
 *
 * With roll == 0 and pitch == 0 the vector is the one the orchestrator built before roll/pitch
 * existed (same arguments, same order). A non-zero roll / pitch adds "roll:=" / "pitch:=",
 * inserted right before "yaw:=". Angles follow ros_gz_sim create -R -P -Y: extrinsic,
 * R = Rz(yaw) Ry(pitch) Rx(roll).
 */
inline std::vector<std::string> buildSpawnArgs(const TargetSpawnParams& p) {
  std::vector<std::string> argv = {
      "ros2", "launch", "free_target_object", "free_target_object.launch.py",
      "name:=" + p.name,
      numArg("x", p.x), numArg("y", p.y), numArg("z", p.z),
      numArg("vx", p.vx), numArg("vy", p.vy), numArg("vz", p.vz),
      numArg("wx", p.wx), numArg("wy", p.wy), numArg("wz", p.wz),
      numArg("size_x", p.size_x), numArg("size_y", p.size_y), numArg("size_z", p.size_z),
      numArg("mass", p.mass),
      std::string("with_collar:=") + (p.with_collar ? "true" : "false"),
      std::string("collar_collision:=") + (p.collar_collision ? "true" : "false")};
  if (p.roll != 0.0) argv.push_back(numArg("roll", p.roll));
  if (p.pitch != 0.0) argv.push_back(numArg("pitch", p.pitch));
  argv.push_back(numArg("yaw", p.yaw));
  return argv;
}

/// Roll and pitch must be finite and |pitch| < pi/2 (no gimbal singularity of the R-P-Y convention).
inline bool rollPitchValid(double roll, double pitch) {
  return std::isfinite(roll) && std::isfinite(pitch) && std::fabs(pitch) < 1.5707963267948966;
}

}  // namespace target_spawn
}  // namespace core
}  // namespace haptic_dmp_learning
