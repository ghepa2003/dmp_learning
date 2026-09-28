#pragma once

/**
 * @file demo_params.hpp
 * @brief Demo parameters file read by the on-board optimizer next to the ProDMP weights file.
 *
 * It does NOT contain weights and cannot be mistaken for a weights file: its keys are different
 * (format_version, weights_sha256, demo, satellite_at_demo, fit) and it binds itself to one
 * weights file by name and SHA-256. SI units (m, s, rad).
 *
 * YAML schema (exact keys):
 *   format_version: 1
 *   weights_file: <file name, relative to the folder of the demo-params file>
 *   weights_sha256: <64 lowercase hex characters>
 *   demo: {tau_s, t_contact_s, delta_g_demo_m, contact_to_end_offset_m, start_position_demo_frame_m,
 *          start_joints_rad, end_joints_rad}
 *   satellite_at_demo: {center_m, axis, omega_rad_s, grasp_point, phase_at_contact_rad}
 *   fit: {num_basis, ridge_lambda, position_filter_window_s, fix_goal_to_demo_endpoint}
 * EVERY key above must be present. The optional entries (listed below) must appear with the value
 * null when not available: an absent key is an
 * error, null is a declared "not available" (optionals: demo.t_contact_s, demo.start_joints_rad,
 * demo.end_joints_rad, satellite_at_demo). No silent defaults. Unknown extra keys are tolerated
 * (like the weights parser). grasp_point is one of kP0, kP90, kP180, kP270.
 */

#include <array>
#include <iosfwd>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/joint_states_csv_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {
namespace demo_params {

struct SatelliteAtDemo {
    Eigen::Vector3d center_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    double omega_rad_s = 0.0;
    GraspPointId grasp_point = GraspPointId::kP0;
    double phase_at_contact_rad = 0.0;
};

struct FitInfo {
    int num_basis = 0;
    double ridge_lambda = 0.0;
    double position_filter_window_s = 0.0;
    bool fix_goal_to_demo_endpoint = false;
};

struct DemoParams {
    int format_version = 1;
    std::string weights_file;    ///< name relative to the folder of the demo-params file
    std::string weights_sha256;  ///< 64 lowercase hex characters
    double tau_s = 0.0;
    std::optional<double> t_contact_s;  ///< relative to the first sample, like tau
    Eigen::Vector3d delta_g_demo_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d contact_to_end_offset_m = Eigen::Vector3d::Zero();
    /// Start position in the DEMO frame (only a rotation w.r.t. the Geomagic, no translation): NOT
    /// the robot position; the optimizer's p0 comes from the robot kinematics.
    Eigen::Vector3d start_position_demo_frame_m = Eigen::Vector3d::Zero();
    std::optional<std::array<double, 7>> start_joints_rad;
    std::optional<std::array<double, 7>> end_joints_rad;
    std::optional<SatelliteAtDemo> satellite_at_demo;
    FitInfo fit;
};

/// Thrown by buildFromDemo()/writeForWeights() when the demo timestamps are not strictly
/// increasing (the ProDMP fit silently drops such samples, so callers may want to treat this case
/// differently from real errors). Derives from std::invalid_argument.
class NonIncreasingTimestampsError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

/// Builds the parameters from a recorded demo (raw samples, in time order):
///  - tau_s = demo.back().t - demo.front().t (same definition as ProDMP::learnFromDemonstration);
///  - t_contact_s = t of the first sample with gripper_trigger minus demo.front().t, else nullopt;
///    contact_to_end_offset_m = final position - position at that sample; WITHOUT a trigger it is the
///    zero vector (the contact is then assumed to be at the end of the demo);
///  - delta_g_demo_m = final - initial position; start_position_demo_frame_m = initial position;
///  - start/end joints = first/last row of @p joint_states, when given;
///  - weights_file = file name of @p weights_path, weights_sha256 = SHA-256 of that file's bytes.
/// @throws NonIncreasingTimestampsError for non-increasing times; std::invalid_argument for fewer than
///         2 samples or an empty
///         joint_states vector; std::runtime_error if the weights file cannot be read.
DemoParams buildFromDemo(const std::vector<Sample>& demo,
                         const std::optional<std::vector<joint_states_csv_io::JointStateSample>>& joint_states,
                         const FitInfo& fit, const std::string& weights_path,
                         const std::optional<SatelliteAtDemo>& satellite);

/// "kP0" | "kP90" | "kP180" | "kP270" -> GraspPointId. @throws std::invalid_argument otherwise.
GraspPointId parseGraspPoint(const std::string& name);

/// Convenience used by the fit tools: buildFromDemo() + write() next to @p weights_path (already
/// written): reads the joint-states CSV derived from @p demo_csv_path (deriveJointStatesCsvPath) if it
/// exists, else start/end joints are null; says which on @p log. Writes deriveDemoParamsPath(weights_path),
/// reads it back and checks the alignment with the weights file. Returns the demo-params path; if
/// @p out is given it receives the parameters. Throws like buildFromDemo/read/verifyWeightsAlignment.
std::string writeForWeights(const std::string& demo_csv_path, const std::vector<Sample>& demo,
                            const FitInfo& fit, const std::string& weights_path,
                            const std::optional<SatelliteAtDemo>& satellite, std::ostream& log,
                            DemoParams* out = nullptr);

/// Writes the file with 17 significant digits (exact double round trip); empty optionals as null.
/// Does not validate (read() does). @throws std::runtime_error if the file cannot be opened.
void write(const std::string& path, const DemoParams& params);

/// Reads and validates. @throws std::runtime_error whose message contains the name of the missing
/// key or invalid field. Validation: format_version == 1; tau_s > 0 and finite; if present
/// 0 <= t_contact_s <= tau_s; finite vectors; joints with exactly 7 values; weights_sha256 of 64
/// hex characters; axis norm > 1e-9; omega != 0 when satellite_at_demo is present; known grasp_point.
DemoParams read(const std::string& path);

/// SHA-256 of raw bytes / of a file's bytes, lowercase hex. sha256FileHex throws std::runtime_error
/// if the file cannot be opened.
std::string sha256Hex(const std::string& bytes);
std::string sha256FileHex(const std::string& path);

/// Opens weights_file relative to the folder of @p demo_params_path, hashes its bytes and throws
/// std::runtime_error if the file is missing or the hash differs from params.weights_sha256 (the
/// message reports both hashes).
void verifyWeightsAlignment(const std::string& demo_params_path, const DemoParams& params);

/// Same folder, "<stem>_demo_params.yaml" (weights.yaml -> weights_demo_params.yaml).
std::string deriveDemoParamsPath(const std::string& weights_path);

}  // namespace demo_params
}  // namespace core
}  // namespace haptic_dmp_learning
