#pragma once

/**
 * @file dynamic_follow.hpp
 * @brief L2: forward dynamics of the arm driven by the Cartesian impedance law, over a short horizon after AT_END.
 *
 * C++ port of model L2 of _gate_out/endpoint_1028_user/inertial_1028/inertial_pred.py (run_l2, impedance_torque,
 * margin_stats). Per control cycle (dt = 1 ms by default):
 *   1. RobotModel::update(q, v)  -> J, p, R, g(q), c(q,v), M(q)   (pinocchio, the controller's own model)
 *   2. tau = CartesianImpedanceSolver::computeTorque(...)  (impedance_solver.cpp:39-88: task spring/damper, damped nullspace
 *      projection, Coriolis, [+ g(q)], slew-rate saturation delta_tau_max per cycle). Nullspace target = the q_nullspace given.
 *   3. a(q,v,tau) = M^-1 (tau - f(v) - c - g_plant),  f(v) = d v + f_c tanh(v / eps)  (d, f_c per joint from the URDF <dynamics>)
 *   4. RK4 on (q, v) with tau constant over the step; dt = control period.
 * The first cycle is NOT slew-limited (tau_prev is unknown; as in the Python), the following ones are.
 *
 * Not ROS. Eigen + Pinocchio (through RobotModel) only.
 *
 * Known differences with respect to the Python (all deliberate, none changes the equations):
 *   - g_plant is g(q) of the controller's model scaled by plant_gravity_z / model_gravity_z (the Python builds a second
 *     pinocchio model with the plant gravity; generalized gravity is linear in the gravity vector, so for a vertical
 *     gravity this is the same up to rounding). A non-vertical plant gravity is not supported.
 *   - a = M^-1 (...) by Cholesky instead of pinocchio ABA (same quantity up to rounding).
 *   - The fingers are not locked in a reduced model: RobotModel::setPassiveJointPositions() holds them at the given position;
 *     the 7x7 block of CRBA, g and c then equal those of the reduced model with the finger masses fused into the hand.
 */

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/pose_rotation.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

namespace satellite_grasp_planner {
namespace core {

/// Per-joint viscous damping [Nms/rad] and Coulomb friction [Nm] of the plant, the damping= / friction= attributes of the URDF
/// <dynamics> tag of each joint.
struct JointDynamics {
  RobotModel::JointVector damping = RobotModel::JointVector::Zero();
  RobotModel::JointVector friction = RobotModel::JointVector::Zero();
};

/// <dynamics damping friction> of @p joint_names (7 names) from URDF text. @throws std::runtime_error if a joint or its
/// <dynamics> tag is missing (no silent zero), std::invalid_argument for a wrong number of names.
JointDynamics parseJointDynamicsFromUrdfXml(const std::string& urdf_xml, const std::vector<std::string>& joint_names);
/// Same from a file. @throws std::runtime_error if it cannot be read.
JointDynamics loadJointDynamicsFromUrdfFile(const std::string& urdf_path, const std::vector<std::string>& joint_names);

struct DynamicFollowParams {
  // Cartesian impedance + nullspace gains. Defaults: the per-run values of the 36 runs of the L2 validation (runs_inertial.csv,
  // identical for all of them), NOT the 200 N/m of franka_gazebo_controllers.yaml. nullspace_* and delta_tau_max: the YAML.
  double translational_stiffness = 2000.0;  ///< [N/m]
  double rotational_stiffness = 150.0;      ///< [Nm/rad]
  double translational_damping = 10.0;      ///< [Ns/m]
  double rotational_damping = 1.0;          ///< [Nms/rad]
  double nullspace_stiffness = 0.2;
  double joint1_nullspace_stiffness = 100.0;
  double nullspace_pinv_damping = 0.02;
  double delta_tau_max = 1.0;  ///< [Nm] per control cycle (= per integration step)
  /// tau += g(q) of the controller's model. true = set A of the validation (out2/A); the YAML has false (see the report).
  bool compensate_gravity_internally = true;

  // Plant.
  double plant_gravity_z_m_s2 = -9.80665;  ///< vertical gravity applied to the links; set A of the validation
  double model_gravity_z_m_s2 = -9.81;     ///< vertical gravity inside RobotModel (pinocchio default); only the ratio is used
  double friction_tanh_eps_rad_s = 0.1;    ///< eps of the Coulomb tanh; 0.1 = primary of the validation (0.01 and below unstable at 1 ms)
  bool apply_urdf_damping = true;
  bool apply_urdf_friction = true;

  // Integration.
  double dt_s = 0.001;       ///< step = control period. delta_tau_max is per step: another dt changes its meaning
  bool abort_on_divergence = true;  ///< stop at the first divergent step (flag in the result); false: keep integrating
  /// A step diverges if q or v is non-finite OR any joint is farther than this from the initial q0 [rad] (default 2 pi: a joint that
  /// has turned a full revolution from the planned q_end is not a physical hold). Does not change the equations.
  double divergence_rad = 6.283185307179586;

  // Used by the caller that builds the initial condition (selector), not by simulate().
  double finger_pos_m = 0.060;  ///< fixed finger position [m] (gripper open = s1.gripper_open_m; measured at AT_END: 0.058-0.064)
  bool dq0_zero = true;         ///< start from dq = 0 (planned q_end has no velocity)
};

/// @throws std::invalid_argument if a field is not finite or out of its range (gains > 0, dt > 0, eps > 0, ...).
void validateDynamicFollowParams(const DynamicFollowParams& params);

/// Information on one integration step, for diagnostics (dynamic_follow_parity --trace).
struct DynamicFollowStepInfo {
  int step = 0;     ///< i: the state is the one at t = i dt, BEFORE the integration of the step
  double t_s = 0.0;
  RobotModel::JointVector q, v, tau, acc;  ///< acc = a(q, v, tau) (the RK4 k1 of v)
};
using DynamicFollowObserver = std::function<void(const DynamicFollowStepInfo&)>;

struct DynamicFollowResult {
  bool diverged = false;
  double t_div_s = 0.0;  ///< (i + 1) dt of the first divergent step (non-finite, or |q - q0| > divergence_rad); meaningful iff diverged
  int steps_done = 0;    ///< integrated steps (all of them unless diverged)
  double dt_s = 0.0;
  /// q at t = 0, dt, 2 dt, ... (row 0 = the initial condition). Only the finite rows (all of them unless diverged).
  std::vector<RobotModel::JointVector> q;
  /// Minimum over the finite rows and the 7 joints of min(q - lower, upper - q) [rad]; negative if a limit is exceeded
  /// (inertial_pred.py margin_stats). Ties: first row, first joint. NaN if no row.
  double min_margin_rad = 0.0;
  int min_margin_joint = -1;  ///< 0-based
  double min_margin_t_s = 0.0;  ///< time of the row of the minimum

  /// q at @p t_s (nearest multiple of dt); throws std::out_of_range if that row does not exist (e.g. diverged before).
  const RobotModel::JointVector& qAt(double t_s) const;
};

/// Target pose at time t [s] after the start of the simulation (orientation unit quaternion; the velocity fields are unused).
using DynamicFollowTarget = std::function<CartesianSample(double t_s)>;

/// A measured pose series (e.g. /target_pose of a run), zero-order hold: at(t) is the last sample with time <= t + 1e-12, the first
/// one before the first sample (inertial_pred.py Target). Times need not be sorted (stable sort), quaternions are normalized.
class PoseSeries {
 public:
  PoseSeries(const std::vector<double>& t, const std::vector<Eigen::Vector3d>& p, const std::vector<Eigen::Quaterniond>& q);
  CartesianSample at(double t_s) const;
  double tBegin() const { return t_.front(); }
  double tEnd() const { return t_.back(); }

 private:
  std::vector<double> t_;
  std::vector<Eigen::Vector3d> p_;
  std::vector<Eigen::Quaterniond> q_;
};

/// Target from a measured series (the series must outlive the returned function: it is captured by shared_ptr).
DynamicFollowTarget makePoseSeriesTarget(std::shared_ptr<const PoseSeries> series);

/// Ideal target: @p goal_at_end rotated by omega t about the axis through @p center (followTargetSample, follow_margin.hpp), t = time
/// after AT_END. Evaluated at every integration step.
DynamicFollowTarget makeRotatingTarget(const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double omega_rad_s,
                                       const haptic_dmp_learning::core::RotatablePose& goal_at_end);

class DynamicFollowSimulator {
 public:
  /// @param robot shared, mutated (one instance per thread). Its fingers are set by the caller (setPassiveJointPositions).
  /// @throws std::invalid_argument for a null robot or invalid params.
  DynamicFollowSimulator(std::shared_ptr<RobotModel> robot, const JointDynamics& dynamics, const DynamicFollowParams& params);

  /// Per-joint friction torque f(v) = d v + f_c tanh(v / eps) (the terms disabled by the params are 0).
  RobotModel::JointVector frictionTorque(const RobotModel::JointVector& v) const;

  /**
   * @brief Integrates n = round(horizon_s / dt) steps from (q0, dq0).
   * @param q_nullspace nullspace target of the controller (q_d); the controller's own is its configuration at activation.
   * @param target pose to track, called with t = i dt at step i (zero-order hold within the step, as the Python).
   * @param observer optional, called once per step.
   * @throws std::invalid_argument for a non-finite initial condition, an empty target or horizon_s <= 0.
   */
  DynamicFollowResult simulate(const RobotModel::JointVector& q0, const RobotModel::JointVector& dq0,
                               const RobotModel::JointVector& q_nullspace, const DynamicFollowTarget& target, double horizon_s,
                               const DynamicFollowObserver* observer = nullptr);

  const DynamicFollowParams& params() const { return params_; }

 private:
  std::shared_ptr<RobotModel> robot_;
  JointDynamics dyn_;
  DynamicFollowParams params_;
};

}  // namespace core
}  // namespace satellite_grasp_planner
