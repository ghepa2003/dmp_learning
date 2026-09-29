#include "satellite_grasp_planner/core/grasp_launch.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace satellite_grasp_planner {
namespace core {

GraspLaunchResult planGraspAndLaunch(const SelectionInputs& in,
                                     const SatelliteSnapshot& snapshot_at_acquisition,
                                     double min_delay_s, ClockFn clock,
                                     std::optional<double> t_now_override_s) {
    if (!std::isfinite(min_delay_s) || min_delay_s < 0.0) {
        throw std::invalid_argument("planGraspAndLaunch: min_delay_s must be finite and >= 0.");
    }
    if (snapshot_at_acquisition.omega_rad_s == 0.0) {
        throw std::invalid_argument("planGraspAndLaunch: snapshot_at_acquisition.omega_rad_s is zero.");
    }
    if (snapshot_at_acquisition.omega_rad_s != in.params.scan.omega_rad_s ||
        snapshot_at_acquisition.center != in.cube_params.center_world ||
        snapshot_at_acquisition.axis != in.cube_params.axis_world) {
        throw std::invalid_argument(
            "planGraspAndLaunch: snapshot_at_acquisition does not match the snapshot 'in' was built "
            "from (omega_rad_s/center/axis differ from in.params.scan.omega_rad_s/in.cube_params) - "
            "pass the ACQUISITION snapshot, not a later re-read one (see doc comment).");
    }

    GraspLaunchResult out;

    const haptic_dmp_learning::core::CubeSatelliteModel model(in.cube_params);
    out.selection = selectGrasp(model, in.prodmp_template, in.robot, in.q0, in.params, clock);

    if (!out.selection.found) {
        out.status = (out.selection.stop_reason == GraspSelection::StopReason::kTimeBudgetExhausted)
                         ? GraspLaunchResult::Status::kBudgetExhaustedNoCandidate
                         : GraspLaunchResult::Status::kNoFeasibleCandidate;
        return out;
    }
    out.status = GraspLaunchResult::Status::kSelected;

    out.tau_launch_s =
        in.contact_assumed_at_end ? in.prodmp_template.tau() : in.params.scan.tau_contact_s;
    if (!(out.tau_launch_s > 0.0)) {
        throw std::invalid_argument(
            "planGraspAndLaunch: tau_launch_s <= 0 (" +
            std::string(in.contact_assumed_at_end ? "prodmp_template.tau()"
                                                   : "in.params.scan.tau_contact_s") +
            " is not positive) - refusing to call planLaunch().");
    }

    const double t_now_s =
        t_now_override_s.value_or(snapshot_at_acquisition.t_s + out.selection.elapsed_s);
    out.launch = planLaunch(snapshot_at_acquisition, t_now_s, out.selection.best.row.theta_rad,
                            out.tau_launch_s, min_delay_s);

    out.goal_position = out.selection.goal_position;
    out.goal_orientation = out.selection.goal_orientation;
    out.k = out.selection.best.row.k;
    out.theta_star_rad = out.selection.best.row.theta_rad;
    out.psi_rad = out.selection.best.psi_rad;
    return out;
}

}  // namespace core
}  // namespace satellite_grasp_planner
