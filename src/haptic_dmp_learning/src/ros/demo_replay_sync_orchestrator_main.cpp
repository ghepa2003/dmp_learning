#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/demo_replay_sync_orchestrator_node.hpp"

/**
 * @brief Entry point for the demo/replay time-sync orchestrator.
 *
 * Fail-loud on two fronts:
 *   - a missing/invalid mandatory parameter (mode, run_id in replay,
 *     hard_force_limit_n in replay) throws out of the constructor;
 *   - a runtime abort (no /clock, no target odometry) sets failed() and calls
 *     rclcpp::shutdown() from inside the orchestration timer.
 * Either way this process exits non-zero.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<haptic_dmp_learning::ros_wrapper::DemoReplaySyncOrchestratorNode> node;
    try {
        node = std::make_shared<haptic_dmp_learning::ros_wrapper::DemoReplaySyncOrchestratorNode>();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("demo_replay_sync_orchestrator"),
                     "Fatal initialization error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    const bool failed = node->failed();
    node.reset();  // run the destructor (stops child processes) before shutdown
    if (rclcpp::ok()) rclcpp::shutdown();
    return failed ? 1 : 0;
}
