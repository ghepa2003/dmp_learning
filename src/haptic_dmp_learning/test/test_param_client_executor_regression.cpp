#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <chrono>
#include <memory>
#include <string>

// Regression test for the root cause behind ensureControllerParamClient() in
// prodmp_gazebo_executor_node.cpp routing rclcpp::SyncParametersClient through a dedicated helper
// rclcpp::Node (param_sync_helper_node_) instead of `this`: rclcpp::Node tracks, PER NODE (not per
// executor instance), whether it is currently associated with SOME executor. Once `this` is added
// to the process's main executor (rclcpp::spin(node) in main()), a SyncParametersClient built on
// `this` fails as soon as it makes a blocking call (set_parameters/set_parameters_atomically/
// get_parameters) - those internally do executor_->add_node(this_node_base_interface) around the
// spin_until_future_complete() wait, and that add_node() throws
// "Node has already been added to an executor." because `this` is already flagged as associated
// with the main executor - REGARDLESS of whether executor_ is a brand new, otherwise-unused
// executor instance. Passing an explicit-but-fresh executor to SyncParametersClient (one candidate
// "simpler" fix) therefore does NOT solve the problem: the conflict is about the NODE's
// association state, not about which executor object is used. The only way to make a blocking
// SyncParametersClient call work from inside a node already being spun by its own executor is to
// route it through a genuinely different node that was never added anywhere - hence the helper
// node. This test proves both halves of that claim empirically against the actual installed
// rclcpp (not just by reading the header).

class RclcppInitEnvironment : public ::testing::Environment {
public:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

// Registered once, globally: gtest's default main() (linked in via ament_add_gtest/gtest_main)
// calls every registered Environment's SetUp()/TearDown() around RUN_ALL_TESTS(), so this test
// file needs no main() of its own.
::testing::Environment* const g_rclcpp_init_env =
    ::testing::AddGlobalTestEnvironment(new RclcppInitEnvironment);

TEST(ParamClientExecutorRegression, BlockingCallOnANodeAlreadyAddedToAnExecutorThrows) {
    auto main_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<rclcpp::Node>("test_node_already_on_executor");
    main_executor->add_node(node);  // simulates rclcpp::spin(node) already running in main()

    // A SyncParametersClient built directly on `node` (mirrors the pre-fix code that passed
    // `this`). No remote node/service needs to exist for this to reproduce the failure: the
    // "already added to an executor" error fires synchronously, at the START of the blocking
    // call (inside its own internal add_node()), before any RPC is attempted.
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "nonexistent_remote_node");
    EXPECT_THROW(
        client->set_parameters_atomically({rclcpp::Parameter("dummy", 1.0)}, std::chrono::milliseconds(200)),
        std::exception);
}

TEST(ParamClientExecutorRegression, BlockingCallOnAFreshHelperNodeDoesNotHitThatFailure) {
    // A node NEVER added to any executor - mirrors param_sync_helper_node_ in
    // ensureControllerParamClient(). It can still legitimately fail (there's no real remote
    // "cartesian_impedance_controller" node to answer, so this will time out), but that failure
    // must NOT be the "already added to an executor" one - proving the helper node avoids the
    // exact conflict test 1 reproduces.
    auto helper_node = std::make_shared<rclcpp::Node>("test_param_sync_helper");
    auto client = std::make_shared<rclcpp::SyncParametersClient>(helper_node, "nonexistent_remote_node");

    try {
        client->set_parameters_atomically({rclcpp::Parameter("dummy", 1.0)}, std::chrono::milliseconds(200));
        // No exception at all (e.g. some rclcpp versions return a default-constructed result on
        // timeout instead of throwing) is also an acceptable outcome here - the only thing this
        // test forbids is the specific "already added" failure.
    } catch (const std::exception& e) {
        const std::string what = e.what();
        EXPECT_EQ(what.find("already been added to an executor"), std::string::npos)
            << "fresh helper node hit the same executor conflict a real controller node would - "
               "the fix's whole premise (a never-added node avoids this) would be false. what(): "
            << what;
    }
}
