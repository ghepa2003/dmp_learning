# One-shot output of satellite_grasp_planner's grasp_planner_node (Task 1 of 2 - the executor that
# consumes this message is a separate, later task). Published ONCE per node lifetime, QoS reliable +
# transient_local + depth 1.

std_msgs/Header header                 # stamp = node's own SIM clock (this->get_clock()->now()) read
                                        # right after selection ends; frame_id "world". NOT
                                        # planGraspAndLaunch()'s internal t_now_s - see
                                        # provisional_delay_s below for that value's own caveat.

uint8 STATUS_UNSET = 0
uint8 STATUS_SELECTED = 1
uint8 STATUS_NO_FEASIBLE_CANDIDATE = 2
uint8 STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE = 3
uint8 status

# Meaningful ONLY when status == STATUS_SELECTED. A zero-initialized (or STATUS_UNSET) message must
# never be read as a valid selection.
geometry_msgs/Pose goal_pose
uint8 k                                 # GraspPointId: 0=kP0, 1=kP90, 2=kP180, 3=kP270
float64 theta_star_rad
float64 psi_rad

float64 contact_time_s                  # GraspLaunchResult::tau_launch_s VERBATIM - no executor "+dt"
                                        # correction (that convention belongs to whoever consumes this
                                        # field, not to the planner). 0.0 if not SELECTED.
float64 omega_rad_s                     # signed, from the snapshot used for selection. Always set.

# DIAGNOSTIC ONLY - do not use this to time a launch. Computed from planGraspAndLaunch()'s default
# t_now_s = snapshot.t_s + WALL-CLOCK elapsed_s, which is wrong under a Gazebo real-time-factor != 1.
# The consuming executor triggers on PHASE, never on this value: theta_trig = theta* - omega*contact_time_s,
# brought back to (-pi, pi] with wrapPi.
float64 provisional_delay_s             # launch.delay_s if SELECTED else 0.0

# Selection diagnostics - always populated regardless of status.
int32 rows_evaluated
int32 total_rollouts
float64 selection_elapsed_s
uint8 STOP_UNSET = 0
uint8 STOP_COMPLETED = 1
uint8 STOP_BOUND_SATISFIED = 2
uint8 STOP_MAX_ROWS_REACHED = 3
uint8 STOP_TIME_BUDGET_EXHAUSTED = 4
uint8 stop_reason
bool bound_violated
float64 max_w_hat_seen
# Total cost (GraspCostBreakdown::total, to be MINIMIZED) of the selected candidate. Meaningful ONLY
# when status == STATUS_SELECTED; 0.0 otherwise - 0.0 is a legitimate cost value too, so read status
# first, never infer "selected" from this field.
float64 best_total_cost

# Meaningful ONLY when status == STATUS_SELECTED; zero otherwise (0,0,0 is a legitimate point too, so
# read status first). p0 the planner used for its rollouts (scan.p0 = forward kinematics at q0), so a
# consumer can reproduce them. Expressed in the WORLD frame; it equals the robot-base frame only
# because robot_base_world = 0, a constraint the node enforces at startup (checkRobotBaseWorldIsZero).
geometry_msgs/Point start_position

# Snapshot the selection was run on. ALWAYS set, whatever the status. theta in the planner's theta
# convention (see DESIGN_NOTES.md); t = SIM clock, from the odometry msg->header.stamp of that sample
# (not the header.stamp above, which is read after selection ends).
float64 snapshot_theta_rad
float64 snapshot_t_s

# SHA-256 (hex) of the weights file the planner loaded the template from. ALWAYS set. It is the hash
# recorded in demo_params (weights_sha256), which buildSelectionInputs checks against the actual hash
# of the file and refuses to start on mismatch: it is therefore the hash of the file really read.
string weights_sha256
