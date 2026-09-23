#include <gtest/gtest.h>
#include "franka_cartesian_control/ros/ros_utils.hpp"
#include <cmath>

using franka_cartesian_control::ros_wrapper::FrameAligner;

/**
 * @brief Without initial_alignment_position_override, behavior must be EXACTLY what it was
 * before that override existed: the offset is deduced from the first raw_pos passed to align().
 */
TEST(FrameAlignerTest, NoOverrideDeducesOffsetFromFirstRawPos) {
    FrameAligner aligner;
    const Eigen::Vector3d activation_pos(0.5, 0.1, 0.4);
    const Eigen::Quaterniond activation_quat = Eigen::Quaterniond::Identity();
    aligner.reset(activation_pos, activation_quat);

    const Eigen::Vector3d first_raw_pos(0.02, -0.01, 0.0);
    const Eigen::Quaterniond raw_quat = Eigen::Quaterniond::Identity();

    Eigen::Vector3d aligned_pos;
    Eigen::Quaterniond aligned_quat;
    aligner.align(first_raw_pos, raw_quat, aligned_pos, aligned_quat);

    // position_offset_ = activation_ee_position_ - raw_pos (the FIRST one received) -> aligned_pos
    // for that same first sample must land exactly on activation_pos.
    EXPECT_TRUE(aligned_pos.isApprox(activation_pos, 1e-12));

    // A later sample at, say, first_raw_pos + delta must be offset by that same delta.
    const Eigen::Vector3d delta(0.01, 0.02, -0.005);
    Eigen::Vector3d aligned_pos2;
    Eigen::Quaterniond aligned_quat2;
    aligner.align(first_raw_pos + delta, raw_quat, aligned_pos2, aligned_quat2);
    EXPECT_TRUE(aligned_pos2.isApprox(activation_pos + delta, 1e-12));
}

/**
 * @brief With initial_alignment_position_override set before the first align(), the offset must
 * be computed from activation_ee_position_ - override, ignoring the first raw_pos entirely - this
 * is what eliminates the dependency on which sample the 1 kHz RealtimeBuffer read happens to see
 * first when the publisher runs at a different rate.
 */
TEST(FrameAlignerTest, OverrideIgnoresFirstRawPosForTheOffset) {
    FrameAligner aligner;
    const Eigen::Vector3d activation_pos(0.5, 0.1, 0.4);
    const Eigen::Quaterniond activation_quat = Eigen::Quaterniond::Identity();
    aligner.reset(activation_pos, activation_quat);

    const Eigen::Vector3d expected_initial_pos(0.0, 0.0, 0.0);  // e.g. the DMP's known init_pos_
    aligner.setInitialAlignmentPositionOverride(expected_initial_pos);

    // The first raw_pos the buffer actually hands us is NOT expected_initial_pos - simulating the
    // 200 Hz/1 kHz mismatch reading a later-than-true-first sample first.
    const Eigen::Vector3d misaligned_first_raw_pos(0.0045, 0.0, 0.0);  // ~4.5 mm off, per the bug report
    const Eigen::Quaterniond raw_quat = Eigen::Quaterniond::Identity();

    Eigen::Vector3d aligned_pos;
    Eigen::Quaterniond aligned_quat;
    aligner.align(misaligned_first_raw_pos, raw_quat, aligned_pos, aligned_quat);

    // Offset must be activation_pos - expected_initial_pos, i.e. activation_pos here since
    // expected_initial_pos is the origin - NOT activation_pos - misaligned_first_raw_pos.
    const Eigen::Vector3d expected_offset = activation_pos - expected_initial_pos;
    const Eigen::Vector3d expected_aligned_pos = expected_offset + misaligned_first_raw_pos;
    EXPECT_TRUE(aligned_pos.isApprox(expected_aligned_pos, 1e-12));
    EXPECT_FALSE(aligned_pos.isApprox(activation_pos, 1e-6));  // would only hold without the bug

    // Once captured, later samples use the SAME offset (override only affects the FIRST capture).
    const Eigen::Vector3d delta(0.01, -0.02, 0.03);
    Eigen::Vector3d aligned_pos2;
    Eigen::Quaterniond aligned_quat2;
    aligner.align(misaligned_first_raw_pos + delta, raw_quat, aligned_pos2, aligned_quat2);
    EXPECT_TRUE(aligned_pos2.isApprox(expected_aligned_pos + delta, 1e-12));
}

/**
 * @brief clearInitialAlignmentPositionOverride() reverts to first-raw_pos deduction.
 */
TEST(FrameAlignerTest, ClearingOverrideRevertsToFirstRawPosDeduction) {
    FrameAligner aligner;
    const Eigen::Vector3d activation_pos(0.2, 0.0, 0.3);
    aligner.reset(activation_pos, Eigen::Quaterniond::Identity());

    aligner.setInitialAlignmentPositionOverride(Eigen::Vector3d(1.0, 1.0, 1.0));
    aligner.clearInitialAlignmentPositionOverride();

    const Eigen::Vector3d first_raw_pos(0.05, 0.05, 0.05);
    Eigen::Vector3d aligned_pos;
    Eigen::Quaterniond aligned_quat;
    aligner.align(first_raw_pos, Eigen::Quaterniond::Identity(), aligned_pos, aligned_quat);

    EXPECT_TRUE(aligned_pos.isApprox(activation_pos, 1e-12));
}

/**
 * @brief reset() must NOT clear a previously-set override - it is caller-set state independent of
 * the activate/deactivate cycle (see setInitialAlignmentPositionOverride() doc comment).
 */
TEST(FrameAlignerTest, ResetDoesNotClearThePreviouslySetOverride) {
    FrameAligner aligner;
    aligner.reset(Eigen::Vector3d(0.5, 0.1, 0.4), Eigen::Quaterniond::Identity());
    aligner.setInitialAlignmentPositionOverride(Eigen::Vector3d(0.0, 0.0, 0.0));

    // Simulate a deactivate/reactivate cycle at a different physical EE pose.
    const Eigen::Vector3d new_activation_pos(0.6, -0.2, 0.35);
    aligner.reset(new_activation_pos, Eigen::Quaterniond::Identity());

    const Eigen::Vector3d some_raw_pos(0.01, 0.01, 0.01);
    Eigen::Vector3d aligned_pos;
    Eigen::Quaterniond aligned_quat;
    aligner.align(some_raw_pos, Eigen::Quaterniond::Identity(), aligned_pos, aligned_quat);

    // Offset must still come from the override (0,0,0), not from some_raw_pos.
    const Eigen::Vector3d expected_aligned_pos = new_activation_pos + some_raw_pos;
    EXPECT_TRUE(aligned_pos.isApprox(expected_aligned_pos, 1e-12));
}
