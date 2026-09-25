#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

#include <cmath>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {

namespace {
constexpr double kMinNorm = 1e-9;
}

CubeSatelliteModel::CubeSatelliteModel(Params params) : params_(std::move(params)) {
    if (!(params_.axis_world.norm() > kMinNorm)) {
        throw std::invalid_argument("CubeSatelliteModel: axis_world has ~zero norm.");
    }
    if (!(params_.face_normal_body.norm() > kMinNorm)) {
        throw std::invalid_argument("CubeSatelliteModel: face_normal_body has ~zero norm.");
    }
    params_.axis_world.normalize();
    params_.face_normal_body.normalize();
    if (std::abs(params_.axis_world.dot(params_.face_normal_body)) > kLateralTolerance) {
        throw std::invalid_argument(
            "CubeSatelliteModel: face_normal_body is not lateral "
            "(not perpendicular to axis_world).");
    }
    if (!(params_.wall_thickness_m > 0.0)) {
        throw std::invalid_argument("CubeSatelliteModel: wall_thickness_m must be > 0.");
    }
    if (!(params_.wall_thickness_m < params_.collar_radius_m)) {
        throw std::invalid_argument(
            "CubeSatelliteModel: wall_thickness_m must be < collar_radius_m.");
    }
}

Eigen::Matrix3d CubeSatelliteModel::rotation(double theta_rad) const {
    return Eigen::AngleAxisd(theta_rad, params_.axis_world).toRotationMatrix();
}

Eigen::Vector3d CubeSatelliteModel::radialDirectionBody(GraspPointId k) const {
    const Eigen::Vector3d u = params_.face_normal_body.cross(params_.axis_world).normalized();
    const Eigen::Vector3d& v = params_.axis_world;
    const double phi = static_cast<int>(k) * M_PI / 2.0;
    return std::cos(phi) * u + std::sin(phi) * v;
}

Eigen::Vector3d CubeSatelliteModel::faceNormalWorld(double theta_rad) const {
    return rotation(theta_rad) * params_.face_normal_body;
}

Eigen::Vector3d CubeSatelliteModel::radialDirectionWorld(GraspPointId k, double theta_rad) const {
    return rotation(theta_rad) * radialDirectionBody(k);
}

GraspTargetPose CubeSatelliteModel::graspPoseAt(GraspPointId k, double theta_rad) const {
    const Eigen::Matrix3d R = rotation(theta_rad);
    const Eigen::Vector3d r_k = radialDirectionBody(k);
    const Eigen::Vector3d p_body =
        (params_.cube_side_m / 2.0 + params_.standoff_m) * params_.face_normal_body +
        params_.collar_radius_m * r_k;

    const Eigen::Vector3d z_tcp = -(R * params_.face_normal_body);
    const Eigen::Vector3d y_tcp = -(R * r_k);
    const Eigen::Vector3d x_tcp = y_tcp.cross(z_tcp);
    Eigen::Matrix3d frame;
    frame.col(0) = x_tcp;
    frame.col(1) = y_tcp;
    frame.col(2) = z_tcp;

    GraspTargetPose pose;
    pose.position_world = params_.center_world + R * p_body;
    pose.orientation_nominal_world = Eigen::Quaterniond(frame).normalized();
    pose.approach_axis_world = z_tcp;
    return pose;
}

double CubeSatelliteModel::cylinderOuterRadius() const { return params_.collar_radius_m; }

double CubeSatelliteModel::cylinderInnerRadius() const {
    return params_.collar_radius_m - params_.wall_thickness_m;
}

Eigen::Vector3d CubeSatelliteModel::cylinderAxisWorld(double theta_rad) const {
    return faceNormalWorld(theta_rad);
}

Eigen::Vector3d CubeSatelliteModel::cylinderBaseCenterWorld(double theta_rad) const {
    // On the face itself (no standoff_m, unlike p_body in graspPoseAt).
    const Eigen::Vector3d base_body = (params_.cube_side_m / 2.0) * params_.face_normal_body;
    return params_.center_world + rotation(theta_rad) * base_body;
}

Eigen::Vector3d CubeSatelliteModel::cylinderTopCenterWorld(double theta_rad) const {
    return cylinderBaseCenterWorld(theta_rad) + params_.standoff_m * cylinderAxisWorld(theta_rad);
}

}  // namespace core
}  // namespace haptic_dmp_learning
