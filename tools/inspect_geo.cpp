#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "franka_cartesian_control/core/robot_model.hpp"

using RobotModel = franka_cartesian_control::core::RobotModel;

int main() {
    std::string urdf_path = "/root/thesis_ws/fer_flat_effort.urdf";
    std::ifstream f(urdf_path);
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    RobotModel model(buffer.str(), joint_names, "fer_hand_tcp");

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483;
    model.update(q0, RobotModel::JointVector::Zero());

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "EE Pos at q0: " << model.eePosition().transpose() << std::endl;
    Eigen::Matrix3d R0 = model.eeOrientation().toRotationMatrix();
    std::cout << "EE R0 col(0) (X_ee): " << R0.col(0).transpose() << std::endl;
    std::cout << "EE R0 col(1) (Y_ee): " << R0.col(1).transpose() << std::endl;
    std::cout << "EE R0 col(2) (Z_ee): " << R0.col(2).transpose() << std::endl;
    std::cout << "EE quat0 (x,y,z,w): " << model.eeOrientation().coeffs().transpose() << std::endl;

    Eigen::Vector3d c(0.45, -0.05, 0.35);
    Eigen::Vector3d p_grasp0(0.3928, -0.1150, 0.2988);
    Eigen::Vector3d r0 = p_grasp0 - c;
    Eigen::Vector3d r_hat0 = r0.normalized();
    std::cout << "c: " << c.transpose() << std::endl;
    std::cout << "p_grasp0: " << p_grasp0.transpose() << std::endl;
    std::cout << "r0: " << r0.transpose() << " (norm = " << r0.norm() << ")" << std::endl;
    std::cout << "r_hat0 (true radial unit vector at theta=0): " << r_hat0.transpose() << std::endl;

    Eigen::Vector3d z_app_yesterday(0.0, 0.0, -1.0);
    double dot_val = r_hat0.dot(z_app_yesterday);
    double angle_deg = std::acos(std::max(-1.0, std::min(1.0, dot_val))) * 180.0 / M_PI;
    std::cout << "Angle between r_hat0 and [0,0,-1]: " << angle_deg << " deg" << std::endl;

    return 0;
}
