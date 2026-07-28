#pragma once
#include <Eigen/Dense>

namespace haptic_dmp_learning {
namespace core {

// A single recorded sample of the master device pose during a demonstration.
// Position only for now (simple, translational DMP) - orientation will be
// added later with a separate QuaternionDMP class, kept out of this struct
// on purpose to avoid coupling the two.
struct Sample {
    double t;                  // seconds, relative to the start of the demo
    Eigen::Vector3d position;  // meters, expressed in the Geomagic Touch's own frame
};

}  // namespace core
}  // namespace haptic_dmp_learning
