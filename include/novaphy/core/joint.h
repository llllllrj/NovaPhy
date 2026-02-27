#pragma once

#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/// Joint types for articulated bodies (MuJoCo-compatible naming)
enum class JointType {
    Revolute,  // 1 DOF: rotation about a single axis (MuJoCo: hinge)
    Fixed,     // 0 DOF: rigid attachment (MuJoCo: weld)
    Free,      // 6 DOF: free-floating (MuJoCo: free)
    Slide,     // 1 DOF: translation along an axis (MuJoCo: slide)
    Ball,      // 3 DOF: spherical rotation (MuJoCo: ball)
};

/// Joint definition for an articulated body link.
struct Joint {
    JointType type = JointType::Revolute;
    Vec3f axis = Vec3f(0, 0, 1);  // rotation/translation axis (for revolute/slide)
    int parent = -1;               // parent link index (-1 = world)

    /// Transform from parent body frame to joint frame (constant offset)
    Transform parent_to_joint = Transform::identity();

    /// Number of position DOFs for this joint type
    int num_q() const {
        switch (type) {
            case JointType::Revolute: return 1;
            case JointType::Fixed:    return 0;
            case JointType::Free:     return 7;  // pos(3) + quat(4)
            case JointType::Slide:    return 1;
            case JointType::Ball:     return 4;  // quat(4): [qx, qy, qz, qw]
        }
        return 0;
    }

    /// Number of velocity DOFs for this joint type
    int num_qd() const {
        switch (type) {
            case JointType::Revolute: return 1;
            case JointType::Fixed:    return 0;
            case JointType::Free:     return 6;  // angular(3) + linear(3)
            case JointType::Slide:    return 1;
            case JointType::Ball:     return 3;  // angular(3)
        }
        return 0;
    }

    /// Compute the joint transform X_J(q) given joint coordinates.
    /// For revolute: rotation about axis by angle q[0]
    /// For fixed: identity
    /// For free: translation + rotation from q
    /// For slide: translation along axis by q[0]
    /// For ball: rotation from quaternion q[0:4]
    Transform joint_transform(const float* q) const;

    /// Compute the motion subspace matrix S (6 x nv).
    /// Returns columns as spatial vectors.
    /// For revolute: single column [axis; 0]
    /// For free: 6x6 identity
    /// For slide: single column [0; axis]
    /// For ball: 3 columns [I_3x3; 0]
    void motion_subspace(SpatialVector* S_cols) const;
};

}  // namespace novaphy
