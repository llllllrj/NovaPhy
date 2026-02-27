#include "novaphy/core/joint.h"

#include <cmath>

namespace novaphy {

Transform Joint::joint_transform(const float* q) const {
    switch (type) {
        case JointType::Revolute: {
            float angle = q[0];
            return Transform::from_axis_angle(axis, angle);
        }
        case JointType::Fixed:
            return Transform::identity();
        case JointType::Free: {
            // q = [px, py, pz, qx, qy, qz, qw]
            Vec3f pos(q[0], q[1], q[2]);
            Quatf rot(q[6], q[3], q[4], q[5]);  // Eigen: w,x,y,z
            rot.normalize();
            return Transform(pos, rot);
        }
        case JointType::Slide: {
            // q = [distance] — translation along axis
            Vec3f translation = axis.normalized() * q[0];
            return Transform::from_translation(translation);
        }
        case JointType::Ball: {
            // q = [qx, qy, qz, qw] — pure rotation
            Quatf rot(q[3], q[0], q[1], q[2]);  // Eigen: w,x,y,z
            rot.normalize();
            return Transform(Vec3f::Zero(), rot);
        }
    }
    return Transform::identity();
}

void Joint::motion_subspace(SpatialVector* S_cols) const {
    switch (type) {
        case JointType::Revolute:
            // Single column: rotation about axis [angular; 0]
            S_cols[0] = make_spatial(axis, Vec3f::Zero());
            break;
        case JointType::Fixed:
            // No columns
            break;
        case JointType::Free:
            // 6 columns: identity (angular first, then linear)
            for (int i = 0; i < 6; ++i) {
                S_cols[i] = SpatialVector::Zero();
                S_cols[i](i) = 1.0f;
            }
            break;
        case JointType::Slide:
            // Single column: translation along axis [0; axis]
            S_cols[0] = make_spatial(Vec3f::Zero(), axis.normalized());
            break;
        case JointType::Ball:
            // 3 columns: angular identity [e_i; 0]
            S_cols[0] = make_spatial(Vec3f(1, 0, 0), Vec3f::Zero());
            S_cols[1] = make_spatial(Vec3f(0, 1, 0), Vec3f::Zero());
            S_cols[2] = make_spatial(Vec3f(0, 0, 1), Vec3f::Zero());
            break;
    }
}

}  // namespace novaphy
