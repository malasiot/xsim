#include <xsim/constraints.hpp>
#include <xsim/convert.hpp>

namespace xsim {

Point2PointConstraint::Point2PointConstraint(const RigidBodyPtr &b1, const RigidBodyPtr &b2, const Eigen::Vector3f &pivot1, const Eigen::Vector3f &pivot2) {
    handle_.reset(new btPoint2PointConstraint (*b1->handle(), *b2->handle(), eigenVectorToBullet(pivot1), eigenVectorToBullet(pivot2))) ;
}

HingeConstraint::HingeConstraint(const RigidBodyPtr &b1, const RigidBodyPtr &b2, const Eigen::Vector3f &anchor, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2)
{
    btVector3 a(eigenVectorToBullet(anchor)), ax1(eigenVectorToBullet(axis1)), ax2(eigenVectorToBullet(axis2)) ;
    handle_.reset(new btHinge2Constraint (*b1->handle(), *b2->handle(), a, ax1, ax2)) ;
}

void HingeConstraint::enableMotor(uint idx, bool state) {
    auto handle = reinterpret_cast<btHinge2Constraint *>(handle_.get()) ;
    handle->enableMotor(idx, state);
}

void HingeConstraint::setMaxMotorForce(uint idx, float val) {
    auto handle = reinterpret_cast<btHinge2Constraint *>(handle_.get()) ;
    handle->setMaxMotorForce(idx, val);
}

void HingeConstraint::setTargetVelocity(uint idx, float val) {
    auto handle = reinterpret_cast<btHinge2Constraint *>(handle_.get()) ;
    handle->setTargetVelocity(idx, val);
}

void HingeConstraint::setParam(uint param, float val, uint idx) {
    auto handle = reinterpret_cast<btHinge2Constraint *>(handle_.get()) ;
    handle->setParam(param, val, idx);
}

void HingeConstraint::setStiffness(uint idx, float val) {
    auto handle = reinterpret_cast<btHinge2Constraint *>(handle_.get()) ;
    handle->setStiffness(idx, val);
}

void HingeConstraint::setDamping(uint idx, float val) {
    auto handle = reinterpret_cast<btHinge2Constraint *>(handle_.get()) ;
    handle->setDamping(idx, val);
}

}
