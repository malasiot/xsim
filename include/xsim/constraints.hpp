#ifndef XVIZ_PHYSICS_CONSTRAINTS_HPP
#define XVIZ_PHYSICS_CONSTRAINTS_HPP

#include <bullet/BulletDynamics/btBulletDynamicsCommon.h>

#include <xsim/rigid_body.hpp>

namespace xsim {

class Constraint {
public:
    btTypedConstraint *handle() const { return handle_.get() ; }
protected:

    std::shared_ptr<btTypedConstraint> handle_ ;
};

class Point2PointConstraint: public Constraint {

public:
    Point2PointConstraint(const RigidBodyPtr &b1, const RigidBodyPtr &b2, const Eigen::Vector3f &pivot1, const Eigen::Vector3f &pivot2) ;
};

class HingeConstraint: public Constraint {

public:
    HingeConstraint(const RigidBodyPtr &b1, const RigidBodyPtr &b2, const Eigen::Vector3f &anchor,
                    const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2) ;

    void enableMotor(uint idx, bool state = true) ;
    void setMaxMotorForce(uint idx, float val) ;
    void setTargetVelocity(uint idx, float val) ;
    void setParam(uint param, float v, uint idx) ;
    void setDamping(uint idx, float val) ;
    void setStiffness(uint idx, float val) ;
};


} // namespace xviz


#endif
