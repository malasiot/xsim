#ifndef XSIM_PHYSICS_MULTI_BODY_BUILDER_HPP
#define XSIM_PHYSICS_MULTI_BODY_BUILDER_HPP

#include <Eigen/Geometry>
#include <xsim/collision.hpp>
#include <xsim/urdf_robot.hpp>
#include <xviz/scene/material.hpp>

namespace xviz {
class Node ;
using NodePtr = std::shared_ptr<Node> ;
}

namespace xsim {

class MultiBody ;
using MultiBodyPtr = std::shared_ptr<MultiBody> ;
class PhysicsWorld ;


enum JointType { RevoluteJoint, ContinuousJoint, PrismaticJoint, FixedJoint, SphericalJoint, FloatingJoint, PlanarJoint } ;

class MBLink {

public:

    MBLink & setLocalInertialFrame(const Eigen::Isometry3f &f) {
        local_inertial_frame_ = f ;
        return *this ;
    }

    MBLink & setInertia(const Eigen::Vector3f &f) {
        inertia_ = f ;
        return *this ;
    }

    MBLink & setInertiaScaling(float f) {
        inertia_scaling_ = f ;
        return *this ;
    }

    MBLink & setLateralFriction(float f) {
        lateral_friction_ = f ;
        return *this ;
    }

    MBLink & setRollingFriction(float f) {
        rolling_friction_ = f ;
        return *this ;
    }

    MBLink & setSpinningFriction(float f) {
        spinning_friction_ = f ;
        return *this ;
    }

    MBLink & setStiffnessAndDamping(float s, float d) {
        stiffness_ = s ;
        damping_ = d ;
        return *this ;
    }

    MBLink & setRestitution(float f) {
        restitution_ = f ;
        return *this ;
    }

    MBLink & enableFrictionAnchor() {
        has_friction_anchor_ = true ;
        return *this ;
    }


    MBLink &setVisualShape(const xviz::NodePtr &vs) {
        visual_ = vs ;
        return *this ;
    }

    MBLink &makeVisualShape(const xviz::MaterialPtr &mat) {
        visual_ = makeVisualShapeFromCollisionShape(shape_, mat) ;
        return *this ;
    }

    MBLink &makeVisualShape(const Eigen::Vector4f &clr) {
        xviz::MaterialPtr mat(new xviz::PhongMaterial(clr)) ;
        visual_ = makeVisualShapeFromCollisionShape(shape_, mat) ;
        return *this ;
    }

private:

    friend class MBJoint ;
    friend class MultiBody ;
    friend class MultiBodyBuilder ;

    float mass_ ;
    std::string name_ ;
    CollisionShapePtr shape_ ;
    xviz::NodePtr visual_ ;

    std::optional<Eigen::Vector3f> inertia_ ;
    Eigen::Isometry3f origin_, local_inertial_frame_ = Eigen::Isometry3f::Identity() ;
    std::optional<float> inertia_scaling_, stiffness_, damping_, lateral_friction_,
    rolling_friction_, spinning_friction_, restitution_ ;
    bool has_friction_anchor_ = false ;
};

class MBJoint {
public:

    MBJoint& setAxis(const Eigen::Vector3f &axis) {
         axis_ = axis ;
         return *this ;
    }

    MBJoint& setLimits(float l, float u) {
         lower_ = l ;
         upper_ = u ;
         return *this ;
    }

    MBJoint& setFriction(float f) {
        friction_ = f ;
        return *this ;
    }

    MBJoint& setDamping(float d) {
        damping_ = d ;
        return *this ;
    }

    MBJoint& setMaxVelocity(float v) {
        max_velocity_ = v ;
        return *this ;
    }

    MBJoint& setMaxForce(float v) {
        max_force_ = v ;
        return *this ;
    }

private:

    friend class MultiBody ;
    friend class MultiBodyBuilder ;

    MBJoint() = default ;

    std::string name_ ;
    std:: string parent_, child_, mimic_joint_ ;
    JointType type_ ;

    Eigen::Vector3f axis_ = {1, 0, 0};
    Eigen::Isometry3f j2p_ ;
    float lower_ = 0.f, upper_ = 0.f, friction_ = 0.f, damping_ = 0.f, max_force_ = 0.f, max_velocity_ = 0.f ;
    float motor_max_force_ = 1000.0f ;
    float mimic_multiplier_ = 1.f, mimic_offset_ = 0.f ;
};

class MultiBodyStateObserver {
public:
    virtual void notify(const std::map<std::string, Eigen::Isometry3f> &link_transforms) = 0;
};

class MultiBodyUpdateScene: public MultiBodyStateObserver {
    public:

    MultiBodyUpdateScene(const xviz::NodePtr &node): node_(node) {}

    void notify(const std::map<std::string, Eigen::Isometry3f> &link_transforms) override ;

private:
    xviz::NodePtr node_ ;
};

class MultiBodyBuilder {
    public:

    MultiBodyBuilder() = default ;

    MultiBodyBuilder(const URDFRobot &r) {
        loadURDF(r) ;
    }

    MultiBodyBuilder &setName(const std::string &name) { name_ = name ; return *this ; }

    MBLink &addLink(const std::string &name, float mass, CollisionShapePtr cshape, const Eigen::Isometry3f &origin = Eigen::Isometry3f::Identity());

    MBJoint &addJoint(const std::string &name, JointType type, const std::string &parent, const std::string &child, const Eigen::Isometry3f &j2p);

    MBLink *findLink(const std::string &name) ;

    MultiBodyBuilder &setFixedBase() {
        fixed_base_ = true ;
        return *this ;
    }

    MultiBodyBuilder &setWorldTransform(const Eigen::Isometry3f &tr) {
        base_tr_ = tr ;
        return *this ;
    }

    MultiBodyBuilder &setLinearDamping(float v) {
        linear_damping_ = v ;
        return *this ;
    }

    MultiBodyBuilder &setAngularDamping(float v) {
        angular_damping_ = v ;
        return *this ;
    }

    MultiBodyBuilder &loadURDF(const URDFRobot &rb);

    MultiBodyBuilder &loadURDF(const std::string &name) ;

    MultiBodyBuilder &setMotionStateObserver(MultiBodyStateObserver *obj) {
        ms_.reset(obj) ;
        return *this ;
    }

    MultiBodyBuilder &setMargin(float m) {
        collision_margin_ = m ;
        return *this ;
    }

private:

    friend class MultiBody ;

    std::string name_ ;
    bool fixed_base_ = false ;
    Eigen::Isometry3f base_tr_ = Eigen::Isometry3f::Identity() ;
    std::optional<float> linear_damping_, angular_damping_ ;
    float collision_margin_ = 0.001 ;

    std::vector<MBLink> links_ ;
    std::vector<MBJoint> joints_ ;

    std::shared_ptr<MultiBodyStateObserver> ms_ ;

    void setMimic(const URDFJoint &joint, MBJoint &j);
};

}

#endif
