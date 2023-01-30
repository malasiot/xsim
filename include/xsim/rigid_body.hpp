#ifndef XSIM_PHYSICS_RIGID_BODY_HPP
#define XSIM_PHYSICS_RIGID_BODY_HPP

#include <memory>
#include <xviz/scene/scene_fwd.hpp>

#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>

#include <xsim/collision.hpp>
#include <xsim/convert.hpp>
#include <xviz/scene/material.hpp>

namespace xsim {

struct RigidBodyData ;

class RigidBodyStateObserver {
public:
    virtual void notify(const Eigen::Isometry3f &tr) = 0;
};

class RigidBodyUpdateScene: public RigidBodyStateObserver {
    public:

    RigidBodyUpdateScene(const xviz::NodePtr &node): node_(node) {}

    void notify(const Eigen::Isometry3f &tr) override ;

private:
    xviz::NodePtr node_ ;
};

class RigidBodyBuilder {
public:

    RigidBodyBuilder &setName(const std::string &name) {
        name_ = name ;
        return *this ;
    }

    RigidBodyBuilder &setMotionStateObserver(RigidBodyStateObserver *ob) {
        ms_ = ob ;
        return *this ;
    }

    RigidBodyBuilder &setCollisionShape(const CollisionShapePtr &shape) {
        collision_shape_ = shape ;
        return *this ;
    }

    RigidBodyBuilder &setVisualShape(const xviz::NodePtr &shape) {
        visual_shape_ = shape ;
        return *this ;
    }

    // create visual shape with same geometry as collision shape and given material
    // (only for simple shapes)
    RigidBodyBuilder &makeVisualShape(const xviz::MaterialPtr &mat) {
        visual_shape_ = makeVisualShapeFromCollisionShape(collision_shape_, mat) ;
        return *this ;
    }

    RigidBodyBuilder &makeVisualShape(const Eigen::Vector4f &clr) {
        xviz::MaterialPtr mat(new xviz::PhongMaterial(clr)) ;
        visual_shape_ = makeVisualShapeFromCollisionShape(collision_shape_, mat) ;
        return *this ;
    }

    RigidBodyBuilder &setMass(float mass) {
        mass_ = mass ;
        return *this ;
    }

    RigidBodyBuilder &setLocalInertia(const Eigen::Vector3f &inertia) {
        local_inertia_ = inertia ;
        return *this ;
    }

    RigidBodyBuilder &setWorldTransform(const Eigen::Isometry3f &tr) {
        world_transform_ = tr ;
        return *this ;
    }

    RigidBodyBuilder &setFriction(float f) {
        friction_ = f ;
        return *this ;
    }

    RigidBodyBuilder &setLinearDamping(float f) {
        linear_damping_ = f ;
        return *this ;
    }

    RigidBodyBuilder &setAngularDamping(float f) {
        angular_damping_ = f ;
        return *this ;
    }

    RigidBodyBuilder &setRollingFriction(float f) {
        rolling_friction_ = f ;
        return *this ;
    }

    RigidBodyBuilder &setSpinningFriction(float f) {
        spinning_friction_ = f ;
        return *this ;
    }

    RigidBodyBuilder &setRestitution(float f) {
        restitution_ = f ;
        return *this ;
    }

    RigidBodyBuilder &setGravity(const Eigen::Vector3f &g) {
        gravity_ = g ;
        return *this ;
    }

private:


    friend class RigidBody ;

    std::string name_ ;
    RigidBodyStateObserver *ms_ = nullptr ;
    CollisionShapePtr collision_shape_ ;
    xviz::NodePtr visual_shape_ ;
    float mass_ = 0.0f ;
    std::optional<Eigen::Vector3f> local_inertia_, gravity_;
    std::optional<float> friction_, linear_damping_, angular_damping_,
        rolling_friction_, spinning_friction_, restitution_ ;

    Eigen::Isometry3f world_transform_ = Eigen::Isometry3f::Identity();
};

class RigidBody: public CollisionObject {
public :
    // dynamic body with given inertia

    btRigidBody *handle() const;

    void setName(const std::string &name);

    std::string getName() const override {
        return name_ ;
    }

    Eigen::Isometry3f getWorldTransform() const override ;
    void setWorldTransform(const Eigen::Isometry3f &tr) ;

    void disableDeactivation();
    void setActive();

    void applyExternalForce(const Eigen::Vector3f &force, const Eigen::Vector3f &pos) ;
    void applyExternalTorque(const Eigen::Vector3f &torque) ;
    void applyCentralImpulse(const Eigen::Vector3f &imp) ;

    void setContactProcessingThreshold(float t);
    void setCCDSweptSphereRadius(float r);

    void setLinearVelocity(const Eigen::Vector3f &v) ;
    void setAngularVelocity(const Eigen::Vector3f &v) ;
    void clearForces() ;
private:

    RigidBody(const RigidBodyBuilder &rb) ;

    friend class PhysicsWorld ;
    friend class MotionState ;

    std::string name_ ;
    std::unique_ptr<btRigidBody> handle_ ;
    CollisionShapePtr collision_shape_ ;
    xviz::NodePtr visual_shape_ ;
    std::unique_ptr<btMotionState> motion_state_ ;
};

using RigidBodyPtr = std::shared_ptr<RigidBody> ;

} // namespace xviz


#endif
