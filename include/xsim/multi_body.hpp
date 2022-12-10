#pragma once

#include <Eigen/Geometry>

#include <xsim/collision.hpp>
#include <xsim/convert.hpp>
#include <xsim/urdf_robot.hpp>

#include <bullet/BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <xsim/multi_body_builder.hpp>

#include <optional>

namespace xsim {

class PhysicsWorld ;
class Joint ;
class MultiBody ;

class Link: public CollisionObject {

public:

    Link & setLocalInertialFrame(const Eigen::Isometry3f &f) {
        local_inertial_frame_ = toBulletTransform(f);
        return *this ;
    }

    std::string getName() const override {
        return name_ ;
    }

    Eigen::Isometry3f getWorldTransform() const override ;

private:

    friend class MultiBody ;
    friend struct MultiBodyData ;
    friend class PhysicsWorld ;
    friend class Joint ;

    Link() {
        local_inertial_frame_.setIdentity() ;
    }

    MultiBody *parent_ ;
    float mass_ ;
    std::string name_ ;
    Link *parent_link_ = nullptr ;
    Joint *parent_joint_ = nullptr ;
    std::vector<Link *> child_links_ ;
    std::vector<Joint *> child_joints_ ;
    CollisionShapePtr shape_ ;
    xviz::NodePtr visual_ ;
    std::unique_ptr<btCollisionShape> proxy_ ;
    std::unique_ptr<btMultiBodyLinkCollider> collider_ ;

    std::optional<float> inertia_scaling_, stiffness_, damping_, lateral_friction_,
        rolling_friction_, spinning_friction_, restitution_ ;
    bool has_friction_anchor_ = false ;

    btVector3 inertia_ ;
    Eigen::Isometry3f origin_ ;
    btTransform local_inertial_frame_ ;
    int mb_index_ = -1 ;
};

enum MotorControlMode { POSITION_CONTROL, VELOCITY_CONTROL } ;

class MotorControl {
public:

    MotorControl(MotorControlMode mode): mode_(mode) {}

    MotorControl &setTargetVelocity(float tv) { target_velocity_ = tv ; return *this ; }
    MotorControl &setMaxVelocity(float mv) { max_velocity_ = mv ; return *this ; }
    MotorControl &setMaxForce(float f) { max_force_ = f ; return *this ; }
    MotorControl &setTargetPosition(float v) { target_pos_ = v ; return *this ; }
    MotorControl &setKp(float v) { kp_ = v ; return *this ; }
    MotorControl &setKd(float v) { kd_ = v ; return *this ; }

    std::optional<float> target_velocity_, max_velocity_, max_force_, kp_, kd_, target_pos_ ;
    MotorControlMode mode_ ;

};

class Joint {
public:

    Link *parentLink() const { return parent_link_ ; }
    Link *childLink() const { return child_link_ ; }

    float getLowerLimit() const { return lower_ ; }
    float getUpperLimit() const { return upper_ ; }
    float getMaxVelocity() const { return max_velocity_ ; }
    float getMaxForce() const { return max_force_ ; }

    bool hasMotor() const { return motor_ != nullptr ; }

    void setMotorMaxImpulse(float v) ;


    float getPosition() ;
    float getVelocity() ;
    float getTorque() ;

    void setPosition(float v) ;
    void setVelocity(float v) ;
    void setTorque(float v) ;

    void setTargetVelocity(float v, float kd = 0.1);
    void setTargetPosition(float v, float kp = 0.1);

    void setMotorControl(const MotorControl &c) ;

    const std::vector<Joint *> &getMimicJoints() const { return mimic_joints_; }
    float getMimicMultiplier() const { return mimic_multiplier_ ; }

private:

    friend class MultiBody ;
    friend struct MultiBodyData ;

    Joint() = default ;

    std::string name_ ;
    std:: string parent_, child_ ;
    Link *parent_link_ = nullptr, *child_link_ = nullptr ;

    JointType type_ ;
    btVector3 axis_ = {1, 0, 0};
    btTransform j2p_ ;
    float lower_ = 0.f, upper_ = 0.f, friction_ = 0.f, damping_ = 0.f, max_force_ = 0.f, max_velocity_ = 0.f ;
    btMultiBody *body_ = nullptr ;
    btMultiBodyJointMotor* motor_ = nullptr ;
    btScalar motor_max_force_ = btScalar(10.0) ;
    std::vector<Joint *> mimic_joints_  ;
    float mimic_multiplier_ = 1.f, mimic_offset_ = 0.f ;
    bool is_mimic_ = false ;
};

class MultiBody {
    MultiBody() = default ;
public:

    double getJointPosition(const std::string &name) const ;
    double getJointVelocity(const std::string &name) const ;
    double getJointTorque(const std::string &name) const  ;

    void setJointPosition(const std::string &name, double v) ;
    void setJointVelocity(const std::string &name, double v) ;
    void setJointTorque(const std::string &name, double v) ;
    void setJointMaxForce(const std::string &name, double v) ;

    void setTargetVelocity(const std::string &name, double v) ;
    void setTargetPosition(const std::string &name, double v) ;

    std::string name() const ;

    Joint *findJoint(const std::string &name) ;
    const Joint *findJoint(const std::string &name) const ;

    const std::vector<std::string> jointNames() const ;

    void getLinkTransforms(std::map<std::string, Eigen::Isometry3f> &names) const ;

    const Link *getLink(const std::string &name) const ;

private:

    friend class PhysicsWorld ;

    btMultiBody *handle() const { return body_.get() ; }

    void buildTree() ;

    void create(const MultiBodyBuilder &b, PhysicsWorld &physics);

    void buildCollisionObject(int link_idx, const btTransform &link_transform);

    void buildJoints(btMultiBodyDynamicsWorld *w, int link_idx, const btTransform &parent_transform_in_world_space);

    int findLink(const std::string &name) const ;

    int getJointIndex(const std::string &name) const;

    void setMimic(const URDFJoint &joint, Joint &j);

    void setUserIndex(uint idx) const ;

    void computeIndicesRecursive(Link *root, uint &count);

    void updateTransforms() ;

    std::vector<Link> links_ ;
    std::map<std::string, int> link_map_ ;
    std::map<std::string, Joint> joints_ ;
    std::unique_ptr<btMultiBody> body_ ;
    std::vector<std::unique_ptr<btMultiBodyConstraint>> constraints_ ;
    std::shared_ptr<MultiBodyStateObserver> ms_ ;
    std::string name_ ;
    Link *root_ = nullptr ;

};

using MultiBodyPtr = std::shared_ptr<MultiBody>;

} // namespace xviz

