#ifndef XSIM_KINEMATIC_HPP
#define XSIM_KINEMATIC_HPP

#include <memory>

#include <xsim/urdf_robot.hpp>

namespace xsim {

using JointState = std::map<std::string, double> ;

class KinematicLink ;

using KinematicLinkPtr = std::shared_ptr<KinematicLink> ;

class KinematicJoint {
public:
    enum JointType { REVOLUTE_JOINT, PRISMATIC_JOINT, FIXED_JOINT, CONTINUOUS_JOINT };
    enum LimitCheckType {  LIMIT_CHECK_THROW_EXCEPTION, LIMIT_CHECK_CLAMP_VALUE, LIMIT_CHECK_IGNORE } ;

    KinematicJoint() = default ;
    virtual ~KinematicJoint() {}

    // get the value corresponding to a dof of this joint
    virtual double getPosition() const = 0;
    // set the value corresponding to a dof of this joint
    virtual void setPosition(double val, LimitCheckType lt = LIMIT_CHECK_CLAMP_VALUE) = 0;
    // chack if this value is within the allowable limits of the joint
    virtual bool allows(double value) const { return true ; }
    // update joint transform matrix based on set values
    virtual void updateLocalTransform() = 0;

    virtual void getLimits(double &l, double &u) const = 0 ;
    // compute the distance between dofs
    virtual double getDistance(double val1, double val2) const = 0 ;

    virtual bool hasDOF() const { return true ; }

    virtual JointType getType() const = 0 ;

    const std::string &name() const { return name_ ; }
    const KinematicLinkPtr parentLink() const { return parent_ ; }
    const KinematicLinkPtr childLink() const { return child_ ; }
    const Eigen::Isometry3f &origin() const { return origin_ ; }
    const Eigen::Isometry3f &local() const { return local_ ; }

protected:
    friend class KinematicModel ;
    friend class KinematicLink ;

    void updateTransforms() ;
    void updateLinkTransforms() ;

    std::string name_ ;
    KinematicLinkPtr parent_, child_ ;

    Eigen::Isometry3f origin_ ; // joint frame transform
    Eigen::Isometry3f local_;  // dof transform
};

using KinematicJointPtr = std::shared_ptr<KinematicJoint> ;

class KinematicLink {
public:
    KinematicLink() = default ;

protected:

    friend class KinematicModel ;
    friend class KinematicJoint ;

    void updateWorldTransform();

    std::string name_ ;
    KinematicJointPtr parent_joint_ ;
    std::vector<KinematicJointPtr> child_joints_ ;
    Eigen::Isometry3f world_ = Eigen::Isometry3f::Identity(); // world transform
};


class KinematicModel {
public :

    KinematicModel(const URDFRobot &robot) {
        create(robot) ;
    }

    // set world transform of root link
    void setWorldTransform(const Eigen::Isometry3f &tr);

    KinematicJointPtr getJoint(const std::string &name) const;
    KinematicLinkPtr getLink(const std::string &name) const;

    double getJointPosition(const std::string &name) const;
    void setJointPosition(const std::string &name, double val, KinematicJoint::LimitCheckType type = KinematicJoint::LIMIT_CHECK_CLAMP_VALUE);

    JointState getJointState() const;
    JointState getJointState(const std::vector<std::string> &joints) const;
    void setJointState(const JointState &state, KinematicJoint::LimitCheckType lt = KinematicJoint::LIMIT_CHECK_CLAMP_VALUE);

    Eigen::Isometry3f getLinkTransform(const std::string &link_name) const;
    std::map<std::string, Eigen::Isometry3f> getLinkTransforms() const;

    // get names of DOF joints
    std::vector<std::string> getJointNames() const ;

private:

    std::map<std::string, KinematicLinkPtr> links_ ;
    std::map<std::string, KinematicJointPtr> joints_ ;
    KinematicLinkPtr root_ ;

    void create(const URDFRobot &robot) ;
    void updateWorldTransforms() ;

};

} // namespace xsim


#endif
