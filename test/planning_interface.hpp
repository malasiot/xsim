#pragma  once

#include <xsim/ompl_planner.hpp>
#include <xsim/collision_space.hpp>
#include <xsim/kinematic.hpp>

#include <xsim/collision_space.hpp>

#include <mutex>

class UR5Planning: public xsim::PlanningInterface {
public:
    UR5Planning(const xsim::URDFRobot &robot, xsim::CollisionSpace *collisions);

    bool isStateValid(const xsim::JointState &state) override;

    std::vector<std::string> getJointChain() const override;

    void getLimits(const std::string &name, double &lower, double &upper) const override;

    bool solveIK(const Eigen::Isometry3f &pose, std::vector<xsim::JointState> &solutions) const override;

    bool solveIK(const Eigen::Isometry3f &pose, const xsim::JointState &seed, xsim::JointState &solution) const override;

    Eigen::Isometry3f getToolPose(const xsim::JointState &state) override;

    xsim::KinematicModel &fk() { return robot_ ; }

private:
    xsim::KinematicModel robot_ ;
    xsim::CollisionSpace *collisions_ ;

    Eigen::Isometry3f tool_to_ee_ ;
    std::mutex mutex_ ;
};
