#pragma once
#include <xsim/world.hpp>
#include <xsim/ompl_planner.hpp>
#include <xsim/collision_space.hpp>
#include <cvx/misc/variant.hpp>

#include "robot.hpp"

class Robot ;
class UR5Planning ;

class World: public xsim::PhysicsWorld {
public:
    struct Parameters {
        Parameters(const cvx::Variant &config) ;

        Eigen::Vector3f box_sz_ = { 0.3/2, 0.12/2, 0.36/2 } ;
        int grid_x_ = 1, grid_y_ = 1 ;
        std::string model_path_ ;

        float table_width_ = 3, table_height_ = 2.5 ;
        float table_offset_x_ = 0.0, table_offset_y_ = 0.65 ;
        float pallet_offset_x_ = 0.0, pallet_offset_y_ = 0.75 ;
    };

    enum RobotId { R1, R2 } ;

    World(const Parameters &params) ;

    std::map<std::string, Eigen::Isometry3f> getBoxTransforms() const ;
    std::vector<std::string> getBoxNames() const ;

    bool setRobot1Pose(const Eigen::Isometry3f &p) ;
    bool setRobot2Pose(const Eigen::Isometry3f &p) ;

    void plan1Cartesian(const Eigen::Isometry3f &v) ;

    void setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius) ;

    void reset() ;
    void updateCollisionEnv() ;

    bool plan(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const std::string &box, Eigen::Isometry3f &orig, float &t1, float &t2) ;
    void execute(const Eigen::Isometry3f &orig, float t1, float t2, float speed) ;

    xsim::CollisionSpace *collisions() { return collisions_.get() ; }
    const Parameters &params() const { return params_ ; }
/*
    void setJointState(Robot r, const xsim::JointState &state) {
        for( const auto &jn: KukaIKSolver::s_joint_names ) {
            std::string joint_name = ( r == R1 ) ? "r1_" : "r2_" ;
            joint_name += jn ;
            auto it = state.find(joint_name) ;
            if ( it != state.end() )
                controller(r)->setJointPosition(joint_name, it->second);
        }
    }

    xsim::JointState getJointState(Robot r) const {
        xsim::JointState state ;
        for( const auto &jn: KukaIKSolver::s_joint_names ) {
            std::string joint_name = ( r == R1 ) ? "r1_" : "r2_" ;
            joint_name += jn ;
            double v = controller(r)->getJointPosition(joint_name) ;
            state.emplace(joint_name, v) ;
        }
        return state ;
    }
*/
    xsim::MultiBodyPtr r1_, r2_ ;

    Robot &robot1() const { return *robot1_ ; }
    Robot &robot2() const { return *robot2_ ; }

    bool isStateValid(const xsim::JointState &js1, const xsim::JointState &js2) ;

    const Eigen::Isometry3f &r1Origin() const { return r1_orig_ ; }
    const Eigen::Isometry3f &r2Origin() const { return r2_orig_ ; }

private:
    xsim::RigidBodyPtr table_rb_ ;
    std::vector<xsim::RigidBodyPtr> boxes_ ;

    std::vector<Eigen::Isometry3f> orig_trs_ ;
    Eigen::Isometry3f r1_orig_, r2_orig_ ;

    std::shared_ptr<xsim::CollisionSpace> collisions_ ;
    std::unique_ptr<xsim::KinematicModel> kinematics_r1_, kinematics_r2_ ;
    Parameters params_ ;
    std::unique_ptr<Robot> robot1_, robot2_ ;

private:

    void createScene();
};
