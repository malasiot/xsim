#pragma once
#include <xsim/world.hpp>
#include <xsim/ompl_planner.hpp>
#include <xsim/collision_space.hpp>

#include "planner.hpp"

class Robot ;
class UR5Planning ;

class World: public xsim::PhysicsWorld {
public:

    World(const std::string &data_root) ;

    std::map<std::string, Eigen::Isometry3f> getBoxTransforms() const ;
    std::vector<std::string> getBoxNames() const ;

    void disableToolCollisions(const std::string &box);
    void enableToolCollisions(const std::string &box);
    void reset() ;
    void resetRobot();
    void updateCollisionEnv() ;

    Robot *controller() { return controller_.get() ; }
    xsim::PlanningInterface *iplan() { return iplan_.get() ; }
    Planner *planner() { return planner_.get() ; }
    xsim::CollisionSpace *collisions() { return collisions_.get() ; }


    void coverage_analysis();
private:
    xsim::MultiBodyPtr robot_mb_ ;
    xsim::RigidBodyPtr table_rb_, box_rb_ ;

    std::shared_ptr<Robot> controller_ ;
    std::shared_ptr<xsim::PlanningInterface> iplan_ ;
    std::shared_ptr<xsim::CollisionSpace> collisions_ ;
    std::shared_ptr<Planner> planner_ ;

private:

    void createScene(const xsim::URDFRobot &robot);
};
