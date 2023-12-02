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

    void disableToolCollisions();
    void enableToolCollisions();
    void reset() ;
    void resetRobot();
    void updateCollisionEnv() ;

    Robot *controller() { return controller_.get() ; }
    xsim::PlanningInterface *iplan() { return iplan_.get() ; }
    Planner *planner() { return planner_.get() ; }
    xsim::CollisionSpace *collisions() { return collisions_.get() ; }

    xsim::RGBDCameraSensor *camera() const { return camera_.get() ; }

    void coverage_analysis();
private:
    xsim::MultiBodyPtr robot_mb_ ;
    xsim::RigidBodyPtr table_rb_;
    std::vector<xsim::RigidBodyPtr> boxes_ ;
    std::vector<Eigen::Isometry3f> orig_trs_ ;
    std::unique_ptr<xsim::RGBDCameraSensor> camera_ ;

    std::shared_ptr<Robot> controller_ ;
    std::shared_ptr<xsim::PlanningInterface> iplan_ ;
    std::shared_ptr<xsim::CollisionSpace> collisions_ ;
    std::shared_ptr<Planner> planner_ ;

    Eigen::Vector3f box_sz_ = { 0.05, 0.05, 0.05 } ;
    int grid_x_ = 3, grid_y_ = 2 ;

    float table_width_ = 0.75, table_height_ = 1.5 ;
    float table_offset_x_ = 0.0, table_offset_y_ = 0.65 ;
    float pallet_offset_x_ = 0.0, pallet_offset_y_ = 0.55 ;

private:

    void createScene(const xsim::URDFRobot &robot);
};
