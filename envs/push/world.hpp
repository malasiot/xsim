#pragma once
#include <xsim/world.hpp>
#include <xsim/ompl_planner.hpp>
#include <xsim/collision_space.hpp>

#include "planner.hpp"

class Robot ;
class UR5Planning ;

class World: public xsim::PhysicsWorld {
public:
    struct Parameters {
        Eigen::Vector3f box_sz_ = { 0.05, 0.05, 0.05 } ;
        int grid_x_ = 3, grid_y_ = 2 ;
        std::string data_dir_ ;

        float table_width_ = 0.75, table_height_ = 1.5 ;
        float table_offset_x_ = 0.0, table_offset_y_ = 0.65 ;
        float pallet_offset_x_ = 0.0, pallet_offset_y_ = 0.55 ;
        float motion_start_offset_ = 0.05 ;
        float motion_push_offset_ = 0.025 ;
    };

    World(const Parameters &params) ;

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
    const Parameters &params() const { return params_ ; }

    xviz::NodePtr collisionScene() const { return vcol_ ; }

    void coverage_analysis();
private:
    xsim::MultiBodyPtr robot_mb_ ;
    xsim::RigidBodyPtr table_rb_ ;
    std::vector<xsim::RigidBodyPtr> boxes_ ;
    xsim::MultiBodyPtr pusher_ ;
    std::vector<Eigen::Isometry3f> orig_trs_ ;

    std::shared_ptr<Robot> controller_ ;
    std::shared_ptr<xsim::PlanningInterface> iplan_ ;
    std::shared_ptr<xsim::CollisionSpace> collisions_ ;
    std::shared_ptr<Planner> planner_ ;
    Parameters params_ ;
    xviz::NodePtr vcol_ ;

private:

    void createScene(const xsim::URDFRobot &robot);
};
