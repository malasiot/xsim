#pragma once
#include <xsim/world.hpp>
#include <xsim/ompl_planner.hpp>

class Robot ;
class UR5Planning ;

class World: public xsim::PhysicsWorld {
public:
    struct Parameters {
        Eigen::Vector3f box_sz_ = { 0.05, 0.05, 0.075 } ;
        int grid_x_ = 3, grid_y_ = 2 ;
        std::string data_dir_ ;

        float table_width_ = 1.0, table_height_ = 1.5 ;
        float table_offset_x_ = 0.0, table_offset_y_ = 0.65 ;
        float pallet_offset_x_ = 0.0, pallet_offset_y_ = 0.55 ;
        float motion_start_offset_ = 0.05 ;
        float motion_push_offset_ = 0.05 ;
    };

    World(const Parameters &params) ;

    void enableCollisions() ;
    void disableCollisions() ;

    xsim::MultiBodyPtr robot_mb_ ;
    xsim::RigidBodyPtr table_rb_ ;
    std::vector<xsim::RigidBodyPtr> boxes_ ;


    std::shared_ptr<Robot> controller_ ;
    std::shared_ptr<UR5Planning> planner_ ;
    Parameters params_ ;

    void disableToolCollisions(const std::string &box);
    void enableToolCollisions(const std::string &box);
    void resetRobot();
private:
    void createScene(const xsim::URDFRobot &robot);
};
