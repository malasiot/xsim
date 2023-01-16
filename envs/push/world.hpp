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

    void reset() ;
    void updateCollisionEnv() ;

    bool plan(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const std::string &box, Eigen::Isometry3f &orig, float &t1, float &t2) ;
    void execute(const Eigen::Isometry3f &orig, float t1, float t2, float speed) ;

    xsim::CollisionSpace *collisions() { return collisions_.get() ; }
    const Parameters &params() const { return params_ ; }

      xsim::MultiBodyPtr pusher_ ;
private:
    xsim::RigidBodyPtr table_rb_ ;
    std::vector<xsim::RigidBodyPtr> boxes_ ;

    std::vector<Eigen::Isometry3f> orig_trs_ ;
    Eigen::Isometry3f pusher_orig_ ;

    std::shared_ptr<xsim::CollisionSpace> collisions_ ;
    Parameters params_ ;

private:

    void createScene();
};
