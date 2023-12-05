#pragma once
#include <xsim/world.hpp>
#include <xsim/ompl_planner.hpp>
#include <xsim/collision_space.hpp>
#include <cvx/misc/variant.hpp>

class Robot ;
class UR5Planning ;

class World: public xsim::PhysicsWorld {
public:
    struct Parameters {
        Parameters(const cvx::Variant &config) ;

        Eigen::Vector3f box_sz_ = { 0.05, 0.05, 0.05 } ;
        int grid_x_ = 3, grid_y_ = 2 ;
        std::string model_path_ ;

        float table_width_ = 3, table_height_ = 2.5 ;
        float table_offset_x_ = 0.0, table_offset_y_ = 0.65 ;
        float pallet_offset_x_ = 0.0, pallet_offset_y_ = 0.55 ;
    };

    World(const Parameters &params) ;

    std::map<std::string, Eigen::Isometry3f> getBoxTransforms() const ;
    std::vector<std::string> getBoxNames() const ;

    void setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius) ;

    void reset() ;
    void updateCollisionEnv() ;

    bool plan(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const std::string &box, Eigen::Isometry3f &orig, float &t1, float &t2) ;
    void execute(const Eigen::Isometry3f &orig, float t1, float t2, float speed) ;

    xsim::CollisionSpace *collisions() { return collisions_.get() ; }
    const Parameters &params() const { return params_ ; }

    xsim::MultiBodyPtr r1_, r2_ ;

private:
    xsim::RigidBodyPtr table_rb_ ;
    std::vector<xsim::RigidBodyPtr> boxes_ ;

    std::vector<Eigen::Isometry3f> orig_trs_ ;
    Eigen::Isometry3f r1_orig_ ;

    std::shared_ptr<xsim::CollisionSpace> collisions_ ;
    Parameters params_ ;

private:

    void createScene();
};
