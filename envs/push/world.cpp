#include "world.hpp"
#include "robot.hpp"
#include "ur5_ik_solver.hpp"
#include "planning_interface.hpp"

#include <xsim/collision_space.hpp>
#include <xsim/robot_scene.hpp>

#include <cvx/misc/format.hpp>
#include <thread>

using namespace std;
using namespace xsim ;
using namespace Eigen ;

World::World(const Parameters &params): params_(params) {

    createMultiBodyDynamicsWorld();
    setGravity({0, 0, -10});

    collisions_.reset(new CollisionSpace()) ;

    createScene() ;
}

std::map<string, Isometry3f> World::getBoxTransforms() const {
    std::map<string, Isometry3f> trs ;
    for( const auto &b: boxes_ ) {
        trs.emplace(b->getName(), b->getWorldTransform()) ;
    }
    return trs ;
}

std::vector<string> World::getBoxNames() const {
    std::vector<string> names(boxes_.size()) ;
    std::transform(boxes_.begin(), boxes_.end(), names.begin(), [](const RigidBodyPtr &b) { return b->getName() ;}) ;
    return names ;
}


void World::reset() {
    for( int i=0 ; i<boxes_.size() ; i++ ) {
        boxes_[i]->setWorldTransform(orig_trs_[i]) ;
    }

//    resetRobot() ;
}

void World::updateCollisionEnv() {
    auto trs = getBoxTransforms() ;
    collisions_->updateObjectTransforms(trs);
}

bool World::plan(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const std::string &box, Eigen::Isometry3f &orig, float &t1, float &t2) {
    Vector3f dir = (p2 - p1).normalized() ;
    float len = (p2 - p1).norm() ;
    float theta = atan2(dir.y(), dir.x()) ;
    orig.setIdentity() ;
    orig.linear() = AngleAxisf(theta, Vector3f::UnitZ()).matrix() ;
    orig.translation() = p1 ;
    t1 = len ;
    t2 = len - 0.01 ;

    Isometry3f ctr = Isometry3f::Identity() ;
    ctr.linear() = orig.linear() ;
    ctr.translation() = (p1 + p2)/2.0 ;
    CollisionShapePtr cshape(new BoxCollisionShape({len/2, 0.025, 0.025})) ;

    collisions_->disableCollision("motion", box);
    collisions_->addCollisionObject("motion", cshape, ctr);

    bool has_collision = collisions_->hasCollision() ;

    collisions_->removeCollisionObject("motion") ;
    collisions_->enableCollision("motion", box);

    if ( !has_collision ) {
        cout << "ok" << endl ;
    }
    return !has_collision ;
}

void World::execute(const Eigen::Isometry3f &orig, float t1, float t2, float speed) {
    pusher_->setBaseWorldTransform(orig) ;
    Joint *ctrl =pusher_->findJoint("pusher_slider_joint") ;

    ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(speed).setTargetPosition(t1));

    while ( fabs(ctrl->getPosition() - t1) > 0.001 ) {
        stepSimulation(0.05) ;
    }

    ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(0.5).setTargetPosition(t2));

    while ( fabs(ctrl->getPosition() - t2) > 0.001 ) {
        stepSimulation(0.05) ;
    }

    pusher_->setBaseWorldTransform(pusher_orig_) ;
    ctrl->setPosition(0) ;

    stepSimulation(0.05) ;


}

void World::createScene() {
    CollisionShapePtr table_cs(new BoxCollisionShape(Vector3f{params_.table_width_/2.0, params_.table_height_/2.0, 0.001})) ;
    table_cs->setMargin(0) ;
    Isometry3f table_tr(Translation3f{params_.table_offset_x_, params_.table_offset_y_,  -0.001}) ;

    pusher_orig_ = Isometry3f(Translation3f{0.0, -1.0, 0}) ;


     URDFRobot pusher = URDFRobot::load(params_.data_dir_ + "robots/pusher.urdf") ;

  /*   pusher_ = addMultiBody(MultiBodyBuilder(pusher)
                              .setName("pusher")
                              .setFixedBase()
                              .setMargin(0.01)
                              .setLinearDamping(0.f)
                              .setAngularDamping(0.f)
                            .setWorldTransform(pusher_orig_)
                              ) ;
*/
     MultiBodyBuilder mb ;

     mb.addLink("pusher_base", 0.0, nullptr, Isometry3f::Identity()) ;
     mb.addLink("pusher_tool", 3.0, CollisionShapePtr(new BoxCollisionShape({0.02, 0.02, 0.02})), Isometry3f::Identity()).makeVisualShape({0, 0, 1, 1}) ;

     Vector3f axis = {1, 0, 0} ;
     mb.addJoint("pusher_slider_joint", xsim::PrismaticJoint, "pusher_base", "pusher_tool", Isometry3f::Identity())
             .setAxis(axis).setLimits(0, 0.5)
             .setFriction(0).setDamping(0) ;

     pusher_ = addMultiBody(mb) ;

    collisions_->addCollisionObject("table", table_cs, table_tr);

    table_rb_ = addRigidBody(RigidBodyBuilder()
                             .setCollisionShape(table_cs)
                             .makeVisualShape({0.5, 0.5, 0.5, 1})
                             .setName("table")
                             .setFriction(3.0)
                             .setWorldTransform(table_tr)
                             ) ;

    CollisionShapePtr box_cs(new BoxCollisionShape(params_.box_sz_));
    box_cs->setMargin(0.0) ;

    CollisionShapePtr box_cs2(new BoxCollisionShape(params_.box_sz_));
    box_cs2->setMargin(0.02) ;

    float gap = 0.0001 ;

    float pallet_width = params_.grid_x_ * ( 2 * params_.box_sz_.x() )  + (params_.grid_x_ - 1 ) * gap ;
    float pallet_height = params_.grid_y_ * ( 2 * params_.box_sz_.y() )  + (params_.grid_y_ - 1 ) * gap ;

    Vector3f offset{params_.pallet_offset_x_ - pallet_width/2.0 + params_.box_sz_.x() , params_.pallet_offset_y_ - pallet_height/2.0 + params_.box_sz_.y(), 0} ;

    vector<string> box_names ;
    for( int i=0 ; i<params_.grid_y_ ; i++ ) {
        for( int j=0 ; j<params_.grid_x_ ; j++ ) {
            Vector3f c{j * ( params_.box_sz_.x() * 2 + gap ), i * (params_.box_sz_.y() * 2 + gap), params_.box_sz_.z()} ;
            Isometry3f box_tr(Translation3f{c + offset}) ;

            string name = cvx::format("box_{}_{}", i, j) ;

            auto box = addRigidBody(RigidBodyBuilder()
                                    .setMass(1.5)
                                    .setCollisionShape(box_cs)
                                    .makeVisualShape({1.0, 0.1, 0.9, 1})
                                    .setName(name)
                                    .setFriction(0.5)
                                    .setRestitution(0.01)
                                    .setSpinningFriction(0.005)
                                    .setWorldTransform(box_tr));

            box->disableDeactivation();

            boxes_.emplace_back(box) ;

            orig_trs_.emplace_back(box_tr) ;

            collisions_->disableCollision("table", name);
            collisions_->addCollisionObject(name, box_cs2, box_tr);

            box_names.emplace_back(name) ;
        }
    }

    for( const auto &ba: box_names ) {
        for( const auto &bb: box_names ) {
            collisions_->disableCollision(ba, bb);
        }
    }



}
