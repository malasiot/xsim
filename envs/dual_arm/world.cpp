#include "world.hpp"
#include "robot.hpp"
#include "kuka_iiwa_ik_solver.hpp"
#include "planning_interface.hpp"
#include <xsim/kinematic.hpp>

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
        boxes_[i]->setLinearVelocity({0, 0, 0}) ;
        boxes_[i]->setAngularVelocity({0, 0, 0}) ;
        boxes_[i]->clearForces() ;
        boxes_[i]->disableDeactivation();
    }

  //  Joint *ctrl =pusher_->findJoint("pusher_slider_joint") ;
  //  ctrl->setMotorControl(MotorControl(VELOCITY_CONTROL).setTargetVelocity(0.0).setMaxForce(1000));
   // stepSimulation(0.05) ;

    updateCollisionEnv();

    resetSimulation();
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

//    Vector3f a = (orig * Vector4f{len, 0, 0, 1}).head<3>() ;

    Isometry3f ctr = Isometry3f::Identity() ;
    ctr.linear() = orig.linear() ;
    ctr.translation() = (p1 + p2)/2.0 ;
    CollisionShapePtr cshape(new BoxCollisionShape({len/2, 0.025, 0.025})) ;

    collisions_->disableCollision("motion", box);
    collisions_->addCollisionObject("motion", cshape, ctr);

    bool has_collision = collisions_->hasCollision() ;

    collisions_->removeCollisionObject("motion") ;
    collisions_->enableCollision("motion", box);

    return !has_collision ;
}

void World::execute(const Eigen::Isometry3f &orig, float t1, float t2, float speed) {
    /*
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
*/

}

void World::createScene() {
    CollisionShapePtr table_cs(new BoxCollisionShape(Vector3f{params_.table_width_/2.0, params_.table_height_/2.0, 0.001})) ;
    table_cs->setMargin(0) ;
    Isometry3f table_tr(Translation3f{params_.table_offset_x_, params_.table_offset_y_,  -0.001}) ;


    URDFRobot r1 = URDFRobot::load(params_.model_path_) ;

    r1.setJointPosition("joint_a2", -0.34);
    r1.setJointPosition("joint_a4", 0.7);

    r1_orig_.setIdentity() ;

    r1_ = addMultiBody(MultiBodyBuilder(r1)
                           .setName("r1")
                           .setFixedBase()
                           .setMargin(0.01)
                           .setLinearDamping(0.f)
                           .setAngularDamping(0.f)
                           .setWorldTransform(r1_orig_)
                       ) ;

    r1_->setJointPosition("joint_a2", -0.34);
    r1_->setJointPosition("joint_a4", 0.7);

    JointState js ;
    js["joint_a2"] = -0.34 ;
    js["joint_a4"] = 0.7 ;
    KukaIKSolver solver ;
    auto pose = solver.forward(js) ;

    Matrix4f tpose ;
    tpose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1 ;

    cout << pose.matrix() << endl ;
    std::vector<xsim::JointState> jss ;
    solver.solve(Isometry3f(pose), 0.5, jss) ;

    KinematicModel model(r1) ;

    auto p = model.getLinkTransforms();

    collisions_->addRobot(r1, 0.01) ;

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

    float gap = 0.001 ;

    float pallet_width = params_.grid_x_ * ( 2 * params_.box_sz_.x() )  + (params_.grid_x_ - 1 ) * gap ;
    float pallet_height = params_.grid_y_ * ( 2 * params_.box_sz_.y() )  + (params_.grid_y_ - 1 ) * gap ;

    Vector3f offset{params_.pallet_offset_x_ - pallet_width/2.0 + params_.box_sz_.x() , params_.pallet_offset_y_ - pallet_height/2.0 + params_.box_sz_.y(), 0} ;

    vector<string> box_names ;
    for( int i=0 ; i<params_.grid_y_ ; i++ ) {
        for( int j=0 ; j<params_.grid_x_ ; j++ ) {
            Vector3f c{j * ( params_.box_sz_.x() * 2 + gap ), i * (params_.box_sz_.y() * 2 + gap), params_.box_sz_.z()} ;
            Isometry3f box_tr(Translation3f{c + offset}) ;

          //  box_tr.linear() = AngleAxisf(M_PI*10/180.0, Vector3f::UnitZ()).matrix() ;

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

World::Parameters::Parameters(const cvx::Variant &config) {
    config.lookup("table.size.x", table_width_) ;
    config.lookup("table.size.y", table_height_) ;
    config.lookup("table.offset.x", table_offset_x_) ;
    config.lookup("table.offset.y", table_offset_y_) ;

    config.lookup("pallet.offset.x", pallet_offset_x_) ;
    config.lookup("pallet.offset.y", pallet_offset_y_) ;
    config.lookup("boxes.grid.rows", grid_y_ ) ;
    config.lookup("boxes.grid.cols", grid_x_ ) ;
    config.lookup("boxes.size.x", box_sz_.x() ) ;
    config.lookup("boxes.size.y", box_sz_.y() ) ;
    config.lookup("boxes.size.z", box_sz_.z() ) ;

    config.lookup("model", model_path_ ) ;
}
