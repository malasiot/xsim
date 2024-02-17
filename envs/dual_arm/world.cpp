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
/*
bool World::setRobot1Pose(const Eigen::Isometry3f &p) {
    KukaIKSolver solver ;

    KukaIKSolver::Problem ik(p * r1_orig_.inverse()) ;

    string prefix("r1_") ;

    std::vector<JointCoeffs> solutions ;
    if ( solver.solve(ik,  solutions) ) {
        for( const auto solution: solutions ) {
            JointState state ;
            for( uint j=0 ; j<7 ; j++ ) {
                state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
            }

            if ( isStateValid(state, getJointState(World::R2)) ) {
                setJointState(World::R1, state) ;
                stepSimulation(0.05);
                return true ;
            }
        }
    }

    return false ;
}


bool World::setRobot2Pose(const Eigen::Isometry3f &p) {
    KukaIKSolver solver ;

    KukaIKSolver::Problem ik(p * r2_orig_.inverse()) ;

    string prefix("r2_") ;

    std::vector<JointCoeffs> solutions ;
    if ( solver.solve(ik,  solutions) ) {
        for( const auto solution: solutions ) {
            JointState state ;
            for( uint j=0 ; j<7 ; j++ ) {
                state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
            }

            if ( isStateValid(getJointState(World::R1), state) ) {
                setJointState(World::R2, state) ;
                stepSimulation(0.05);
                return true ;
            }
        }
    }

    return false ;
}

void World::plan1Cartesian(const Eigen::Isometry3f &v2)
{
    JointState state1 = getJointState(World::R1) ;

    JointTrajectory traj ;
    traj.addPoint(0, getJointState(World::R1)) ;

    string prefix("r1_") ;

    KukaIKSolver solver ;

    KukaIKSolver::Problem ik(v2 * r1_orig_.inverse()) ;

    JointCoeffs c1 = {0} ;
    for( uint j=0 ; j<7 ; j++ ) {
        c1[j] = state1[prefix + KukaIKSolver::s_joint_names[j]] ;
    }

    ik.setSeedState(c1) ;

    std::vector<JointCoeffs> solutions ;
    if ( solver.solve(ik,  solutions) ) {
        for( const auto solution: solutions ) {
            JointState state ;
            for( uint j=0 ; j<7 ; j++ ) {
                state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
            }

            if ( isStateValid(state, getJointState(World::R2)) ) {
                traj.addPoint(1.0, state) ;
            }
        }
    }

}

*/


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

void World::setBoxMass()
{
    boxes_[0]->setGravity({0, 0, -10});

}

bool World::isStateValid(const xsim::JointState &state1, const JointState &state2)
{
    auto it = state1.find("r1_joint_a7") ;
    if ( it != state1.end() ) {
        if ( it->second < 0 ) return false ;
    }
    kinematics_r1_->setJointState(state1) ;

    map<string, Isometry3f> trs1 = kinematics_r1_->getLinkTransforms() ;
    collisions_->updateObjectTransforms(trs1) ;

    kinematics_r2_->setJointState(state2) ;

    map<string, Isometry3f> trs2 = kinematics_r2_->getLinkTransforms() ;
    collisions_->updateObjectTransforms(trs2) ;

    return !collisions_->hasCollision() ;
}
#define HORIZ
void World::createScene() {
    CollisionShapePtr table_cs(new BoxCollisionShape(Vector3f{params_.table_width_/2.0, params_.table_height_/2.0, 0.001})) ;
    table_cs->setMargin(0) ;
    Isometry3f table_tr(Translation3f{params_.table_offset_x_, params_.table_offset_y_,  -0.001}) ;

    CollisionShapePtr box0_cs(new BoxCollisionShape({0.1, 0.1, 0.1})) ;
    box0_cs->setMargin(0) ;
    Isometry3f box0_tr(Translation3f{0, 0.1, 0.5}) ;


     double l = 0.72 ;

#ifdef HORIZ
  r1_orig_.setIdentity() ;
    r1_orig_.translate(Vector3f{-l/2.0, 0, 0.75});
    r1_orig_.rotate(AngleAxisf(-M_PI/2, Vector3f::UnitX())) ;
//    r1_orig_.rotate(AngleAxisf(M_PI, Vector3f::UnitZ())) ;


    r2_orig_.setIdentity() ;
    r2_orig_.translate(Vector3f{l/2.0, 0, 0.75});
    r2_orig_.rotate(AngleAxisf(-M_PI/2, Vector3f::UnitX())) ;
#else

    r1_orig_.setIdentity() ;
    r1_orig_.translate(Vector3f{-l/2.0, 0, 0});


    r2_orig_.setIdentity() ;
    r2_orig_.translate(Vector3f{l/2.0, 0, 0});

#endif

    URDFRobot r1 = URDFRobot::loadURDF(params_.model_path_, "r1_") ;
    r1.setWorldTransform(r1_orig_);


    r1_ = addMultiBody(MultiBodyBuilder(r1)
                           .setName("r1")
                           .setFixedBase()
                           .setMargin(0.001)
                           .setLinearDamping(0.f)
                           .setAngularDamping(0.f)
                    //       .setWorldTransform(r1_orig_)
                       ) ;

    kinematics_r1_.reset(new KinematicModel(r1)) ;


    collisions_->disableCollision("r1_link_0", "table");

    collisions_->addRobot(r1, 0.01) ;

    URDFRobot r2 = URDFRobot::loadURDF(params_.model_path_, "r2_") ;
    r2.setWorldTransform(r2_orig_);


    r2_ = addMultiBody(MultiBodyBuilder(r2)
                           .setName("r2")
                           .setFixedBase()
                           .setMargin(0.001)
                           .setLinearDamping(0.f)
                           .setAngularDamping(0.f)
                    //       .setWorldTransform(r1_orig_)
                       ) ;

    kinematics_r2_.reset(new KinematicModel(r2)) ;

    collisions_->disableCollision("r2_link_0", "table");

    collisions_->addRobot(r2, 0.01) ;
    collisions_->addCollisionObject("table", table_cs, table_tr);
 //    collisions_->addCollisionObject("box", box0_cs, box0_tr);


    table_rb_ = addRigidBody(RigidBodyBuilder()
                             .setCollisionShape(table_cs)
                             .makeVisualShape({0.5, 0.5, 0.5, 1})
                             .setName("table")
                             .setFriction(3.0)
                             .setWorldTransform(table_tr)
                             ) ;

    auto box0_rb = addRigidBody(RigidBodyBuilder()
                                 .setCollisionShape(box0_cs)
                                 .makeVisualShape({0.1, 0.5, 0.5, 1})
                                 .setName("box")
                                 .setFriction(3.0)
                                 .setWorldTransform(box0_tr)
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
                                    .setMass(1)
                                    .setCollisionShape(box_cs)
                                    .makeVisualShape({1.0, 0.1, 0.9, 1})
                                    .setName(name)
                                    .setFriction(5.5)
                                    .setRestitution(0.01)
                                    .setSpinningFriction(0.005)
                                    .setWorldTransform(box_tr));


            box->disableDeactivation();

            box->setGravity({0, 0, 0}) ;

            boxes_.emplace_back(box) ;

            orig_trs_.emplace_back(box_tr) ;

            collisions_->disableCollision("table", name);
          //  collisions_->addCollisionObject(name, box_cs2, box_tr);

            box_names.emplace_back(name) ;
        }
    }

    for( const auto &ba: box_names ) {
        for( const auto &bb: box_names ) {
            collisions_->disableCollision(ba, bb);
        }
    }



    robot1_.reset(new Robot("r1_", r1_, r1_orig_)) ;
    robot2_.reset(new Robot("r2_", r2_, r2_orig_)) ;

    robot1_->setStateValidityChecker([this](const JointState &state){
        return isStateValid(state, robot2_->getJointState()) ;
    });

    robot2_->setStateValidityChecker([this](const JointState &state){
        return isStateValid(robot1_->getJointState(), state) ;
    });

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
