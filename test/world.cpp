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

World::World(const std::string &data_dir) {

    createMultiBodyDynamicsWorld();
    setGravity({0, 0, -10});

    URDFRobot robot = URDFRobot::load(data_dir + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;


    for ( int i=0 ; i<6  ; i++) {
        robot.setJointPosition(UR5IKSolver::ur5_joint_names[i], 0);
    }
    robot.setJointPosition("shoulder_lift_joint", -1.2);
    robot.setJointPosition("elbow_joint", 0.7);
    //    robot.setWorldTransform(Isometry3f(Translation3f{-0.1, -0.2, 0.65}));

    collisions_.reset(new CollisionSpace()) ;
    collisions_->disableCollision("base_link", "table");
    collisions_->disableCollision("shoulder_link", "table");
    collisions_->addRobot(robot, 0.01) ;

    iplan_.reset(new UR5Planning(robot, collisions_.get())) ;
    planner_.reset(new Planner(iplan_.get())) ;

    createScene(robot) ;

    resetRobot() ;
}


void World::resetRobot() {
    for ( int i=0 ; i<6  ; i++) {
        robot_mb_->setJointPosition(UR5IKSolver::ur5_joint_names[i], 0);
    }

    robot_mb_->setJointPosition("shoulder_lift_joint", -1.2);
    robot_mb_->setJointPosition("elbow_joint", 0.7);

    JointState state ;
    for ( int i=0 ; i<6  ; i++) {
        double val = robot_mb_->getJointPosition(UR5IKSolver::ur5_joint_names[i]) ;
        state.emplace(UR5IKSolver::ur5_joint_names[i], val) ;
    }

    stepSimulation(0.05) ;

    map<string, Isometry3f> trs ;
    robot_mb_->getLinkTransforms(trs);
    iplan_->setStartState(state) ;
}


void World::updateCollisionEnv() {
//    auto trs = getBoxTransforms() ;
///    collisions_->updateObjectTransforms(trs);
}

void World::createScene(const URDFRobot &robot) {

    CollisionShapePtr table_cs(new BoxCollisionShape(Vector3f{0.5, 0.5, 0.001})) ;
    table_cs->setMargin(0) ;
    Isometry3f table_tr(Translation3f{0, 0.5,  -0.001}) ;


    collisions_->disableCollision("table", "base_link");
    collisions_->addCollisionObject("table", table_cs, table_tr);

    // ur5 + gripper
    robot_mb_ = addMultiBody(MultiBodyBuilder(robot)
                             .setName("robot")
                             .setFixedBase()
                             .setMargin(0.01)
                             .setLinearDamping(0.f)
                             .setAngularDamping(0.f)
                             ) ;

    resetRobot() ;

    table_rb_ = addRigidBody(RigidBodyBuilder()
                             .setCollisionShape(table_cs)
                             .makeVisualShape({0.5, 0.5, 0.5, 1})
                             .setName("table")
                             .setFriction(3.0)
                             .setWorldTransform(table_tr)
                             ) ;

    controller_.reset(new Robot(this, robot_mb_)) ;

    Vector3f box_sz{0.1, 0.05, 0.1};
    CollisionShapePtr box_cs(new BoxCollisionShape(box_sz));
    box_cs->setMargin(0.02) ;

     Isometry3f box_tr(Translation3f{0, 0.5, box_sz.z()}) ;


     auto box = addRigidBody(RigidBodyBuilder()
                             .setMass(1.5)
                             .setCollisionShape(box_cs)
                             .makeVisualShape({1.0, 0.1, 0.9, 1})
                             .setName("box")
                             .setFriction(0.5)
                             .setRestitution(0.01)
                             .setSpinningFriction(0.005)
                             .setWorldTransform(box_tr));

     box->disableDeactivation();

     collisions_->disableCollision("box", "table");
     collisions_->addCollisionObject("box", box_cs, box_tr);


}

void World::coverage_analysis() {

    ofstream strm("/tmp/coverage.ply") ;

    vector<Vector3f> reachable, collision_free ;

    for( float x = -0.5 ; x<= 0.5 ; x+=0.02 ) {
        for( float y = 0 ; y<= 1; y+=0.02 ) {
            for( float z = 0.02 ; z < 0.5 ; z += 0.01 ) {
                JointState seed, solution ;
                Matrix3f m(AngleAxisf(M_PI, Vector3f::UnitY())) ;
                Isometry3f p = Isometry3f::Identity() ;
                p.linear() = m ;
                p.translation() = Vector3f{x, y, z} ;

                vector<JointState> solutions ;
                if ( iplan()->solveIK(p, solutions)  ) {
                    bool found = false ;
                    for( int i=0 ; i<solutions.size() ; i++ ) {
                        if ( iplan()->isStateValid(solutions[i]) ) {
                            found = true ;

                            break ;
                        }
                    }
                    if ( !found ) collision_free.emplace_back(x, y, z) ;
                }
            }
        }
    }

    strm << "ply\nformat ascii 1.0\n" ;
    strm << "element vertex " << collision_free.size() + reachable.size() << endl ;
    strm << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n";
    strm << "end_header\n";

    for( const auto &v: reachable ) {
        strm << v.adjoint() << ' ' << "0 255 0\n" ;
    }

    for( const auto &v: collision_free ) {
        strm << v.adjoint() << ' ' << "255 0 0\n" ;
    }


}
