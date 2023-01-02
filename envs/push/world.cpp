#include "world.hpp"
#include "robot.hpp"

#include <xsim/joint_state_planner.hpp>
#include <xsim/collision_space.hpp>
#include <xsim/kinematic.hpp>

#include <xsim/collision_space.hpp>

#include <cvx/misc/format.hpp>
#include <thread>

using namespace std;
using namespace xsim ;
using namespace Eigen ;


class UR5Planning: public PlanningInterface {
public:
    UR5Planning(const URDFRobot &robot, std::vector<RigidBodyPtr> &boxes): robot_(robot), boxes_(boxes) {

        collisions_.disableCollision("base_link", "table");
        collisions_.addRobot(robot) ;

        for ( int i=0 ; i<6  ; i++) {
            double val = robot_.getJointPosition(UR5IKSolver::ur5_joint_names[i]) ;
            start_state_.emplace(UR5IKSolver::ur5_joint_names[i], val) ;
        }
    }


    bool isStateValid(const JointState &state) override  {
        std::lock_guard<std::mutex> lock(mutex_) ;

        robot_.setJointState(state) ;

        map<string, Isometry3f> trs = robot_.getLinkTransforms() ;
        collisions_.updateObjectTransforms(trs) ;
        for( const auto &box: boxes_ ) {
          collisions_.updateObjectTransform(box->getName(), box->getWorldTransform());
        //    cout << box->getName() << ' ' << box->getWorldTransform().matrix() << endl ;
        }

        return !collisions_.hasCollision() ;
    }
    std::vector<std::string> getJointChain() const override {
        vector<string> joints ;
        std::copy(UR5IKSolver::ur5_joint_names, UR5IKSolver::ur5_joint_names + 6, std::back_inserter(joints)) ;
        return joints ;
    }
    void getLimits(const std::string &name, double &lower, double &upper) const override {
        KinematicJointPtr joint = robot_.getJoint(name) ;
        assert(joint) ;
        joint->getLimits(lower, upper) ;
    }

    bool solveIK(const Eigen::Isometry3f &pose, std::vector<JointState> &solutions) const override {
        UR5IKSolver solver ;
        return solver.solve(pose, solutions) ;
    }

    bool solveIK(const Eigen::Isometry3f &pose, const JointState &seed, JointState &solution) const override {
        UR5IKSolver solver ;
        return solver.solve(pose, seed, solution) ;
    }

    Isometry3f getToolPose() override {
        robot_.setJointState(start_state_) ;
        auto pose = robot_.getLinkTransform("ee_link") ;
        //    pose.translation() -= Vector3f{-0.1, -0.2, 0.65}; // transform to robot base coordinates
        return pose ;
    }

    KinematicModel robot_ ;
    CollisionSpace collisions_ ;
    std::vector<xsim::RigidBodyPtr> &boxes_ ;
    std::mutex mutex_ ;
};



World::World(const Parameters &params): params_(params) {

    createMultiBodyDynamicsWorld();
    setGravity({0, 0, -10});

    URDFRobot robot = URDFRobot::load(params.data_dir_ + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;

    robot.setJointPosition("shoulder_lift_joint", -1.2);
    robot.setJointPosition("elbow_joint", 0.7);
    //    robot.setWorldTransform(Isometry3f(Translation3f{-0.1, -0.2, 0.65}));


    planner_.reset(new UR5Planning(robot, boxes_)) ;

    createScene(robot) ;

}

void World::disableToolCollisions(const std::string &box) {
    static_pointer_cast<UR5Planning>(planner_)->collisions_.disableCollision(box, "tool") ;
}

void World::enableToolCollisions(const std::string &box) {
    static_pointer_cast<UR5Planning>(planner_)->collisions_.enableCollision(box, "tool") ;
}

class WorldCollisionFilter: public xsim::CollisionFilter {
public:
    WorldCollisionFilter() = default ;

    bool collide(CollisionObject *obj1, CollisionObject *obj2) {

        if ( obj1->getName() == "base_link" && obj2->getName() == "table" ) return false ;
        if ( obj1->getName() == "table" && obj2->getName() == "base_link" ) return false ;
        cout << obj1->getName() << ' ' << obj2->getName() << endl ;
        return true ;
    }
};

void World::resetRobot() {
    robot_mb_->setJointPosition("shoulder_lift_joint", -1.2);
    robot_mb_->setJointPosition("elbow_joint", 0.7);


    JointState state ;
    for ( int i=0 ; i<6  ; i++) {
        double val = robot_mb_->getJointPosition(UR5IKSolver::ur5_joint_names[i]) ;
        state.emplace(UR5IKSolver::ur5_joint_names[i], val) ;
    }
    planner_->setStartState(state) ;
}

void World::createScene(const URDFRobot &robot) {
    CollisionShapePtr table_cs(new BoxCollisionShape(Vector3f{params_.table_width_/2.0, params_.table_height_/2.0, 0.001})) ;
    table_cs->setMargin(0) ;
    Isometry3f table_tr(Translation3f{params_.table_offset_x_, params_.table_offset_y_,  -0.001}) ;

    planner_->collisions_.disableCollision("table", "base_link");
    planner_->collisions_.addCollisionObject("table", table_cs, table_tr);


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
                             .setFriction(1.0)
                             .setWorldTransform(table_tr)
                             ) ;

    controller_.reset(new Robot(robot_mb_, planner_.get())) ;

    // setCollisionFilter(new WorldCollisionFilter());



    CollisionShapePtr box_cs(new BoxCollisionShape(params_.box_sz_));
    box_cs->setMargin(0.02) ;


    float gap = 0.001 ;

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
                                    .setMass(0.5)
                                    .setCollisionShape(box_cs)
                                    .makeVisualShape({1.0, 0.1, 0.9, 1})
                                    .setName(name)
                                    .setFriction(0.5)
                                    .setRestitution(0.01)
                                    .setSpinningFriction(0.005)
                                    .setWorldTransform(box_tr));

            box->disableDeactivation();

            boxes_.emplace_back(box) ;

            planner_->collisions_.disableCollision("table", name);
            planner_->collisions_.addCollisionObject(name, box_cs, box_tr);


            box_names.emplace_back(name) ;
        }

    }

    for( const auto &ba: box_names ) {
        for( const auto &bb: box_names ) {
            planner_->collisions_.disableCollision(ba, bb);
        }
    }


}
