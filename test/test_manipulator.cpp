#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>

#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xsim/robot_scene.hpp>
#include <xviz/gui/manipulator.hpp>
#include <xviz/gui/viewer.hpp>

#include <iostream>
#include <thread>

#include <xsim/world.hpp>

#include <xsim/multi_body.hpp>
#include <xsim/soft_body.hpp>
#include <xsim/soft_body_shape.hpp>

#include <QApplication>
#include <QMainWindow>

#include "mainwindow.hpp"
#include <xsim/joint_state_planner.hpp>
#include <xsim/collision_space.hpp>
#include <xsim/kinematic.hpp>

#include "gui.hpp"

using namespace xsim ;
using namespace xviz ;
using namespace std ;
using namespace Eigen ;

PhysicsWorld physics ;

MultiBodyPtr robot_mb, table_mb ;
RigidBodyPtr cube_rb ;

class UR5Planning: public PlanningInterface {
public:
    UR5Planning(const URDFRobot &robot, const URDFRobot &table): robot_(robot) {

        collisions_.addRobot(robot) ;
        collisions_.addRobot(table) ;


        collisions_.hasCollision() ;

        //       CollisionShapePtr box(new BoxCollisionShape{{0.05, 0.05, 0.05}}) ;
        //       collisions_.addCollisionObject("box1", box, Isometry3f(Translation3f{{0.25, 0.15, 0.38}})) ;
        //       collisions_.addCollisionObject("box2", box, Isometry3f(Translation3f{{0, 0, 0.9}})) ;

        for ( int i=0 ; i<6  ; i++) {
            double val = robot_.getJointPosition(UR5IKSolver::ur5_joint_names[i]) ;
            start_state_.emplace(UR5IKSolver::ur5_joint_names[i], val) ;
        }
    }


    bool isStateValid(const JointState &state) override  {
        robot_.setJointState(state) ;

        map<string, Isometry3f> trs = robot_.getLinkTransforms() ;
        collisions_.updateObjectTransforms(trs) ;
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
        return {};
    }

    KinematicModel robot_ ;
    CollisionSpace collisions_ ;
};

void createScene(const std::string &path, const URDFRobot &robot, const URDFRobot &table) {

    physics.createMultiBodyDynamicsWorld();
    physics.setGravity({0, 0, -10});



    // ur5 + gripper
    robot_mb = physics.addMultiBody(MultiBodyBuilder(robot)
                                    .setName("robot")
                                    .setFixedBase()
                                    .setLinearDamping(0.f)
                                    .setAngularDamping(0.f)
                                    ) ;

    robot_mb->setJointPosition("shoulder_lift_joint", -0.6);
    robot_mb->setJointPosition("elbow_joint", 0.2);

    // table
    table_mb = physics.addMultiBody(MultiBodyBuilder(table).setName("table"));

    // a cube to grasp

    cube_rb = physics.addRigidBody(RigidBodyBuilder()
                                   .setMass(0.1)
                                   .setCollisionShape(make_shared<BoxCollisionShape>(Vector3f(0.08, 0.08, 0.08)))
                                   .makeVisualShape({1.0, 0.1, 0.6, 1})
                                   .setName("cube")
                                   .setWorldTransform(Isometry3f(Translation3f{0.5, 0.25, 0.7})));
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;
    // ResourceLoader::instance().setLocalPath("/home/malasiot/source/xviz/data/physics/models/");

    // load URDFs
    string path = "/home/malasiot/source/xsim/data/" ;
    URDFRobot robot = URDFRobot::load(path + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;
    robot.setJointPosition("shoulder_lift_joint", -0.6);
    robot.setJointPosition("elbow_joint", 0.2);
    robot.setWorldTransform(Isometry3f(Translation3f{-0.1, -0.2, 0.65}));
    URDFRobot table = URDFRobot::load(path + "models/table.urdf");

    createScene(path, robot, table) ;

    Robot rb(robot_mb, new UR5Planning(robot, table)) ;

    GUI *gui = new GUI(physics, rb) ;
    MainWindow window ;
    window.setGui(gui) ;
    window.resize(1024, 1024) ;


    QObject::connect(&window, &MainWindow::controlValueChanged, gui, &GUI::changeControlValue) ;
    QObject::connect(gui, &GUI::robotStateChanged, &window, &MainWindow::updateControls);

    for( const auto &jname: rb.armJointNames() ) {
        const URDFJoint *j = robot.findJoint(jname) ;
        window.addSlider(jname, j->lower_, j->upper_) ;
    }
    window.endSliders() ;

    window.show() ;
    return app.exec();
}
