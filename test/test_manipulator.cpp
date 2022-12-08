#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>

#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xviz/robot/robot_scene.hpp>
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

#include "gui.hpp"

using namespace xviz ;
using namespace xsim ;

using namespace std ;
using namespace Eigen ;

PhysicsWorld physics ;

MultiBodyPtr robot_mb, table_mb ;
RigidBodyPtr cube_rb ;


void createScene(const std::string &path, const URDFRobot &robot) {

    physics.createMultiBodyDynamicsWorld();
    physics.setGravity({0, 0, -10});

    URDFRobot table = URDFRobot::load(path + "models/table.urdf");

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
    robot.setWorldTransform(Isometry3f(Translation3f{-0.1, -0.2, 0.65}));

    createScene(path, robot) ;

    Robot rb(robot_mb) ;

    GUI *gui = new GUI(physics, rb) ;
    MainWindow window ;
    window.setGui(gui) ;
    window.resize(1024, 1024) ;


    QObject::connect(&window, &MainWindow::controlValueChanged, gui, &GUI::changeControlValue) ;
    QObject::connect(gui, &GUI::robotStateChanged, &window, &MainWindow::updateControls);

    for( const auto &jname: rb.armJointNames() ) {
        URDFJoint *j = robot.findJoint(jname) ;
        window.addSlider(jname, j->lower_, j->upper_) ;
    }
    window.endSliders() ;

    window.show() ;
    return app.exec();
}
