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
#include "world.hpp"
#include "robot.hpp"

using namespace xsim ;
using namespace xviz ;
using namespace std ;
using namespace Eigen ;
#if 0
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
                                   .setCollisionShape(make_shared<BoxCollisionShape>(Vector3f(0.08, 0.08, 0.08)))
                                   .makeVisualShape({1.0, 0.1, 0.6, 1})
                                   .setName("cube")
                                   .setWorldTransform(Isometry3f(Translation3f{0.35, 0.25, 0.7})));
}
#endif
int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;


    World *world = new World("/home/malasiot/source/xsim/data/") ;

    GUI *gui = new GUI(world) ;
    MainWindow window ;
    window.setGui(gui) ;
    window.resize(1024, 1024) ;


    QObject::connect(&window, &MainWindow::controlValueChanged, gui, &GUI::changeControlValue) ;


    for( const auto &jname: world->iplan()->getJointChain() ) {
        double l, u ;
        world->iplan()->getLimits(jname, l, u) ;
        window.addSlider(jname, l, u) ;
    }
    window.endSliders() ;

    window.show() ;
    return app.exec();
}
