#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>

#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xsim/robot_scene.hpp>

#include <iostream>
#include <thread>

#include <xsim/world.hpp>

#include "bullet_gui.hpp"

#include <xsim/multi_body.hpp>

#include <QApplication>
#include <QMainWindow>

using namespace xviz ;
using namespace xsim ;
using namespace std ;
using namespace Eigen ;

PhysicsWorld physics ;
ScenePtr scene(new Scene) ;

MultiBodyPtr robot_mb ;

class GUI: public SimulationGui {
public:
    GUI(PhysicsWorld &physics):
        SimulationGui(physics) {

        initCamera({0, 0, 0}, 1.0, SceneViewer::YAxis) ;
    }
};

void createWorld() {

    physics.createMultiBodyDynamicsWorld();
    physics.setGravity({0, -10, 0});

    string path = "/home/malasiot/local/bullet3/data/TwoJointRobot_wo_fixedJoints.urdf" ;


    robot_mb = physics.addMultiBody(MultiBodyBuilder(URDFRobot::load(path))
                                    .setFixedBase()
                                    .setLinearDamping(0.f)
                                    .setAngularDamping(0.f)) ;


   //body->addLink("world", 0.0, nullptr) ;
//    body->addJoint("world_to_base", PrismaticJoint, "world", "gripper_pole", global).setAxis(Vector3f::UnitX()) ;


    for( auto &j: robot_mb->jointNames() ) {
        Joint *joint = robot_mb->findJoint(j) ;
        if ( joint->hasMotor() ) {
            joint->setMotorControl(MotorControl(VELOCITY_CONTROL).setTargetVelocity(0).setMaxForce(0));
        }
    }

}

int main(int argc, char **argv)
{
    createWorld() ;

    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;

    QMainWindow window ;
    window.setCentralWidget(new GUI(physics)) ;
    window.resize(512, 512) ;
    window.show() ;

    return app.exec();
}
