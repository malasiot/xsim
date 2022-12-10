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

MultiBodyPtr robot_mb ;

class GUI: public SimulationGui {
public:
    GUI(PhysicsWorld &physics,
        URDFRobot &rb, const string &ctrl_joint):
        SimulationGui(physics), robot_(rb), ctrl_joint_(ctrl_joint) {

        initCamera({0, -1, 0}, 0.5, SceneViewer::YAxis) ;
        vert_pos_ = 0.0, gripper_pos_ = 0.5 ;
        setDrawAxes(true);

    }

    void keyPressEvent(QKeyEvent *event) override {

        if ( event->key() == Qt::Key_Q ) {
            vert_pos_ -= 0.05 ;
            robot_mb->setJointPosition("world_to_base", vert_pos_) ;
        } else if ( event->key() == Qt::Key_W ) {
            vert_pos_ += 0.05 ;
            robot_mb->setJointPosition("world_to_base", vert_pos_) ;
        } else if ( event->key() == Qt::Key_A ) {
            gripper_pos_ -= 0.03 ;
            robot_mb->setJointPosition("left_gripper_joint", gripper_pos_) ;
            robot_mb->setJointPosition("right_gripper_joint", gripper_pos_) ;
        }  else if ( event->key() == Qt::Key_S ) {
            gripper_pos_ += 0.03 ;
            robot_mb->setJointPosition("left_gripper_joint", gripper_pos_) ;
            robot_mb->setJointPosition("right_gripper_joint", gripper_pos_) ;
        }

        update() ;

    }

private:
    URDFRobot &robot_ ;
    float vert_pos_ = 0.0, gripper_pos_ = 0.5 ;
    string ctrl_joint_ ;
};

URDFRobot robot ;

void createWorld() {

    physics.createMultiBodyDynamicsWorld();
    physics.setGravity({0, -10, 0});

    //ground
    physics.addRigidBody(RigidBodyBuilder()
                         .setCollisionShape(CollisionShapePtr(new BoxCollisionShape({1.5f, 0.05f, 1.5f})))
                         .setWorldTransform(Isometry3f(Translation3f{0, -1.5, 0}))
                         .makeVisualShape({0.5, 0.5, 0.5, 1}));


    // box
    physics.addRigidBody(RigidBodyBuilder()
                         .setMass(2.0)
                         .setCollisionShape(CollisionShapePtr(new BoxCollisionShape({0.03, 0.03f, 0.03f})))
                         .setWorldTransform(Isometry3f(Translation3f{0, -1.25, 0}))
                         .makeVisualShape({0.5, 0.5, 0, 1}));

    string path = "/home/malasiot/local/bullet3/examples/pybullet/gym/pybullet_data/pr2_gripper.urdf" ;
    robot = URDFRobot::load(path) ;

    Isometry3f global = Isometry3f::Identity() ;

    global.rotate(AngleAxisf(-M_PI/2, Vector3f::UnitZ())) ;
//    global.translate(Vector3f{0, 0, -0.2});

    MultiBodyBuilder mb(robot) ;

    mb.addLink("world", 0.0, nullptr) ;
    mb.addJoint("world_to_base", PrismaticJoint, "world", "gripper_pole", global).setAxis(Vector3f::UnitX()) ;

    robot_mb = physics.addMultiBody(mb) ;

}

int main(int argc, char **argv)
{
    createWorld() ;

    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;

    QMainWindow window ;
    window.setCentralWidget(new GUI(physics, robot, "left_gripper_joint")) ;
    window.resize(512, 512) ;
    window.show() ;

    return app.exec();
}
