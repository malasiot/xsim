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
MultiBodyPtr car ;

class GUI: public SimulationGui {
public:
    GUI(xsim::PhysicsWorld &physics):
        SimulationGui(physics) {

        initCamera({0, 0, 0}, 0.4, SceneViewer::ZAxis) ;
    }

    void keyPressEvent(QKeyEvent *event) override {
         int key = event->key() ;

         if ( key == Qt::Key_Left ) {
             vehicle_steering -= steering_increment;
             vehicle_steering = std::max(vehicle_steering, -steering_clamp) ;

         } else if ( key == Qt::Key_Right ) {
             vehicle_steering += steering_increment;
             vehicle_steering = std::max(vehicle_steering, steering_clamp) ;

         } else if ( key == Qt::Key_Up ) {
            velocity += velocity_increment ;
            velocity = std::min(velocity, velocity_clamp) ;

         } else if ( key == Qt::Key_Down ) {
            velocity -= velocity_increment ;
            velocity = std::max(velocity, -velocity_clamp) ;

         } else
             SimulationGui::keyPressEvent(event) ;

    }

private:

    float vehicle_steering = 0.f ;
    float engine_force = 0.f ;
    float velocity = 0.f ;
    const float steering_increment = 0.1f ;
    const float steering_clamp = 0.3f ;
    const float velocity_increment = 0.1f ;
    const float velocity_clamp = 2.0f ;
    const float max_engine_force = 1000.0f ;
    const float breaking_force = 100.f ;
};

void createScene() {

    physics.createMultiBodyDynamicsWorld();
    physics.setGravity({0, 0, -10});

    // ground
    Isometry3f tr(Translation3f{0, 0, -0.03}) ;

    Vector3f ground_hs{50.f, 50.f, 0.03} ;
    auto groundNode = NodeHelpers::makeBox(ground_hs, {0.5, 0.5, 0.5, 1}) ;
    groundNode->setTransform(tr) ;
    scene->addChild(groundNode) ;

    CollisionShapePtr ground_shape(new BoxCollisionShape(ground_hs));

    RigidBodyBuilder ground_rb ;
    ground_rb.setCollisionShape(ground_shape) ;
    ground_rb.setWorldTransform(tr);
    ground_rb.setName("ground");
    physics.addRigidBody(RigidBodyBuilder()
        .setCollisionShape(CollisionShapePtr(new BoxCollisionShape(BoxCollisionShape({50.f, 50.f, 0.03}))))
        .makeVisualShape({0.5, 0.5, 0.5, 1})
        .setWorldTransform(Isometry3f(Translation3f{0, 0, -0.03}))
        .setName("ground")) ;


   // racecar

    string path = "/home/malasiot/local/bullet3/examples/pybullet/gym/pybullet_data/racecar/racecar_differential.urdf" ;

    //chassis

    car = physics.addMultiBody(MultiBodyBuilder()
                               .loadURDF(URDFRobot::load(path))
                               .setWorldTransform(Isometry3f(Translation3f{0, 0, 0.2})));

    for ( const auto &j: car->jointNames() ) {

        auto joint = car->findJoint(j) ;
        cout << j << endl ;
        if ( joint && joint->hasMotor() ) {
            joint->setMotorControl(MotorControl(VELOCITY_CONTROL).setMaxForce(100).setTargetVelocity(0));
        }
    }
}



int main(int argc, char **argv)
{
    createScene() ;

    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;
  //  ResourceLoader::instance().setLocalPath("/home/malasiot/local/bullet3/examples/pybullet/gym/pybullet_data/racecar/meshes/");

    QMainWindow window ;
    window.setCentralWidget(new GUI(physics)) ;
    window.resize(512, 512) ;
    window.show() ;

    return app.exec();
}
