#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>

#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>

#include <xviz/robot/robot_scene.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xsim/world.hpp>
#include <xsim/multi_body.hpp>

#include "bullet_gui.hpp"

#include <QApplication>
#include <QMainWindow>


#include <iostream>
#include <thread>

using namespace xviz ;
using namespace xsim ;

using namespace std ;
using namespace Eigen ;

PhysicsWorld physics ;
ScenePtr scene(new Scene) ;

MultiBodyPtr robot_mb ;
RigidBodyPtr cube_body, ground_body ;

class GUI: public SimulationGui, CollisionFeedback {
public:
    GUI(PhysicsWorld &physics):
    SimulationGui(physics) {
        physics_.setCollisionFeedback(this) ;
        collision_sensor_.reset(new GhostObject(CollisionShapePtr(new SphereCollisionShape(0.1)))) ;
        Isometry3f tr = Isometry3f::Identity() ;
        tr.translate(Vector3f(0, 0, -0.5)) ;
        collision_sensor_->setWorldTransform(tr) ;
        physics_.addGhost(collision_sensor_) ;
    }

    void onUpdate(float delta) override {
        SimulationGui::onUpdate(delta) ;

        vector<ContactResult> results ;
        physics.contactPairTest(cube_body, ground_body, 0.0, results) ;
        cout << results.size() << endl ;
    }

    void processContact(ContactResult &r) override {
        if ( r.a_ == nullptr || r.b_ == nullptr ) return ;
        if ( r.a_->getName() == "ground" || r.b_->getName() == "ground" ) return  ;
        cout << r.a_->getName() << ' ' << r.b_->getName() << endl ;
    }

    GhostObjectPtr  collision_sensor_ ;


};

NodePtr makeCube(const string &name, const Vector3f &hs, const Vector4f &clr) {
    NodePtr node = NodeHelpers::makeBox(hs, clr) ;
    node->setName(name) ;
    return node ;
}

void createWorld() {

    physics.createMultiBodyDynamicsWorld();

    Isometry3f tr(Translation3f{0, -1.8, 0}) ;

    Vector3f ground_hs{3.5f, 0.05f, 3.5f} ;

    RigidBodyBuilder ground_rb ;
    ground_rb.setCollisionShape(CollisionShapePtr(new BoxCollisionShape(ground_hs))) ;
    ground_rb.setWorldTransform(tr);
    ground_rb.setName("ground");
    ground_rb.makeVisualShape({0.5, 0.5, 0.5, 1});
    ground_body = physics.addRigidBody(ground_rb) ;

    float link_size = 0.4 ;
    Vector3f box_hs{0.05, link_size/2, 0.05} ;
    float box_mass = 0.1 ;

    CollisionShapePtr box_shape(new BoxCollisionShape(box_hs)) ;

    Isometry3f offset ;
    offset.setIdentity() ;
    offset.translate(Vector3f{0, -link_size/2, 0}) ;

    NodePtr base_node = makeCube("base", box_hs, {1, 0, 0, 1});
    NodePtr link1_node = makeCube("link1", box_hs, {1, 1, 0, 1}) ;
    NodePtr link2_node = makeCube("link2", box_hs, {1, 0, 1, 1}) ;
    NodePtr link3_node = makeCube("link3", box_hs, {0, 0, 1, 1}) ;

    MultiBodyBuilder mb ;

    mb.addLink("base", 0.0, box_shape, offset).setLocalInertialFrame(offset).setVisualShape(base_node) ;
    mb.addLink("link1", box_mass, box_shape, offset).setLocalInertialFrame(offset).setVisualShape(link1_node) ;
    mb.addLink("link2", box_mass, box_shape, offset).setLocalInertialFrame(offset).setVisualShape(link2_node) ;
    mb.addLink("link3", box_mass, box_shape, offset).setLocalInertialFrame(offset).setVisualShape(link3_node) ;

    Vector3f axis = {1, 0, 0} ;
    Isometry3f j2p ;
    j2p.setIdentity() ;
    j2p.translate(Vector3f{0, -link_size, 0}) ;
    mb.addJoint("j1", xsim::RevoluteJoint, "base", "link1", j2p).setAxis(axis).setFriction(0).setDamping(0) ;
    mb.addJoint("j2", xsim::RevoluteJoint, "link1", "link2", j2p).setAxis(axis).setFriction(0).setDamping(0) ;
    mb.addJoint("j3", xsim::RevoluteJoint, "link2", "link3", j2p).setAxis(axis).setFriction(0).setDamping(0) ;

    robot_mb = physics.addMultiBody(mb) ;

    robot_mb->findJoint("j1")->setMotorMaxImpulse(0) ;
    robot_mb->findJoint("j2")->setMotorMaxImpulse(0) ;
    robot_mb->findJoint("j3")->setMotorMaxImpulse(0) ;

    Vector3f col_hs{0.1f, 0.1f, 0.1f} ;
    Isometry3f col_tr = Isometry3f::Identity();
    col_tr.translate(Vector3f{0.0, -1.5+0.2, -0.25});

    NodePtr cube = makeCube("cube", col_hs, {1, 0 , 0, 1}) ;

    cube_body = physics.addRigidBody(RigidBodyBuilder()
                         .setMass(0.15)
                         .setCollisionShape(CollisionShapePtr(new BoxCollisionShape(col_hs)))
                         .setName("cube")
                         .setVisualShape(cube)
                         .setWorldTransform(col_tr)) ;


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
