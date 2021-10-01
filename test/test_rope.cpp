#include <xviz/scene/material.hpp>

#include <QApplication>
#include <QMainWindow>

#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/scene.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xsim/world.hpp>

#include <iostream>

#include "bullet_gui.hpp"

using namespace Eigen ;

using namespace std ;
using namespace xviz ;
using namespace xsim ;

class GUI: public SimulationGui {
public:
    GUI(PhysicsWorld &physics, vector<RigidBodyPtr> &objects):
    SimulationGui(physics), objects_(objects) {

    }

    void onUpdate(float delta) override {


        vector<ContactResult> contacts ;
        if ( physics_.contactTest(objects_[0], contacts ) ) {
            for( ContactResult &c: contacts ) {

                    cout << c.a_->getName() << ' ' << c.b_->getName() << endl ;
            }
        }


        SimulationGui::onUpdate(delta) ;

    }

private:
    vector<RigidBodyPtr> objects_ ;
};

static PhysicsWorld physics ;
static ScenePtr scene ;

static const float chain_radius = 0.1 ;
static const float chain_length = 0.3 ;
static const int TOTAL_BOXES = 6;


static Vector4f sColors[4] =  {
    Vector4f(60. / 256., 186. / 256., 84. / 256., 1),
    Vector4f(244. / 256., 194. / 256., 13. / 256., 1),
    Vector4f(219. / 256., 50. / 256., 54. / 256., 1),
    Vector4f(72. / 256., 133. / 256., 237. / 256., 1),

};

vector<RigidBodyPtr> boxes;

// create cylinder aligned with Y axis
NodePtr makeCylinder(float radius, float length, const Eigen::Affine3f &tr, const Eigen::Vector4f &clr) {

    // we need an extra node to perform rotation of cylinder so that it is aligned with Y axis instead of Z

    auto node = NodeHelpers::makeCylinder(radius, length, clr) ;

    node->transform().rotate(Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitX()));

    NodePtr externalNode(new Node) ;
    externalNode->setTransform(tr) ;
    externalNode->addChild(node) ;

    return externalNode ;
}

void createWorld() {
    // init physics

    physics.createDefaultDynamicsWorld();

   // create ground plane object

    RigidBodyPtr ground =physics.addRigidBody(RigidBodyBuilder()
                                              .setName("ground")
                                              .setCollisionShape(CollisionShapePtr(new BoxCollisionShape({10., 0.04, 10.})))
                                              .setWorldTransform(Isometry3f(Translation3f{0, -0.04, 0}))
                                              .makeVisualShape({0.5, 0.5, 0.5, 1}));

    // create static pole
 //   Affine3f poleTransform(Translation3f{0.5, 5, 0}) ;
 //   scene->addCylinder(0.25, 10, poleTransform.matrix(), {0, 1, 0, 1})->setName("pole") ;
 //   RigidBody pole(CylinderCollisionShape(0.25, 10), poleTransform);
 //   pole.setName("pole") ;
 //   physics.addBody(pole) ;

    // create static box from mesh
    Isometry3f meshTransform(Translation3f{2.5, 0.5, 1.5}) ;

    NodePtr meshNode(new Node) ;
    meshNode->load("/home/malasiot/Downloads/CoffeeTable.obj", 0) ;
    meshNode->transform().scale(3.5);
    NodePtr meshContainer(new Node) ;
    meshContainer->setName("mesh");

    meshContainer->addChild(meshNode) ;

    CollisionShapePtr meshColShape(new ConvexHullCollisionShape("/home/malasiot/Downloads/CoffeeTable.obj")) ;
    meshColShape->setLocalScale(3.5);

    RigidBodyPtr mesh = physics.addRigidBody(RigidBodyBuilder()
                                             .setName("mesh")
                                             .setMass(1.0)
                                             .setVisualShape(meshContainer)
                                             .setCollisionShape(meshColShape)
                                             .setWorldTransform(meshTransform));

    // create collision shape for chain element

    btScalar mass(1.0) ;

    CollisionShapePtr colShape(new CylinderCollisionShape(chain_radius, chain_length)) ;

    int lastBoxIndex = TOTAL_BOXES - 1;

    for (int i = 0; i < TOTAL_BOXES; i++) {
        float tx = 0, ty = 2.0f+ i*1.6*chain_length, tz = 0 ;

        Isometry3f box_tr(Translation3f{tx, ty, tz}) ;

        NodePtr chain_node = makeCylinder(chain_radius, chain_length, box_tr, sColors[i%4]) ;
        stringstream name ;
        name << "chain " << i ;
        chain_node->setName(name.str()) ;

        if ( i== lastBoxIndex ) {
            RigidBodyPtr box = physics.addRigidBody(RigidBodyBuilder()
                                                    .setMass(0)
                                                    .setCollisionShape(colShape)
                                                    .setWorldTransform(box_tr)
                                                    .setVisualShape(chain_node)
                                                    .setName(name.str())) ;
            boxes.push_back(box) ;
        }
        else {
            RigidBodyPtr box = physics.addRigidBody(RigidBodyBuilder()
                                                    .setCollisionShape(colShape)
                                                    .setMass(mass)
                                                    .setVisualShape(chain_node)
                                                    .setName(name.str())
                                                     .setWorldTransform(box_tr)) ;

            boxes.push_back(box) ;
        }

    }
    //add N-1 spring constraints

    for (int i = 0; i < TOTAL_BOXES - 1; ++i) {
        Point2PointConstraint c(boxes[i], boxes[i+1], {0.0, 1.5*chain_length/2.0, 0}, {0.0, -1.5*chain_length/2.0, 0}) ;
        physics.addConstraint(c);
    }

    boxes.push_back(ground) ;
    boxes.push_back(mesh) ;

}

int main(int argc, char **argv)
{
    createWorld() ;

    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext();

    QMainWindow window ;
    window.setCentralWidget(new GUI(physics, boxes)) ;
    window.resize(1024, 1024) ;
    window.show() ;

    return app.exec();
}
