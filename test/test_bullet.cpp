#include <xviz/scene/renderer.hpp>
#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>

#include <iostream>
#include <thread>

#include <xsim/world.hpp>
#include <xviz/scene/scene.hpp>
#include <xviz/scene/node_helpers.hpp>

#include "bullet_gui.hpp"

#include <QApplication>
#include <QMainWindow>

#include <random>

using namespace xviz ;
using namespace xsim ;

using namespace std ;
using namespace Eigen ;

#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

PhysicsWorld physics ;

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

void createWorld() {

    // init physics

    physics.createDefaultDynamicsWorld();

    Isometry3f tr(Translation3f{0, -10, 0}) ;

    Vector3f ground_hs{5.0f, 0.05f, 5.0f} ;
    physics.addRigidBody(RigidBodyBuilder()
                         .setCollisionShape(CollisionShapePtr(new BoxCollisionShape(ground_hs)))
                         .setWorldTransform(tr)
                         .makeVisualShape(Vector4f{0.5, 0.5, 0.5, 1}));

    //create a few dynamic rigidbodies
    // Re-using the same collision is better for memory usage and performance

    Vector3f cube_hs{.1f, .1f, .1f} ;

    GeometryPtr geom(new BoxGeometry(cube_hs)) ;
    CollisionShapePtr colShape(new BoxCollisionShape(cube_hs));
    btScalar mass(1.f);

    /// Create Dynamic Objects
    ///
    ///
    std::uniform_real_distribution<float> dis(0.0, 1.0);
    for (int k = 0; k < ARRAY_SIZE_Y; k++)  {
        for (int i = 0; i < ARRAY_SIZE_X; i++)  {
            for (int j = 0; j < ARRAY_SIZE_Z; j++) {
                float tx = 0.2 * i ;
                float ty = 2 + 0.2 * k ;
                float tz = 0.2 * j ;

                Isometry3f tr(Translation3f{tx, ty, tz}) ;

                Vector4f clr{dis(gen), dis(gen), dis(gen), 1.0f} ;

                stringstream name ;
                name << i << ' ' << j << ' ' << endl ;

                NodePtr node(new Node) ; // visual node
                MaterialPtr mat(new PhongMaterial(clr));
                node->addDrawable(geom, mat);

                physics.addRigidBody(RigidBodyBuilder()
                                     .setName(name.str())
                                     .setMass(mass)
                                     .setCollisionShape(colShape)
                                     .setWorldTransform(tr)
                                     .setVisualShape(node)) ;
            }
        }
    }
}

int main(int argc, char **argv)
{
    createWorld() ;

    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;

    QMainWindow window ;
    window.setCentralWidget(new SimulationGui(physics)) ;
    window.resize(512, 512) ;
    window.show() ;

    return app.exec();
}
