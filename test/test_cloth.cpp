#include <xviz/scene/material.hpp>

#include <QApplication>
#include <QMainWindow>

#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>

#include <xviz/scene/node.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/mesh.hpp>

#include <iostream>

#include <xsim/world.hpp>
#include <xsim/soft_body.hpp>
#include <xsim/soft_body_shape.hpp>

#include "bullet_gui.hpp"

#include <xviz/scene/scene.hpp>
#include <xviz/scene/node_helpers.hpp>

using namespace Eigen ;

using namespace std ;
using namespace xviz ;
using namespace xsim ;

static PhysicsWorld physics ;

void createWorld() {
    // init physics

    physics.createSoftMultiBodyDynamicsWorld();

    // create ground plane object

    RigidBodyPtr ground = physics.addRigidBody(RigidBodyBuilder()
        .setName("ground")
        .setCollisionShape(CollisionShapePtr(new BoxCollisionShape({10., 0.05, 10.})))
        .setWorldTransform(Isometry3f(Translation3f{0, -0.05, 0}))
        .makeVisualShape({ 0.5, 0.5, 0.5, 1})) ;

    // create static pole
    Isometry3f poleTransform ;
    poleTransform.setIdentity() ;
    poleTransform.translate(Vector3f{.15, 2.5, 0.5}) ;
    poleTransform.rotate(AngleAxisf(0.5*M_PI, Vector3f::UnitZ()));
    poleTransform.translate(Vector3f{-0.15, -2.5, 0.5}) ;

    RigidBodyPtr pole = physics.addRigidBody( RigidBodyBuilder()
        .setName("pole")
        .setCollisionShape(CollisionShapePtr(new CylinderCollisionShape(0.25, 10)))
        .setWorldTransform(poleTransform)
        .makeVisualShape({ 0, 1, 0, 1})
    );


    const btScalar s = 4;  //size of cloth patch
    const int NUM_X = 31;  //vertices on X axis
    const int NUM_Z = 31;  //vertices on Z axis

    MaterialPtr material(new PhongMaterial(Vector3f{0, 1, 1}));
    material->setSide(Material::Side::Both) ;

    std::shared_ptr<SoftBodyShape> sbs(new SoftPatch2D({-2, 3, 0}, {2, 3, 0}, {-2, 3, 4}, NUM_X, NUM_Z, SoftPatch2D::TopEdge, false));
    SoftBodyPtr cloth = physics.addSoftBody(SoftBodyBuilder()
                        .setName("cloth")
                        .setShape(sbs)
                        .setMargin(0.05)
                        .setMass(1)
                        .makeVisualShape(material)
                    );
/*
    std::shared_ptr<SoftBodyShape> sbs1(new SoftPatch2D({-2, 3, 0.5}, {2, 3, 0}, {-2, 3, 4}, NUM_X, NUM_Z, 0, false));
    SoftBodyPtr cloth1 = physics.addSoftBody(SoftBodyBuilder()
                        .setName("cloth1")
                        .setShape(sbs1)
                        .setMargin(0.05)
                        .setMass(1)
                        .makeVisualShape(material)
                    );
                    */
}




int main(int argc, char **argv)
{

    createWorld() ;

    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setMajorVersion(3);
    format.setMinorVersion(3);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    format.setSwapInterval(1);

    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CoreProfile);

    QSurfaceFormat::setDefaultFormat(format);

    QMainWindow window ;
    window.setCentralWidget(new SimulationGui(physics, { 0, 0, 0}, 3.0)) ;
    window.resize(512, 512) ;
    window.show() ;

    return app.exec();
}
