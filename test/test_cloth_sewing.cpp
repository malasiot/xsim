#include <xviz/scene/material.hpp>

#include <QApplication>
#include <QMainWindow>

#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>

#include <xviz/scene/node.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/mesh.hpp>

#include <iostream>

#include <xviz/physics/world.hpp>
#include <xviz/physics/soft_body.hpp>
#include <xviz/physics/soft_body_shape.hpp>

#include "bullet_gui.hpp"

#include <xviz/scene/scene.hpp>
#include <xviz/scene/node_helpers.hpp>

using namespace Eigen ;

using namespace std ;
using namespace xviz ;

static PhysicsWorld physics ;

class TwoPatchesTest: public SoftBodyShape {
public:

    TwoPatchesTest() ;

protected:

    void makePatch(const Eigen::Vector3f &c00, const Eigen::Vector3f &c10, const Eigen::Vector3f &c01,
                   uint nvX, uint nvY, uint offset) ;

};
#define IDX(_x_, _y_) ((_y_)*nvX + (_x_))

TwoPatchesTest::TwoPatchesTest()
{
    const uint nvX = 11, nvY = 11 ;

    assert( nvX >=2 && nvY >=2) ;
    uint numNodes = nvX * nvY * 2;

    nodes_.resize(numNodes) ;
    node_mass_.resize(numNodes) ;


    makePatch({-2, 3, 0}, {2, 3, 0}, {-2, 3, 4}, nvX, nvY, 0) ;
    makePatch({-2, 3, 4.5}, {2, 3, 4.5}, {-2, 3, 7}, nvX, nvY, nvX*nvY) ;

    node_mass_[0] = 0 ;
    node_mass_[nvX-1] = 0;

    for( uint x=0 ; x<nvX ; x++ )
        links_.emplace_back(IDX(x, nvY-1), IDX(x, 0));
}

void TwoPatchesTest::makePatch(const Vector3f &c00, const Vector3f &c10, const Vector3f &c01, uint nvX, uint nvY, uint offset)
{
#define IDX2(x, y) (IDX(x, y) + offset)
    /* Create nodes	*/

    btScalar stepX = (c00 - c10).norm() / (btScalar)(nvX - 1) ;
    btScalar stepY = (c00 - c01).norm() / (btScalar)(nvY - 1) ;

    Vector3f deltaX = (c10 - c00).normalized() * stepX ;
    Vector3f deltaY = (c01 - c00).normalized() * stepY ;

    Vector3f py = c00 ;
    for ( uint iy = 0; iy < nvY; ++iy, py += deltaY ) {
        Vector3f px = py ;
        for ( uint ix = 0; ix < nvX; ++ix, px += deltaX ) {
            nodes_[IDX2(ix, iy)] = toBulletVector(px) ;
            node_mass_[IDX2(ix, iy)] = 1;
        }
    }

    /* Create links	and faces */
    for ( uint iy = 0; iy < nvY; ++iy) {
        for (uint ix = 0; ix < nvX; ++ix) {
            const uint idx = IDX2(ix, iy);
            const bool mdx = (ix + 1) < nvX;
            const bool mdy = (iy + 1) < nvY;
            if (mdx) links_.emplace_back(idx, IDX2(ix + 1, iy));
            if (mdy) links_.emplace_back(idx, IDX2(ix, iy + 1));

            if (mdx && mdy) {
                if ((ix + iy) & 1)
                {
                    faces_.emplace_back(IDX2(ix, iy), IDX2(ix + 1, iy), IDX2(ix + 1, iy + 1));
                    faces_.emplace_back(IDX2(ix, iy), IDX2(ix + 1, iy + 1), IDX2(ix, iy + 1));

                 //   links_.emplace_back(IDX2(ix, iy), IDX2(ix + 1, iy + 1));

                }
                else
                {
                    faces_.emplace_back(IDX2(ix, iy + 1), IDX2(ix, iy), IDX2(ix + 1, iy));
                    faces_.emplace_back(IDX2(ix, iy + 1), IDX2(ix + 1, iy), IDX2(ix + 1, iy + 1));

                //    links_.emplace_back(IDX2(ix + 1, iy), IDX2(ix, iy + 1));

                }
            }
        }
    }
}

void createWorld() {
    // init physics

    physics.createSoftBodyDynamicsWorld();

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

    MaterialPtr material(new PhongMaterial({0, 1, 1, 1}));
    material->setSide(Material::Side::Both) ;

    std::shared_ptr<SoftBodyShape> sbs(new TwoPatchesTest());
    SoftBodyPtr cloth = physics.addSoftBody(SoftBodyBuilder()
                        .setName("cloth")
                        .setShape(sbs)
                        .setMargin(0.05)
                        .setMass(1)
                        .makeVisualShape(material)
                    );
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

