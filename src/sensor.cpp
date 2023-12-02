#include <xsim/sensor.hpp>
#include <xsim/collision.hpp>
#include <xsim/world.hpp>

using namespace std ;
using namespace xviz ;
using namespace Eigen ;

namespace xsim {

Eigen::Isometry3f Sensor::getWorldTransform() const {
    if ( parent_ == nullptr ) return pose_ ;
    else return parent_->getWorldTransform() * pose_ ;
}

CollisionSensor::CollisionSensor(CollisionShapePtr shape): shape_(shape) {
    ghost_.reset(new btGhostObject()) ;
    ghost_->setCollisionShape(shape_->handle()) ;
    ghost_->setCollisionFlags(ghost_->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    ghost_->setWorldTransform(toBulletTransform(getWorldTransform()));
}

void CollisionSensor::init(PhysicsWorld &w) {
    w.getDynamicsWorld()->addCollisionObject(ghost_.get(), btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger) ;
}

void CollisionSensor::testCollision() {
    collisions_.clear() ;
    for( int i = 0; i < ghost_->getNumOverlappingObjects(); i++ ) {
        btCollisionObject *ob = ghost_->getOverlappingObject(i);
        CollisionObject *co = reinterpret_cast<CollisionObject *>(ob->getUserPointer()) ;
        if ( co )
            collisions_.push_back(co) ;
    }
}

CameraSensor::CameraSensor(uint32_t width, uint32_t height, float fovy, float near, float far):
    width_(width), height_(height), fov_(fovy), near_(near), far_(far) {
    camera_.reset(new PerspectiveCamera(width_/static_cast<float>(height_), fov_, near_, far_)) ;
    camera_->setViewport(width, height) ;
    camera_->setBgColor({1, 1, 1, 1});

    surface_.reset(new OffscreenSurface(QSize(width, height))) ;
}

xviz::NodePtr CameraSensor::makeVisual(float d) const{

    float ratio = width_ / height_ ;
    float near_width = tan(fov_/2) * d;
    float near_height = near_width / ratio;

    GeometryPtr m(new Geometry(Geometry::Lines)) ;

    m->vertices() = { {0,  0, 0},
                               {near_width, near_height, -d}, {near_width, -near_height, -d},
                               {-near_width, -near_height, -d}, {-near_width, near_height, -d},
    {-near_width/8, near_height, -d},{near_width/8, near_height, -d}, {0, near_height + near_width/8, -d}

    };

    m->indices() = { 0, 1, 0, 2, 0, 3, 0, 4, 1, 2, 2, 3, 3, 4, 4, 1, 5, 6, 6, 7, 7, 5 } ;
   // m->indices() = { 3, 2, 1, 4, 3, 1 } ;

    NodePtr node(new Node) ;
    node->addDrawable(m, MaterialPtr(new ConstantMaterial({1, 0, 1, 1}))) ;

    node->setTransform(getWorldTransform().inverse()) ;

    return node ;
}


RGBCameraSensor::RGBCameraSensor(uint32_t width, uint32_t height, float fovy, float near, float far):
    CameraSensor(width, height, fovy, near, far) {
}

void CameraSensor::doCapture() {
    assert(scene_) ;
    auto wr = getWorldTransform() ;
    camera_->setViewTransform(wr.matrix()) ;
    surface_->use() ;
    rdr_.render(scene_, camera_) ;

}

Image RGBCameraSensor::capture() {
    if ( !is_active_ ) return Image() ;
    else {
        OffscreenSurface surface(QSize(width_, height_)) ;
        doCapture() ;
        auto im = surface.getImage(false) ;
        surface_->release() ;
        return im ;
    }
}

std::pair<Image, Image> RGBDCameraSensor::capture()
{
    if ( !is_active_ ) return make_pair(Image(), Image()) ;
    else {
        OffscreenSurface surface(QSize(width_, height_)) ;
        doCapture() ;
        auto p = make_pair(surface.getImage(false), surface.getDepthBuffer(near_, far_)) ;
        surface_->release() ;
        return p ;
    }
}

RGBDCameraSensor::RGBDCameraSensor(uint32_t width, uint32_t height, float fovy, float near, float far):
    CameraSensor(width, height, fovy, near, far) {
}

}
