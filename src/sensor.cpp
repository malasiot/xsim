#include <xsim/sensor.hpp>
#include <xsim/collision.hpp>
#include <xsim/world.hpp>

namespace xsim {

bool Sensor::needsUpdate(float ts) {
    if ( ts <= last_update_ ) return false ;
    return ( ts - last_update_ >= update_interval_ );
}

Eigen::Isometry3f Sensor::getWorldTransform() const {
    if ( parent_ == nullptr ) return pose_ ;
    else return parent_->getWorldTransform() * pose_ ;
}

void Sensor::update(float ts)
{
    if ( is_active_ && needsUpdate(ts) ) {
        doUpdate() ;
        last_update_ = ts ;
    }
}
/*
CameraSensor::CameraSensor(CameraPtr camera, NodePtr scene): camera_(camera), scene_(scene) {
    const Viewport &vp = camera_->getViewport() ;
    renderer_.reset(new OffscreenRenderer(vp.width_, vp.height_)) ;
    renderer_->setCamera(camera_) ;
}

cv::Mat CameraSensor::getImage() {
    return image_ ;
}

void CameraSensor::doUpdate() {
    camera_->setViewTransform(getWorldTransform().matrix());
    image_ = renderer_->getColor() ;
}
*/
CollisionSensor::CollisionSensor(CollisionShapePtr shape): shape_(shape) {
    ghost_.reset(new btGhostObject()) ;
    ghost_->setCollisionShape(shape_->handle()) ;
    ghost_->setCollisionFlags(ghost_->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    ghost_->setWorldTransform(toBulletTransform(getWorldTransform()));
}

void CollisionSensor::init(PhysicsWorld &w) {
    w.getDynamicsWorld()->addCollisionObject(ghost_.get(), btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger) ;
}

void CollisionSensor::doUpdate() {
    collisions_.clear() ;
    for( int i = 0; i < ghost_->getNumOverlappingObjects(); i++ ) {
        btCollisionObject *ob = ghost_->getOverlappingObject(i);
        CollisionObject *co = reinterpret_cast<CollisionObject *>(ob->getUserPointer()) ;
        if ( co )
            collisions_.push_back(co) ;
    }
}

}
