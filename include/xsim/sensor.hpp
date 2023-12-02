#pragma once

#include <xviz/scene/scene_fwd.hpp>
#include <xviz/scene/camera.hpp>
#include <xviz/gui/offscreen.hpp>
#include <xviz/scene/renderer.hpp>
#include <xsim/collision.hpp>

#include <Eigen/Geometry>
#include <memory>

#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>

namespace xsim {

class CollisionObject ;
class PhysicsWorld ;

class SensorEvent ;
using SensorEventListener = std::function<void(const SensorEvent *)> ;

class Sensor {
public:

    void setParent(CollisionObject *parent) {
        parent_ = parent ;
    }

    virtual void setPose(const Eigen::Isometry3f &pose) {
        pose_ = pose ;
    }

    Eigen::Isometry3f getWorldTransform() const;

    void setActive(bool v = true) { is_active_ = v ;}
    bool isActive() const { return is_active_ ; }

protected:
    bool is_active_ = true ;
    Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity() ;
    CollisionObject *parent_ = nullptr ;
};

using SensorPtr = std::shared_ptr<Sensor> ;


class CameraSensor: public Sensor {
public:
    CameraSensor(uint32_t width, uint32_t height, float fovy, float near, float far) ;

    void setScene(const xviz::NodePtr &scene) { scene_ = scene ; }

    xviz::NodePtr makeVisual(float d) const ;

protected:

    void doCapture() ;

protected:

    float width_, height_, fov_, near_, far_ ;
    xviz::NodePtr scene_ ;
    xviz::CameraPtr camera_ ;
    xviz::Renderer rdr_ ;
    std::unique_ptr<xviz::OffscreenSurface> surface_ ;
};

class RGBCameraSensor: public CameraSensor {
public:
    RGBCameraSensor(uint32_t width, uint32_t height, float fovy, float near, float far) ;

    xviz::Image capture()  ;
};

class RGBDCameraSensor: public CameraSensor {
public:
    RGBDCameraSensor(uint32_t width, uint32_t height, float fovy, float near, float far) ;

    std::pair<xviz::Image, xviz::Image> capture()  ;
};



class CollisionSensor: public Sensor {
public:
    CollisionSensor(CollisionShapePtr shape) ;

    bool hasCollisions() const { return !collisions_.empty() ; }
    size_t numCollisions() { return collisions_.size() ; }

    void init(PhysicsWorld &world) ;
    void testCollision() ;

protected:
    void doUpdate()  ;

private:

    CollisionShapePtr shape_;
    std::unique_ptr<btGhostObject> ghost_ ;
    std::vector<CollisionObject *> collisions_ ;
};


}
