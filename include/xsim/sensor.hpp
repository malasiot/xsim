#pragma once

#include <xviz/scene/scene_fwd.hpp>
#include <xviz/scene/camera.hpp>
#include <xsim/collision.hpp>

#include <Eigen/Geometry>
#include <memory>

#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>

namespace xsim {

class CollisionObject ;
class PhysicsWorld ;

class Sensor {
public:

    void setParent(CollisionObject *parent) {
        parent_ = parent ;
    }

    void setPose(const Eigen::Isometry3f &pose) {
        pose_ = pose ;
    }

    // called when adding a sensor in the world
    virtual void init(PhysicsWorld &world) {}

    // called when the world is destroyed
    virtual void destroy(PhysicsWorld &world) {}

    // called by physics thread
    void update(float ts) ;

protected:

    // override to perform the measurement
    virtual void doUpdate() = 0;

    Eigen::Isometry3f getWorldTransform() const ;

  private:

    // determine whether we need to update based on the given update interval
    bool needsUpdate(float ts) ;


    float update_interval_ = 0.f, last_update_ = 0.f ;
    bool is_active_ = true ;
    Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity() ;
    CollisionObject *parent_ = nullptr ;
};

using SensorPtr = std::shared_ptr<Sensor> ;

/*

class CameraSensor: public Sensor {
public:
    CameraSensor(CameraPtr camera, NodePtr scene) ;

    cv::Mat getImage() ;

protected:

    virtual void doUpdate() override ;

private:

    cv::Mat image_ ;
    CameraPtr camera_ ;
    NodePtr scene_ ;
    std::unique_ptr<OffscreenRenderer> renderer_ ;
};

*/

class CollisionSensor: public Sensor {
public:
    CollisionSensor(CollisionShapePtr shape) ;

    bool hasCollisions() const { return !collisions_.empty() ; }
    size_t numCollisions() { return collisions_.size() ; }

    void init(PhysicsWorld &world) override;

protected:
    virtual void doUpdate() override ;

private:

    CollisionShapePtr shape_;
    std::unique_ptr<btGhostObject> ghost_ ;
    std::vector<CollisionObject *> collisions_ ;
};


}
