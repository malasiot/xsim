#ifndef XVIZ_PHYSICS_WORLD_HPP
#define XVIZ_PHYSICS_WORLD_HPP

#include <memory>

#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <bullet/BulletSoftBody/btSoftBodyHelpers.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>

#include <xsim/collision.hpp>
#include <xsim/rigid_body.hpp>
#include <xsim/soft_body.hpp>
#include <xsim/multi_body.hpp>
#include <xsim/constraints.hpp>
#include <xsim/sensor.hpp>
#include <xviz/scene/camera.hpp>

namespace xsim {


struct RayHitResult {
    CollisionObject *o_ ; // hit object
    Eigen::Vector3f p_, n_ ; // position and normal of hit in world coordinates
};

class PhysicsWorld {
public:
    PhysicsWorld() ;
    ~PhysicsWorld() ;

    using UpdateCallback = std::function<void()> ;

    void createDefaultDynamicsWorld();
    void createMultiBodyDynamicsWorld();
    void createSoftBodyDynamicsWorld() ;
    void createSoftMultiBodyDynamicsWorld() ;

    void resetSimulation() ;

    btDynamicsWorld* getDynamicsWorld();

    xviz::NodePtr getVisual() const { return visual_ ; }

    void setGravity(const Eigen::Vector3f &g) ;

    void stepSimulation(float deltaTime);

    // test contact of body with world
    bool contactTest(const RigidBodyPtr &b1, std::vector<ContactResult> &results) ;

    // Test contact between pair of collision objects (up to a distance threshold)
    // Objects can be rigidbodies or multi-body links
    bool contactPairTest(const RigidBodyPtr &b1, const RigidBodyPtr &b2, float threshold, std::vector<ContactResult> &results);
    bool contactPairTest(const RigidBodyPtr &b1, const Link *b2, float threshold, std::vector<ContactResult> &results);
    bool contactPairTest(const Link *b1, const Link *b2, float threshold, std::vector<ContactResult> &results);

    bool rayPick(const Eigen::Vector3f &origin, const Eigen::Vector3f &dir, RayHitResult &res);


    RigidBodyPtr addRigidBody(const RigidBodyBuilder &rb) ;

    // rigid body helpers

    // dynamic body
    RigidBodyPtr addRigidBody(btScalar mass, RigidBodyStateObserver *ms, const CollisionShapePtr &shape, const Eigen::Isometry3f &tr, const Eigen::Vector3f &localInertia) ;
    RigidBodyPtr addRigidBody(btScalar mass, RigidBodyStateObserver *ms, const CollisionShapePtr &shape, const Eigen::Isometry3f &tr) ;

    // static body
    RigidBodyPtr addRigidBody(const CollisionShapePtr &shape, const Eigen::Isometry3f &tr) ;

    MultiBodyPtr addMultiBody(const MultiBodyBuilder &body) ;
    SoftBodyPtr addSoftBody(const SoftBodyBuilder &body);

    uint addGhost(const GhostObjectPtr &ghost) ;

    void addConstraint(const Constraint &c);

    RigidBodyPtr getRigidBody(uint idx) const ;
    SoftBodyPtr getSoftBody(uint idx) const ;
    MultiBodyPtr getMultiBody(uint idx) const ;

    RigidBodyPtr findRigidBody(const std::string &name) ;
    SoftBodyPtr findSoftBody(const std::string &name) ;
    MultiBodyPtr findMultiBody(const std::string &name) ;

    // Call this to receive reports on collisions after each step
    void setCollisionFeedback(CollisionFeedback *feedback) ;

    // call this to disable collision among pairs of objects
    void setCollisionFilter(CollisionFilter *f) ;

    btSoftBodyWorldInfo &getSoftBodyWorldInfo() { return soft_body_world_info_ ; }

    void setUpdateCallback(UpdateCallback cb) ;

private:

    void addCollisionShape(const btCollisionShape *shape);
    void queryCollisions();
    static void tickCallback(btDynamicsWorld *world, btScalar step);
    void updateSimTime(float step) ;
    float getSimTime() const ;

    btAlignedObjectArray<const btCollisionShape *> collision_shapes_ ;
    std::unique_ptr<btBroadphaseInterface> broadphase_ ;
    std::unique_ptr<btCollisionDispatcher> dispatcher_ ;
    std::unique_ptr<btConstraintSolver> solver_ ;
    std::unique_ptr<btDefaultCollisionConfiguration> collision_config_ ;
    std::unique_ptr<btDynamicsWorld> dynamics_world_;

    xviz::NodePtr visual_ ;
    std::vector<RigidBodyPtr> bodies_ ;
    std::map<std::string, uint> body_map_ ;
    std::vector<SoftBodyPtr> soft_bodies_ ;
    std::map<std::string, uint> soft_body_map_ ;
    std::vector<MultiBodyPtr> multi_bodies_ ;
    std::map<std::string, uint> multi_body_map_ ;
    std::vector<GhostObjectPtr> ghosts_ ;

    std::vector<Constraint> constraints_ ;
    std::unique_ptr<btOverlapFilterCallback> filter_callback_ ;
    std::unique_ptr<btInternalTickCallback> tick_callback_ ;
    CollisionFeedback *collision_feedback_ = nullptr ;

    btSoftBodyWorldInfo soft_body_world_info_ ;
    UpdateCallback update_cb_ = nullptr ;
    float sim_time_ = 0.0f;


};



class RayPicker {
public:
    RayPicker(PhysicsWorld &world): world_(world.getDynamicsWorld()) {}

    bool movePickedBody(const xviz::Ray &ray) ;

    bool pickBody(const xviz::Ray &ray) ;

    void removePickingConstraint() ;

private:

    btDynamicsWorld *world_ ;
    btRigidBody* picked_body_ = nullptr;
    btTypedConstraint* picked_constraint_ = nullptr ;
    btMultiBodyPoint2Point *mb_picked_constraint_ = nullptr ;
    int saved_state_;
    btVector3 old_picking_pos_;
    btVector3 hit_pos_;
    btScalar old_picking_dist_;
    bool prev_can_sleep_ ;
};


} // namespace xviz


#endif
