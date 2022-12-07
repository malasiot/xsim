#include <xsim/collision_space.hpp>
#include <xsim/convert.hpp>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

using namespace Eigen ;

namespace xsim {

struct CollisionData {
    std::string name_ ;
    std::unique_ptr<btCollisionObject> obj_ ;
    Isometry3f lt_ ;
};

struct CollisionFilterCallback : public btOverlapFilterCallback
{
    CollisionFilterCallback() {
    }

    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
    {
        assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
        assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);

        CollisionData *k0 = reinterpret_cast<CollisionData *>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
        CollisionData *k1 = reinterpret_cast<CollisionData *>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());

        return true ;
    }
};

class CollisionDispatcher : public btCollisionDispatcher  {
public:
    CollisionDispatcher(btCollisionConfiguration* collisionConfiguration): btCollisionDispatcher(collisionConfiguration) {}

#if BT_BULLET_VERSION > 279
    virtual bool needsCollision(const btCollisionObject* co0, const btCollisionObject* co1)
#else
    virtual bool needsCollision(btCollisionObject* co0, btCollisionObject* co1)
#endif
    {
        CollisionData *k0 = reinterpret_cast<CollisionData *>(co0->getUserPointer());
        CollisionData *k1 = reinterpret_cast<CollisionData *>(co1->getUserPointer());

        return true ;

    }

};

CollisionSpace::CollisionSpace() {
    broadphase_.reset(new btDbvtBroadphase());
    config_.reset(new btDefaultCollisionConfiguration())  ;
    dispatcher_.reset(new  CollisionDispatcher(config_.get()));
    world_.reset(new btCollisionWorld(dispatcher_.get(), broadphase_.get(), config_.get()));
}

CollisionSpace::~CollisionSpace() {

}

void CollisionSpace::addCollisionShape(const CollisionShapePtr &shape, const Isometry3f &tr) {
    shapes_.push_back(shape) ;

    btCollisionObject *collisionObj = new btCollisionObject() ;

    collisionObj->setCollisionShape(shape->handle()) ;

    Isometry3f lt = Isometry3f::Identity() ;

    btTransform c = toBulletTransform(tr *  lt)  ;
    collisionObj->setWorldTransform(c) ;

    std::unique_ptr<CollisionData> data(new CollisionData) ;
    data->lt_ = lt ;
    data->obj_.reset(collisionObj) ;

    collisionObj->setUserPointer(reinterpret_cast<void*>(data.get()));

    user_data_.emplace_back(std::move(data)) ;

    world_->addCollisionObject(collisionObj, int(btBroadphaseProxy::StaticFilter), int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter)) ;
}

bool CollisionSpace::hasCollision() {

    world_->getPairCache()->setOverlapFilterCallback(cb_.get());
  //  broadphase_->calculateOverlappingPairs(world_->getDispatcher());
    //dispatcher->setChecker(checker.get()) ;
    world_->performDiscreteCollisionDetection();

    int numManifolds = world_->getDispatcher()->getNumManifolds();
    for ( int i = 0; i < numManifolds; ++i ) {
        btPersistentManifold *contactManifold =
                world_->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject *obA = static_cast<const btCollisionObject *>(contactManifold->getBody0());
        const btCollisionObject *obB = static_cast<const btCollisionObject *>(contactManifold->getBody1());

        const CollisionData *coA = static_cast<const CollisionData *>(obA->getUserPointer());
        assert(coA != nullptr) ;

        const CollisionData *coB = static_cast<const CollisionData *>(obB->getUserPointer());
        //   assert(coB != nullptr) ;

        int numContacts = contactManifold->getNumContacts();

    }

}

}
