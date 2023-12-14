#include <xsim/world.hpp>
#include <xsim/multi_body.hpp>
#include <xsim/convert.hpp>
#include <xsim/soft_body.hpp>
#include <xviz/scene/node.hpp>

#include <bullet/BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>
#include <bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <bullet/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <bullet/BulletDynamics/MLCPSolvers/btLemkeSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <bullet/BulletSoftBody/btSoftMultiBodyDynamicsWorld.h>

#include <xsim/sdf_parser.hpp>

using namespace std ;
using namespace Eigen ;
using namespace xviz ;

namespace xsim {

PhysicsWorld::PhysicsWorld() {
    visual_.reset(new Node) ;
}

void PhysicsWorld::tickCallback(btDynamicsWorld *world, btScalar step) {
    PhysicsWorld *w = static_cast<PhysicsWorld *>(world->getWorldUserInfo()) ;
    w->updateSimTime(step) ;

    w->queryCollisions() ;

}

void PhysicsWorld::updateSimTime(float step) {
    sim_time_ += step ;
}

float PhysicsWorld::getSimTime() const {
    return sim_time_ ;
}

void PhysicsWorld::createDefaultDynamicsWorld() {
    collision_config_.reset( new btDefaultCollisionConfiguration() ) ;

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher_.reset( new btCollisionDispatcher(collision_config_.get()) ) ;

    //  broadphase_.reset(new btDbvtBroadphase() ) ;

    btVector3 worldAabbMin(-10000, -10000, -10000);
    btVector3 worldAabbMax(10000, 10000, 10000);
    broadphase_.reset(new btAxisSweep3(worldAabbMin, worldAabbMax));


    // solver_.reset(new btNNCGConstraintSolver());
    //solver_.reset(new btMLCPSolver(new btSolveProjectedGaussSeidel()));
    solver_.reset(new btMLCPSolver(new btDantzigSolver()));
    //m_solver = new btMLCPSolver(new btLemkeSolver());

    solver_.reset( new btSequentialImpulseConstraintSolver() ) ;

    dynamics_world_.reset( new btDiscreteDynamicsWorld(dispatcher_.get(), broadphase_.get(), solver_.get(), collision_config_.get()));
    dynamics_world_->getDispatchInfo().m_useContinuous = true;

    dynamics_world_->setGravity(btVector3(0, -10, 0));

    dynamics_world_->setInternalTickCallback(tickCallback, static_cast<void *>(this)) ;

    broadphase_->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

}


enum MyFilterModes
{
    FILTER_GROUPAMASKB_AND_GROUPBMASKA2 = 0,
    FILTER_GROUPAMASKB_OR_GROUPBMASKA2
};

struct MyOverlapFilterCallback2 : public btOverlapFilterCallback
{
    int m_filterMode = FILTER_GROUPAMASKB_OR_GROUPBMASKA2 ;

    MyOverlapFilterCallback2()
        : m_filterMode(FILTER_GROUPAMASKB_AND_GROUPBMASKA2)
    {
    }

    virtual ~MyOverlapFilterCallback2()
    {
    }
    // return true when pairs need collision
    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
    {

        int objectUniqueIdB = -1, linkIndexB = -1;
        btCollisionObject* colObjB = (btCollisionObject*)proxy1->m_clientObject;
        btMultiBodyLinkCollider* mblB = btMultiBodyLinkCollider::upcast(colObjB);
        if (mblB)
        {
            objectUniqueIdB = mblB->m_multiBody->getUserIndex();
            MultiBody *mbB = reinterpret_cast<MultiBody *>(mblB->m_multiBody->getUserPointer());
            linkIndexB = mblB->m_link;
        }
        else
        {
            objectUniqueIdB = colObjB->getUserIndex();
            linkIndexB = -1;
        }
        int objectUniqueIdA = -1, linkIndexA = -1;
        btCollisionObject* colObjA = (btCollisionObject*)proxy0->m_clientObject;
        btMultiBodyLinkCollider* mblA = btMultiBodyLinkCollider::upcast(colObjA);
        if (mblA)
        {
            objectUniqueIdA = mblA->m_multiBody->getUserIndex();
            MultiBody *mbA = reinterpret_cast<MultiBody *>(mblA->m_multiBody->getUserPointer());
            linkIndexA = mblA->m_link;
        }
        else
        {
            objectUniqueIdA = colObjA->getUserIndex();
            linkIndexA = -1;
        }
        int collisionFilterGroupA = proxy0->m_collisionFilterGroup;
        int collisionFilterMaskA = proxy0->m_collisionFilterMask;
        int collisionFilterGroupB = proxy1->m_collisionFilterGroup;
        int collisionFilterMaskB = proxy1->m_collisionFilterMask;


        if (m_filterMode == FILTER_GROUPAMASKB_AND_GROUPBMASKA2)
        {
            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            return collides;
        }

        if (m_filterMode == FILTER_GROUPAMASKB_OR_GROUPBMASKA2)
        {
            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            collides = collides || (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            return collides;
        }


        return false;
    }
};


const int maxProxies = 32766;

void PhysicsWorld::createMultiBodyDynamicsWorld()
{
    collision_config_.reset( new btDefaultCollisionConfiguration() ) ;

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher_.reset( new btCollisionDispatcher(collision_config_.get()) ) ;


    broadphase_.reset(new btDbvtBroadphase() ) ;


    auto cache = broadphase_->getOverlappingPairCache() ;
    cache->setOverlapFilterCallback(new MyOverlapFilterCallback2);

    solver_.reset(new btMultiBodyConstraintSolver()) ;

    dynamics_world_.reset( new btMultiBodyDynamicsWorld(dispatcher_.get(), broadphase_.get(), static_cast<btMultiBodyConstraintSolver *>(solver_.get()), collision_config_.get()));
    dynamics_world_->getDispatchInfo().m_useContinuous = false;
 //   dynamics_world_->getDispatchInfo().m_allowedCcdPenetration = 0.01;
    dynamics_world_->getSolverInfo().m_numIterations = 100;

    dynamics_world_->setGravity(btVector3(0, -10, 0));

    dynamics_world_->setInternalTickCallback(tickCallback, static_cast<void *>(this)) ;

  //  broadphase_->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
}



void PhysicsWorld::createSoftBodyDynamicsWorld()
{
    collision_config_.reset( new btSoftBodyRigidBodyCollisionConfiguration() ) ;
    dispatcher_.reset( new btCollisionDispatcher(collision_config_.get()) ) ;


    soft_body_world_info_.m_dispatcher = dispatcher_.get() ;

    btVector3 worldAabbMin(-1000, -1000, -1000);
    btVector3 worldAabbMax(1000, 1000, 1000);

    broadphase_.reset(new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies));

    soft_body_world_info_.m_broadphase = broadphase_.get();

    solver_.reset(new btSequentialImpulseConstraintSolver());

    btSoftBodySolver* softBodySolver = 0;

    dynamics_world_.reset( new btSoftRigidDynamicsWorld(dispatcher_.get(), broadphase_.get(), solver_.get(), collision_config_.get(), softBodySolver));


    dynamics_world_->setGravity(btVector3(0, -10, 0));

    dynamics_world_->setInternalTickCallback(tickCallback, static_cast<void *>(this)) ;

    soft_body_world_info_.m_gravity.setValue(0, -10, 0) ;
    soft_body_world_info_.m_sparsesdf.Initialize() ;

    soft_body_world_info_.m_sparsesdf.Reset();

    soft_body_world_info_.air_density = (btScalar)1.2;
    soft_body_world_info_.water_density = 0;
    soft_body_world_info_.water_offset = 0;
    soft_body_world_info_.water_normal = btVector3(0, 0, 0);

}

void PhysicsWorld::createSoftMultiBodyDynamicsWorld()
{
    collision_config_.reset( new btSoftBodyRigidBodyCollisionConfiguration() ) ;
    dispatcher_.reset( new btCollisionDispatcher(collision_config_.get()) ) ;

    soft_body_world_info_.m_dispatcher = dispatcher_.get() ;

    btVector3 worldAabbMin(-1000, -1000, -1000);
    btVector3 worldAabbMax(1000, 1000, 1000);

    broadphase_.reset(new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies));

    soft_body_world_info_.m_broadphase = broadphase_.get();

    solver_.reset(new btMultiBodyConstraintSolver());

    btSoftBodySolver* softBodySolver = 0;

    dynamics_world_.reset( new btSoftMultiBodyDynamicsWorld(dispatcher_.get(), broadphase_.get(),
                                                            reinterpret_cast<btMultiBodyConstraintSolver*>(solver_.get()), collision_config_.get(), softBodySolver));


    dynamics_world_->setGravity(btVector3(0, -10, 0));

    dynamics_world_->setInternalTickCallback(tickCallback, static_cast<void *>(this)) ;

    soft_body_world_info_.m_gravity.setValue(0, -10, 0) ;
    soft_body_world_info_.m_sparsesdf.Initialize() ;

    soft_body_world_info_.m_sparsesdf.Reset();

    soft_body_world_info_.air_density = (btScalar)1.2;
    soft_body_world_info_.water_density = 0;
    soft_body_world_info_.water_offset = 0;
    soft_body_world_info_.water_normal = btVector3(0, 0, 0);

}

void PhysicsWorld::resetSimulation() {
    solver_->reset();
    dynamics_world_->clearForces();
    broadphase_->resetPool(dispatcher_.get());

}

void PhysicsWorld::initFromSDF(const std::string &sdf_file_path) {
    SDFWorld world ;
    world.parse(sdf_file_path) ;
}

PhysicsWorld::~PhysicsWorld()
{
}

btDynamicsWorld *PhysicsWorld::getDynamicsWorld() {
    return dynamics_world_.get();
}

void PhysicsWorld::setGravity(const Vector3f &g) {
    btVector3 bg = toBulletVector(g);
    dynamics_world_->setGravity(bg) ;

    soft_body_world_info_.m_gravity = bg ;
}

void PhysicsWorld::stepSimulation(float deltaTime) {
    if ( dynamics_world_ ) {
        dynamics_world_->stepSimulation(deltaTime, 10, 1. / 240.);
        for( auto mb: multi_bodies_ ) {
            for( const Link &l: mb->links_ ) {
                if ( l.collider_ && l.visual_ ) {
                    Isometry3f wtr(toEigenTransform(l.collider_->getWorldTransform())) ;
                    l.visual_->setTransform(wtr) ;
                }
            }
        }

        for( auto sb: soft_bodies_ ) {
            sb->updateVisualGeometry();
        }

        if ( update_cb_ ) update_cb_() ;
    }
}

class ContactResultCallback: public btCollisionWorld::ContactResultCallback {
public:

    ContactResultCallback(PhysicsWorld &w, std::vector<ContactResult> &contacts): results_(contacts), world_(w) {}

    btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) override {
        const btRigidBody *obA =  btRigidBody::upcast(colObj0Wrap->getCollisionObject()) ;
        const btRigidBody *obB =  btRigidBody::upcast(colObj1Wrap->getCollisionObject()) ;
        const btMultiBodyLinkCollider *colA = dynamic_cast<const btMultiBodyLinkCollider *>(colObj0Wrap->getCollisionObject());
        const btMultiBodyLinkCollider *colB = dynamic_cast<const btMultiBodyLinkCollider *>(colObj1Wrap->getCollisionObject());

        CollisionObject *pA = reinterpret_cast<CollisionObject *>(colObj0Wrap->getCollisionObject()->getUserPointer()) ;
        CollisionObject *pB = reinterpret_cast<CollisionObject *>(colObj1Wrap->getCollisionObject()->getUserPointer()) ;

        ContactResult result ;
        result.a_ = pA ;
        result.b_ = pB ;

        result.pa_ =  toEigenVector(cp.getPositionWorldOnA()) ;
        result.pb_ = toEigenVector(cp.getPositionWorldOnB()) ;
        result.normal_ = toEigenVector(cp.m_normalWorldOnB);

        results_.emplace_back(std::move(result)) ;


        return 0 ;
    }

    std::vector<ContactResult> &results_ ;
    PhysicsWorld &world_ ;


};

bool PhysicsWorld::contactTest(const RigidBodyPtr &b1, std::vector<ContactResult> &results) {
    ContactResultCallback cb(*this, results) ;
    dynamics_world_->contactTest(b1->handle(), cb) ;

    return results.size() ;
}

bool PhysicsWorld::contactPairTest(const RigidBodyPtr &b1, const RigidBodyPtr &b2, float thresh, std::vector<ContactResult> &results) {
    ContactResultCallback cb(*this, results) ;
    cb.m_closestDistanceThreshold = thresh ;
    dynamics_world_->contactPairTest(b1->handle(), b2->handle(), cb) ;
    return results.size() ;
}

bool PhysicsWorld::contactPairTest(const RigidBodyPtr &b1, const Link *b2, float thresh, std::vector<ContactResult> &results) {
    ContactResultCallback cb(*this, results) ;
    cb.m_closestDistanceThreshold = thresh ;
    dynamics_world_->contactPairTest(b1->handle(), b2->collider_.get(), cb) ;
    return results.size() ;
}


bool PhysicsWorld::rayPick(const Vector3f &origin, const Vector3f &dir, RayHitResult &res)
{
    btVector3 rayFromWorld = eigenVectorToBullet(origin) ;
    btVector3 rayToWorld = eigenVectorToBullet(dir*10000) ;

    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);

    rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
    dynamics_world_->rayTest(rayFromWorld, rayToWorld, rayCallback);

    if (rayCallback.hasHit()) {
        Vector3f pickPos = toEigenVector(rayCallback.m_hitPointWorld);
        Vector3f pickNorm = toEigenVector(rayCallback.m_hitNormalWorld) ;
        const btCollisionObject* body = rayCallback.m_collisionObject;

        res.p_ = pickPos ;
        res.n_ = pickNorm ;
        res.o_ = reinterpret_cast<CollisionObject *>(body->getUserPointer()) ;

        return true ;
    }

    return false ;
}


void PhysicsWorld::addCollisionShape(const btCollisionShape *shape) {
    collision_shapes_.push_back(shape) ;
}

RigidBodyPtr PhysicsWorld::addRigidBody(const RigidBodyBuilder &rb) {

    RigidBodyPtr body(new RigidBody(rb)) ;

    dynamics_world_->addRigidBody(body->handle());

    if ( body->visual_shape_ ) visual_->addChild(body->visual_shape_) ;

    uint idx = bodies_.size() ;
    bodies_.emplace_back(body) ;
    body->handle()->setUserIndex(idx) ;
    body->handle()->setUserPointer(reinterpret_cast<void *>(body.get())) ;

     string bname = body->getName() ;
    if ( !bname.empty() )
        body_map_.emplace(bname, idx) ;
    return body ;
}

RigidBodyPtr PhysicsWorld::addRigidBody(const CollisionShapePtr &shape, const Eigen::Isometry3f &tr) {
    return addRigidBody(RigidBodyBuilder().setCollisionShape(shape).setWorldTransform(tr)) ;
}

RigidBodyPtr PhysicsWorld::addRigidBody(btScalar mass, RigidBodyStateObserver *ms, const CollisionShapePtr &shape, const Eigen::Isometry3f &tr, const Eigen::Vector3f &localInertia) {
    return addRigidBody(RigidBodyBuilder().setMass(mass).setCollisionShape(shape).setMotionStateObserver(ms).setLocalInertia(localInertia).setWorldTransform(tr)) ;
}

RigidBodyPtr PhysicsWorld::addRigidBody(btScalar mass, RigidBodyStateObserver *ms, const CollisionShapePtr &shape, const Eigen::Isometry3f &tr) {
    return addRigidBody(RigidBodyBuilder().setMass(mass).setCollisionShape(shape).setMotionStateObserver(ms).setWorldTransform(tr)) ;
}

MultiBodyPtr PhysicsWorld::addMultiBody(const MultiBodyBuilder &mb) {
    MultiBodyPtr body(new MultiBody) ;
    body->create(mb, *this) ;
    uint idx = multi_bodies_.size() ;
    multi_bodies_.emplace_back(body) ;

    body->handle()->setUserIndex(idx) ;
    body->handle()->setUserPointer(body.get());
    string bname = body->name() ;
    if ( !bname.empty() )
        multi_body_map_.emplace(bname, idx) ;

    return body ;
}

SoftBodyPtr PhysicsWorld::addSoftBody(const SoftBodyBuilder &b) {

    SoftBodyPtr body(new SoftBody(b, *this)) ;

    if ( btSoftRigidDynamicsWorld *dw = dynamic_cast<btSoftRigidDynamicsWorld *>(dynamics_world_.get()) ) {
        dw->addSoftBody(body->handle());
    } else if ( btSoftMultiBodyDynamicsWorld *dw = dynamic_cast<btSoftMultiBodyDynamicsWorld *>(dynamics_world_.get())) {
        dw->addSoftBody(body->handle());
    } else return nullptr ;

    if ( body->visual_ ) {
        body->updateVisualGeometry();
        visual_->addChild(body->visual_) ;
    }

    uint idx = soft_bodies_.size() ;
    soft_bodies_.emplace_back(body) ;
    body->handle()->setUserIndex(idx) ;
    body->handle()->setUserPointer(reinterpret_cast<void *>(body.get())) ;
    string bname = body->getName() ;
    if ( !bname.empty() )
        soft_body_map_.emplace(bname, idx) ;
    return body ;
}

uint PhysicsWorld::addGhost(const GhostObjectPtr &ghost)
{
     dynamics_world_->addCollisionObject(ghost->handle(), btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger) ;

     return 0 ;
}

void PhysicsWorld::addConstraint(const Constraint &c) {
    dynamics_world_->addConstraint(c.handle());
    constraints_.emplace_back(std::move(c)) ;
}

RigidBodyPtr PhysicsWorld::getRigidBody(uint idx) const {
    return bodies_[idx];
}


SoftBodyPtr PhysicsWorld::getSoftBody(uint idx) const {
    return soft_bodies_[idx];
}


MultiBodyPtr PhysicsWorld::getMultiBody(uint idx) const {
    return multi_bodies_[idx];
}

RigidBodyPtr PhysicsWorld::findRigidBody(const string &name) {
    auto it = body_map_.find(name) ;
    if ( it != body_map_.end() )
        return getRigidBody(it->second) ;
    else
        return nullptr ;
}

SoftBodyPtr PhysicsWorld::findSoftBody(const string &name) {
    auto it = soft_body_map_.find(name) ;
    if ( it != soft_body_map_.end() )
        return getSoftBody(it->second) ;
    else
        return nullptr ;
}


MultiBodyPtr PhysicsWorld::findMultiBody(const string &name) {
    auto it = multi_body_map_.find(name) ;
    if ( it != multi_body_map_.end() )
        return getMultiBody(it->second) ;
    else
        return nullptr ;
}

struct btCollisionFilter : public btOverlapFilterCallback
{
    CollisionFilter *filter_ = nullptr ;

    btCollisionFilter(CollisionFilter *f): filter_(f) {}

    // return true when pairs need collision
    virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0,  btBroadphaseProxy *proxy1) const  {
        assert(proxy0 != NULL && proxy1 != NULL) ;

        bool collide = (proxy0->m_collisionFilterGroup
                        & proxy1->m_collisionFilterMask) != 0;
        collide = collide && ( proxy1->m_collisionFilterGroup
                               & proxy0->m_collisionFilterMask);

        btCollisionObject *ob0 = static_cast<btCollisionObject *>(proxy0->m_clientObject);
        if ( !ob0 ) return collide ;

        btCollisionObject *ob1 = static_cast<btCollisionObject *>(proxy1->m_clientObject);
        if ( !ob1 ) return collide ;

        CollisionObject *co0 = static_cast<CollisionObject *>(ob0->getUserPointer());
        assert(co0 != nullptr) ;

        CollisionObject *co1 = static_cast<CollisionObject *>(ob1->getUserPointer());
        assert(co1 != nullptr) ;

        return filter_->collide(co0, co1) ;
    }
};

void PhysicsWorld::setCollisionFilter(CollisionFilter *f) {

    if ( f ) filter_callback_.reset(new btCollisionFilter(f));
    else filter_callback_.reset() ;

    btOverlappingPairCache* pair_cache = dynamics_world_->getPairCache();

    assert(pair_cache != nullptr ) ;

    pair_cache->setOverlapFilterCallback(filter_callback_.get());
}


void PhysicsWorld::setUpdateCallback(PhysicsWorld::UpdateCallback cb) {
    update_cb_ = cb ;
}

void PhysicsWorld::setCollisionFeedback(CollisionFeedback *feedback) {
    collision_feedback_ = feedback ;
}

void PhysicsWorld::queryCollisions() {
    if  ( !collision_feedback_ ) return ;

    int numManifolds = dynamics_world_->getDispatcher()->getNumManifolds();
    for ( int i = 0; i < numManifolds; ++i ) {
        btPersistentManifold *contactManifold =
            dynamics_world_->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject *obA = static_cast<const btCollisionObject *>(contactManifold->getBody0());
        const btCollisionObject *obB = static_cast<const btCollisionObject *>(contactManifold->getBody1());

        const CollisionObject *coA = static_cast<const CollisionObject *>(obA->getUserPointer());
        assert(coA != nullptr) ;

        const CollisionObject *coB = static_cast<const CollisionObject *>(obB->getUserPointer());
     //   assert(coB != nullptr) ;

        int numContacts = contactManifold->getNumContacts();

        for (int j = 0; j < numContacts; ++j) {
            btManifoldPoint &pt = contactManifold->getContactPoint(j);
            if (pt.getDistance() < 0.f)  {
                ContactResult result ;
                result.a_ = coA ;
                result.b_ = coB ;

                result.pa_ =  toEigenVector(pt.getPositionWorldOnA()) ;
                result.pb_ = toEigenVector(pt.getPositionWorldOnB()) ;
                result.normal_ = toEigenVector(pt.m_normalWorldOnB);

                collision_feedback_->processContact(result) ;
            }
        }
      }


}


bool RayPicker::movePickedBody(const Ray &ray)
{
    btVector3 rayFromWorld = eigenVectorToBullet(ray.origin()) ;
    btVector3 rayToWorld = eigenVectorToBullet(ray.dir()*10000) ;

    if (picked_body_ && picked_constraint_)
    {
        btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(picked_constraint_);
        if (pickCon)
        {
            //keep it at the same picking distance

            btVector3 newPivotB;

            btVector3 dir = rayToWorld - rayFromWorld;
            dir.normalize();
            dir *= old_picking_dist_ ;;

            newPivotB = rayFromWorld + dir;
            pickCon->setPivotB(newPivotB);
            return true;
        }
    }

    if ( mb_picked_constraint_ ) {
        //keep it at the same picking distance

        btVector3 dir = rayToWorld - rayFromWorld;
        dir.normalize();
        dir *= old_picking_dist_ ;

        btVector3 newPivotB = rayFromWorld + dir;

        mb_picked_constraint_->setPivotInB(newPivotB);
    }

    return false;

}

bool RayPicker::pickBody(const Ray &ray)
{
    if (!world_) return false;

    btVector3 rayFromWorld = eigenVectorToBullet(ray.origin()) ;
    btVector3 rayToWorld = eigenVectorToBullet(ray.dir()*10000) ;

    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);

    rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
    world_->rayTest(rayFromWorld, rayToWorld, rayCallback);
    if (rayCallback.hasHit())
    {
        btVector3 pickPos = rayCallback.m_hitPointWorld;
        btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body)
        {
            //other exclusions?
            if (!(body->isStaticObject() || body->isKinematicObject()))
            {
                picked_body_ = body;
                saved_state_ = picked_body_->getActivationState();
                picked_body_->setActivationState(DISABLE_DEACTIVATION);
                //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
                btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                world_->addConstraint(p2p, true);
                picked_constraint_ = p2p;
                btScalar mousePickClamping = 30.f;
                p2p->m_setting.m_impulseClamp = mousePickClamping;
                //very weak constraint for picking
                p2p->m_setting.m_tau = 0.001f;
            }
        } else {
            btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
            if (multiCol && multiCol->m_multiBody)
            {
                prev_can_sleep_ = multiCol->m_multiBody->getCanSleep();
                multiCol->m_multiBody->setCanSleep(false);

                btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

                btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody, multiCol->m_link, 0, pivotInA, pickPos);
                //if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
                //see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
                //so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
                //it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)
                btScalar scaling = 1;
                p2p->setMaxAppliedImpulse(2 * scaling);

                btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)world_;
                world->addMultiBodyConstraint(p2p);

                mb_picked_constraint_ = p2p;

            }
        }

        //					pickObject(pickPos, rayCallback.m_collisionObject);
        old_picking_pos_ = rayToWorld;
        hit_pos_ = pickPos;
        old_picking_dist_ = (pickPos - rayFromWorld).length();
        //					printf("hit !\n");
        //add p2p
    }
    return false;
}

void RayPicker::removePickingConstraint()
{
    if (picked_constraint_)
    {
        picked_body_->forceActivationState(saved_state_);
        picked_body_->activate();
        world_->removeConstraint(picked_constraint_);
        delete picked_constraint_;
        picked_constraint_ = nullptr ;
        picked_body_ = nullptr;
    }

    if ( mb_picked_constraint_ ) {
        mb_picked_constraint_->getMultiBodyA()->setCanSleep(prev_can_sleep_);
        btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)world_;
        world->removeMultiBodyConstraint(mb_picked_constraint_);
        delete mb_picked_constraint_ ;
        mb_picked_constraint_ = nullptr ;
    }

}

void RigidBodyUpdateScene::notify(const Isometry3f &tr)
{
    Node *parent = node_->parent() ;
    Matrix4f trs = ( parent == nullptr ) ? tr.matrix() : parent->globalTransform().matrix().inverse() * tr.matrix() ;
    node_->setTransform( Affine3f(trs) );
}

/*
Isometry3f UpdateSceneMotionState::getTransform() const {
    auto a = node_->transform() ;
    Isometry3f b;
    b.translation() = a.translation();
    b.linear() = a.rotation();
    return b ;
}
*/

}
