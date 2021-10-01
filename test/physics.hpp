#ifndef BULLET_PHYSICS_HPP
#define BULLET_PHYSICS_HPP

#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <bullet/BulletSoftBody/btSoftBodyHelpers.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>

#include <memory>
#include <vector>

#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/physics/convert.hpp>

using namespace Eigen ;


class MotionState: public btMotionState {
    public:

    MotionState ( const cvx::viz::NodePtr &node): node_(node) {
        Affine3f world = node->globalTransform() ;
        world_trans_ = cvx::viz::toBulletTransform(world) ;

    }

    virtual void getWorldTransform( btTransform& centerOfMassWorldTrans ) const override {
        centerOfMassWorldTrans = world_trans_;
    }

    /// synchronizes world transform from physics to user
    /// Bullet only calls the update of worldtransform for active objects
    virtual void setWorldTransform( const btTransform& centerOfMassWorldTrans )
    {
        world_trans_ = centerOfMassWorldTrans;
        node_->matrix() = cvx::viz::toEigenTransform(world_trans_);
    }

private:

    btTransform	world_trans_;
    cvx::viz::NodePtr node_ ;
};


struct Physics
{
    std::unique_ptr<btBroadphaseInterface> m_broadphase ;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher ;
    std::unique_ptr<btConstraintSolver> m_solver ;
    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionConfiguration ;
    std::unique_ptr<btDynamicsWorld> m_dynamicsWorld;

    btSoftBodyWorldInfo softBodyWorldInfo;

    //data for picking objects
    class btRigidBody* m_pickedBody = nullptr;
    class btTypedConstraint* m_pickedConstraint = nullptr;
    int m_savedState;
    btVector3 m_oldPickingPos;
    btVector3 m_hitPos;
    btScalar m_oldPickingDist;



    btDynamicsWorld* getDynamicsWorld()
    {
        return m_dynamicsWorld.get();
    }


    virtual void createEmptyDynamicsWorld()
    {
        ///collision configuration contains default setup for memory, collision setup
        m_collisionConfiguration.reset( new btDefaultCollisionConfiguration() ) ;
        //m_collisionConfiguration->setConvexConvexMultipointIterations();

        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        m_dispatcher.reset( new btCollisionDispatcher(m_collisionConfiguration.get()) ) ;

        m_broadphase.reset(new btDbvtBroadphase() ) ;

        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        m_solver.reset( new btSequentialImpulseConstraintSolver() ) ;

        m_dynamicsWorld.reset( new btDiscreteDynamicsWorld(m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfiguration.get()));

        m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
    }

    virtual void createSoftBodyDynamicsWorld()
    {
        ///collision configuration contains default setup for memory, collision setup
        m_collisionConfiguration.reset( new btSoftBodyRigidBodyCollisionConfiguration() ) ;
        //m_collisionConfiguration->setConvexConvexMultipointIterations();

        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        m_dispatcher.reset( new btCollisionDispatcher(m_collisionConfiguration.get()) ) ;

        m_broadphase.reset(new btDbvtBroadphase() ) ;

        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        m_solver.reset( new btSequentialImpulseConstraintSolver() ) ;

        m_dynamicsWorld.reset( new btSoftRigidDynamicsWorld(m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfiguration.get()));

        m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

        softBodyWorldInfo.m_broadphase = m_broadphase.get();
        softBodyWorldInfo.m_dispatcher = m_dispatcher.get();
        softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
        softBodyWorldInfo.m_sparsesdf.Initialize();

    }

    void stepSimulation(float deltaTime)
    {
        if ( m_dynamicsWorld ) {
            m_dynamicsWorld->stepSimulation(deltaTime);
        }
    }

    void physicsDebugDraw(int debugFlags)
    {
        if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
        {
            m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
            m_dynamicsWorld->debugDrawWorld();
        }
    }

    void exitPhysics()
    {
     //   removePickingConstraint();

        //remove the rigidbodies from the dynamics world and delete them

        if ( m_dynamicsWorld )
        {
            int i;
            for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
            {
                m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
            }
            for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
            {
                btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
                btRigidBody* body = btRigidBody::upcast(obj);
                if (body && body->getMotionState())
                {
                    delete body->getMotionState();
                }
                m_dynamicsWorld->removeCollisionObject(obj);
                delete obj;
            }
        }

        m_dynamicsWorld.release() ;
        m_solver.release();
        m_broadphase.release();
        m_dispatcher.release();
        m_collisionConfiguration.release() ;

    }


    bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
    {
        if (m_pickedBody && m_pickedConstraint)
        {
            btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
            if (pickCon)
            {
                //keep it at the same picking distance

                btVector3 newPivotB;

                btVector3 dir = rayToWorld - rayFromWorld;
                dir.normalize();
                dir *= m_oldPickingDist;

                newPivotB = rayFromWorld + dir;
                pickCon->setPivotB(newPivotB);
                return true;
            }
        }
        return false;
    }

    bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
    {
        if (!m_dynamicsWorld) return false;

        btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);

        rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
        m_dynamicsWorld->rayTest(rayFromWorld, rayToWorld, rayCallback);
        if (rayCallback.hasHit())
        {
            btVector3 pickPos = rayCallback.m_hitPointWorld;
            btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
            if (body)
            {
                //other exclusions?
                if (!(body->isStaticObject() || body->isKinematicObject()))
                {
                    m_pickedBody = body;
                    m_savedState = m_pickedBody->getActivationState();
                    m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
                    //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                    btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
                    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                    m_dynamicsWorld->addConstraint(p2p, true);
                    m_pickedConstraint = p2p;
                    btScalar mousePickClamping = 30.f;
                    p2p->m_setting.m_impulseClamp = mousePickClamping;
                    //very weak constraint for picking
                    p2p->m_setting.m_tau = 0.001f;
                }
            }

            //					pickObject(pickPos, rayCallback.m_collisionObject);
            m_oldPickingPos = rayToWorld;
            m_hitPos = pickPos;
            m_oldPickingDist = (pickPos - rayFromWorld).length();
            //					printf("hit !\n");
            //add p2p
        }
        return false;
    }

    virtual void removePickingConstraint()
    {
        if (m_pickedConstraint)
        {
            m_pickedBody->forceActivationState(m_savedState);
            m_pickedBody->activate();
            m_dynamicsWorld->removeConstraint(m_pickedConstraint);
            delete m_pickedConstraint;
            m_pickedConstraint = 0;
            m_pickedBody = 0;
        }
    }

    btCollisionShape *createBoxShape(const btVector3& halfExtents)  {
        auto shape = new btBoxShape(halfExtents);
        collision_shapes_.addCollisionShape(shape);
        return shape ;
    }

    btCollisionShape *createCylinderShape(float radius, float len)  {
        auto shape = new btCylinderShape(btVector3(radius, len/2.0, radius));
        addCollisionShape(shape);
        return shape ;
    }

    void deleteRigidBody(btRigidBody* body)
    {
        m_dynamicsWorld->removeRigidBody(body);
        btMotionState* ms = body->getMotionState();
        delete body;
        delete ms;
    }

    btRigidBody* createRigidBody(float mass, const cvx::viz::NodePtr &node,  btCollisionShape* shape, const btVector3 &localInertia)
    {
        btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

        MotionState* myMotionState = new MotionState(node);

        btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

        btRigidBody* body = new btRigidBody(cInfo);
        //body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

        m_dynamicsWorld->addRigidBody(body);
        return body;
    }

    btRigidBody* createStaticRigidBody(const btTransform &startTransform, btCollisionShape* shape)
    {
        btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

        btVector3 localInertia(0, 0, 0);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

        btRigidBody::btRigidBodyConstructionInfo cInfo(btScalar(0.), myMotionState, shape, localInertia);

        btRigidBody* body = new btRigidBody(cInfo);

        body->setUserIndex(-1);
        m_dynamicsWorld->addRigidBody(body);
        return body;
    }

};









#endif
