#include <xsim/collision_space.hpp>
#include <xsim/convert.hpp>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <iostream>
#include <fstream>
#include <set>

using namespace Eigen ;
using namespace std ;
using namespace xviz ;

namespace xsim {

struct CollisionObjectPrivate {
    std::string name_ ;
    std::unique_ptr<btCollisionObject> obj_ ;
    CollisionShapePtr shape_ ;
};

class CollisionLinkChecker {
public:



};

struct CollisionFilterCallback : public btOverlapFilterCallback
{
    CollisionFilterCallback() {
    }

    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
    {
        assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
        assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);

        CollisionObjectPrivate *k0 = reinterpret_cast<CollisionObjectPrivate *>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
        CollisionObjectPrivate *k1 = reinterpret_cast<CollisionObjectPrivate *>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());

        if ( isActive(k0->name_) && isActive(k1->name_) && isActive(k0->name_, k1->name_) ) {
 //          cout << k0->name_ << ' ' << k1->name_ << endl ;
            return true ;
        }


        return false ;
    }

    void excludeLink(const string &link_name) {
        exclusions_.insert(link_name) ;
    }

    void excludeLinkPair(const string &link1, const std::string &link2) {
        expairs_.insert(make_pair(link1, link2)) ;
        expairs_.insert(make_pair(link2, link1)) ;
    }

    void includeLinkPair(const string &link1, const std::string &link2) {
        expairs_.erase(make_pair(link1, link2)) ;
        expairs_.erase(make_pair(link2, link1)) ;
    }

    bool isActive(const string &link) const { return exclusions_.count(link) == 0 ; }
    bool isActive(const string &link1, const string &link2) const { return expairs_.count(make_pair(link1, link2)) == 0 ; }

    set<std::pair<std::string, std::string>> expairs_ ;
    set<string> exclusions_ ;

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
        CollisionObjectPrivate *k0 = reinterpret_cast<CollisionObjectPrivate *>(co0->getUserPointer());
        CollisionObjectPrivate *k1 = reinterpret_cast<CollisionObjectPrivate *>(co1->getUserPointer());

        return true ;

    }

};

CollisionSpace::CollisionSpace() {
    broadphase_.reset(new btDbvtBroadphase());
    config_.reset(new btDefaultCollisionConfiguration())  ;
    dispatcher_.reset(new  btCollisionDispatcher(config_.get()));
    cb_.reset(new CollisionFilterCallback()) ;
    broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(cb_.get());
    world_.reset(new btCollisionWorld(dispatcher_.get(), broadphase_.get(), config_.get()));

}

CollisionSpace::~CollisionSpace() {

}

void CollisionSpace::addCollisionObject(const std::string &name, const CollisionShapePtr &shape, const Isometry3f &tr) {
    assert(!name.empty());

    btCollisionObject *collisionObj = new btCollisionObject() ;

    collisionObj->setCollisionShape(shape->handle()) ;

    btTransform c = toBulletTransform(tr)  ;
    collisionObj->setWorldTransform(c) ;

    std::shared_ptr<CollisionObjectPrivate> data(new CollisionObjectPrivate) ;
    data->obj_.reset(collisionObj) ;
    data->name_ = name ;
    data->shape_ = shape ;

    collisionObj->setUserPointer(reinterpret_cast<void*>(data.get()));

    world_->addCollisionObject(collisionObj) ;

    objects_.emplace(name, data) ;
}

void CollisionSpace::addRobot(const URDFRobot &robot, float margin, bool disable_self_collisions)
{

    map<string, Isometry3f> link_transforms ;
    robot.computeLinkTransforms(link_transforms) ;

    if ( disable_self_collisions ) {
        for( const auto &l1: robot.links_ ) {
            for( const auto &l2: robot.links_ ) {
                if ( l1.first != l2.first )
                    disableCollision(l1.first, l2.first);
            }
        }
    }

    vector<string> link_names ;
    for( const auto &bp: robot.links_ ) {
        const string &name = bp.first ;
        const URDFLink &link = bp.second ;

        link_names.push_back(name) ;

        vector<CollisionShapePtr> shapes ;
        vector<Isometry3f> origins ;

        for( const auto &geom: link.collision_geoms_ ) {

            Isometry3f col_origin = geom->origin_ ;

            origins.push_back(col_origin) ;

            CollisionShapePtr cs = makeCollisionShape(geom.get()) ;

            if ( cs ) {
                shapes.push_back(cs) ;
            }
        }

        CollisionShapePtr col_shape ;
        Isometry3f col_origin = Isometry3f::Identity() ;

        if ( shapes.empty() ) {
            continue ;
        }   else if ( shapes.size() == 1 ) {
            col_shape = shapes[0] ;
            col_origin = origins[0] ;

        } else {
            GroupCollisionShape *gc = new GroupCollisionShape() ;

            for( uint i=0 ; i<shapes.size() ; i++ ) {
                gc->addChild(shapes[i], origins[i]) ;
            }
            col_shape.reset(gc) ;
         }
        col_shape->setMargin(margin);

        auto it = link_transforms.find(link.name_) ;

     //   cout << link.name_ << ' ' << (it->second* col_origin).matrix() << endl ;

        addCollisionObject(link.name_, col_shape, it->second * col_origin);
    }
}

CollisionShapePtr CollisionSpace::makeCollisionShape(const URDFGeometry *geom) {
    float scale = 1.0 ;
    CollisionShapePtr shape ;

    if ( const URDFBoxGeometry *g = dynamic_cast<const URDFBoxGeometry *>(geom) ) {
        shape.reset(new BoxCollisionShape(g->he_))  ;
    } else if ( const URDFCylinderGeometry *g = dynamic_cast<const URDFCylinderGeometry *>(geom) ) {
        shape.reset(new CylinderCollisionShape(g->radius_, g->height_/2.0, CylinderCollisionShape::ZAxis))  ;
    } else if ( const URDFMeshGeometry *g = dynamic_cast<const URDFMeshGeometry *>(geom) ) {
        shape.reset(new ConvexHullCollisionShape(g->path_, g->scale_));
    } else if ( const URDFSphereGeometry *g = dynamic_cast<const URDFSphereGeometry *>(geom) ) {
        shape.reset(new SphereCollisionShape(g->radius_));
    }

    if ( shape ) shape->setLocalScale(scale) ;
    return shape ;
}

bool CollisionSpace::hasCollision() {
    broadphase_->calculateOverlappingPairs(dispatcher_.get());
    world_->performDiscreteCollisionDetection();

    int numManifolds = world_->getDispatcher()->getNumManifolds();

    for ( int i = 0; i < numManifolds; ++i ) {
        btPersistentManifold *contactManifold =
                world_->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject *obA = static_cast<const btCollisionObject *>(contactManifold->getBody0());
        const btCollisionObject *obB = static_cast<const btCollisionObject *>(contactManifold->getBody1());

        const CollisionObjectPrivate *coA = static_cast<const CollisionObjectPrivate *>(obA->getUserPointer());
        assert(coA != nullptr) ;

        const CollisionObjectPrivate *coB = static_cast<const CollisionObjectPrivate *>(obB->getUserPointer());
           assert(coB != nullptr) ;

        int numContacts = contactManifold->getNumContacts();

        if ( numContacts > 0 ) {
            cout << "collision: " << coA->name_ << ' ' << coB->name_ << ' ' << numContacts << endl ;
            return true ;
        }



    }
    return false ;

}

void CollisionSpace::disableCollision(const std::string &l1, const std::string &l2) {
    cb_->excludeLinkPair(l1, l2) ;
}

void CollisionSpace::enableCollision(const std::string &l1, const std::string &l2) {
    cb_->includeLinkPair(l1, l2) ;
}

void CollisionSpace::updateObjectTransform(const std::string &name, const Eigen::Isometry3f &tr) {
    auto it = objects_.find(name) ;
    if ( it == objects_.end() ) return ;
    std::shared_ptr<CollisionObjectPrivate> p = it->second ;
    p->obj_->setWorldTransform(toBulletTransform(tr));
    world_->updateSingleAabb(p->obj_.get());
}

void CollisionSpace::updateObjectTransforms(const map<string, Isometry3f> &trs) {
    for( const auto &tp: trs )
        updateObjectTransform(tp.first, tp.second) ;
}

}
