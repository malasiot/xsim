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
    CollisionShapePtr shape_ ;
    std::unique_ptr<btCollisionObject> obj_ ;
    Isometry3f wtr_ ;
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
   //        cout << k0->name_ << ' ' << k1->name_ << endl ;
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

void saveSolidCube(ofstream &strm, const btVector3 &vmin, const btVector3 &vmax, int &offset) {

    std::vector<btVector3> normals{{ 0.0, 0.0, 1.0 }, {0.0, 0.0, -1.0}, { 0.0, 1.0, 0.0 }, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, { -1.0, 0.0, 0.0}} ;
    std::vector<btVector3> vertices = {{ vmin.getX(), vmax.y(), vmax.z() }, { vmax.x(), vmax.y(), vmax.z() }, { vmax.x(), vmin.y(), vmax.z() },
                                      { vmin.x(), vmin.y(), vmax.z() },
                                      { vmin.x(), vmax.y(), vmin.z() }, { vmax.x(), vmax.y(), vmin.z() }, { vmax.x(), vmin.y(), vmin.z() },
                                      { vmin.x(), vmin.y(), vmin.z() } } ;

    std::vector<uint32_t> vtx_indices {  1, 0, 3,  7, 4, 5,  4, 0, 1,  5, 1, 2,  2, 3, 7,  0, 4, 7,  1, 3, 2,  7, 5, 6,  4, 1, 5,  5, 2, 6,  2, 7, 6, 0, 7, 3};
    std::vector<uint32_t> nrm_indices {  0, 0, 0,  1, 1, 1,  2, 2, 2,  3, 3, 3,  4, 4, 4,  5, 5, 5,  0, 0, 0,  1, 1, 1,  2, 2, 2,  3, 3, 3,  4, 4, 4, 5, 5, 5};

    for( const auto &v: vertices ) {
        strm << "v " << v.x() << " " << v.y() << " " << v.z() << endl ;
    }

    for( const auto &n: normals ) {
        strm << "vn " << n.x() << ' ' << n.y() << ' ' << n.z() << endl ;
    }

    for( int i=0 ; i< vtx_indices.size() ; i++ ) {
        strm << "f " << vtx_indices[i]+offset+1 << "//" << nrm_indices[i]+offset+1 << ' ' ; ++i ;
        strm <<  vtx_indices[i]+offset+1 << "//" << nrm_indices[i]+offset+1 << ' ' ; ++i ;
        strm <<  vtx_indices[i]+offset+1 << "//" << nrm_indices[i]+offset+1 << endl ;
    }

    offset += 8 ;

}

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

void getAABB(const btCollisionObject *obj, btVector3& aabb_min, btVector3& aabb_max)
  {
    obj->getCollisionShape()->getAabb(obj->getWorldTransform(), aabb_min, aabb_max);
    const btScalar& distance = obj->getContactProcessingThreshold();
    // note that bullet expands each AABB by 4 cm
  //  btVector3 contact_threshold(distance, distance, distance);
 //   aabb_min -= contact_threshold;
 //   aabb_max += contact_threshold;
  }

void updateBroadphaseAABB(const btCollisionObject * cow,
                                 const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                 const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btVector3 aabb_min, aabb_max;
  getAABB(cow, aabb_min, aabb_max);

  assert(cow->getBroadphaseHandle() != nullptr);
  broadphase->setAabb((btBroadphaseProxy *)cow->getBroadphaseHandle(), aabb_min, aabb_max, dispatcher.get());
}

void addCollisionObjectToBroadphase(btCollisionObject *obj, int filter_group, int filter_mask,
                                    const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                    std::unique_ptr<btCollisionDispatcher>& dispatcher) {
    btVector3 aabb_min, aabb_max;
    getAABB(obj, aabb_min, aabb_max);

    int type = obj->getCollisionShape()->getShapeType();
    obj->setBroadphaseHandle(broadphase->createProxy(aabb_min, aabb_max, type, (void *)obj, filter_group, filter_mask,
                                                       dispatcher.get()));
}
 ofstream strm("/tmp/aabb.obj") ;
 int offset = 0 ;
void CollisionSpace::addCollisionObject(const std::string &name, const CollisionShapePtr &shape, const Isometry3f &tr) {
    btCollisionObject *collisionObj = new btCollisionObject() ;

    collisionObj->setCollisionShape(shape->handle()) ;

    btTransform c = toBulletTransform(tr)  ;
    collisionObj->setWorldTransform(c) ;

    std::shared_ptr<CollisionObjectPrivate> data(new CollisionObjectPrivate) ;
    data->wtr_ = tr ;
    data->obj_.reset(collisionObj) ;
    data->shape_ = shape ;
    data->name_ = name ;

    collisionObj->setUserPointer(reinterpret_cast<void*>(data.get()));

    world_->addCollisionObject(collisionObj, int(btBroadphaseProxy::StaticFilter), int(btBroadphaseProxy::KinematicFilter)) ;

    btVector3 aabb_min, aabb_max ;
    getAABB(collisionObj, aabb_min, aabb_max) ;


        saveSolidCube(strm, aabb_min, aabb_max, offset) ;
    objects_.emplace_back(data) ;


}

void CollisionSpace::addRobot(const xviz::URDFRobot &robot, bool self_collisions)
{

    uint count = 0 ;

    map<string, Isometry3f> link_transforms ;
    robot.computeLinkTransforms(link_transforms) ;

    for( const auto &bp: robot.links_ ) {
        const string &name = bp.first ;
        const URDFLink &link = bp.second ;

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
            col_shape->setMargin(0.00);
        }
  col_shape->setMargin(0.04);
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

struct OverlapCallback: public btOverlapCallback
{
    virtual ~OverlapCallback()
    {
    }
    //return true for deletion of the pair
    bool processOverlap(btBroadphasePair& pair) {
        auto proxy0 = pair.m_pProxy0 ;
        auto proxy1 = pair.m_pProxy1 ;
        assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
        assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);

        CollisionObjectPrivate *k0 = reinterpret_cast<CollisionObjectPrivate *>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
        CollisionObjectPrivate *k1 = reinterpret_cast<CollisionObjectPrivate *>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());

        cout << k0->name_ << ' '  << k1->name_ << endl ;
        return true;
    }

};


bool CollisionSpace::hasCollision() {


    world_->performDiscreteCollisionDetection();


    int numManifolds = world_->getDispatcher()->getNumManifolds();

 //   if ( numManifolds > 0 ) return true ;

    for ( int i = 0; i < numManifolds; ++i ) {
        btPersistentManifold *contactManifold =
                world_->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject *obA = static_cast<const btCollisionObject *>(contactManifold->getBody0());
        const btCollisionObject *obB = static_cast<const btCollisionObject *>(contactManifold->getBody1());

        const CollisionObjectPrivate *coA = static_cast<const CollisionObjectPrivate *>(obA->getUserPointer());
        assert(coA != nullptr) ;

        const CollisionObjectPrivate *coB = static_cast<const CollisionObjectPrivate *>(obB->getUserPointer());
        //   assert(coB != nullptr) ;

        cout << "collision: " << coA->name_ << ' ' << coB->name_ << endl ;

        int numContacts = contactManifold->getNumContacts();

        cout << numContacts << endl ;

    }
    return false ;

}

void CollisionSpace::disableCollision(const std::string &l1, const std::string &l2) {
    cb_->excludeLinkPair(l1, l2) ;
}

}
