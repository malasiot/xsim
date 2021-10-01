#ifndef XSIM_PHYSICS_COLLISION_HPP
#define XSIM_PHYSICS_COLLISION_HPP

#include <memory>
#include <map>

#include <bullet/btBulletCollisionCommon.h>
#include <Eigen/Geometry>
#include <xsim/convert.hpp>
#include <xviz/scene/scene_fwd.hpp>

#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>
#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>


namespace xsim {


class CollisionShape {

public:

    virtual ~CollisionShape() {}

    btCollisionShape *handle() const { return handle_.get() ; }

    Eigen::Vector3f computeLocalInertia(btScalar mass) {
        btVector3 inertia{0, 0, 0} ;
        handle_->calculateLocalInertia(mass, inertia) ;
        return toEigenVector(inertia) ;
    }

    void setLocalScale(float scale) {
        handle_->setLocalScaling(btVector3(scale, scale, scale)) ;
    }

    void setMargin(float v) {
        handle_->setMargin(v) ;
    }

protected:

    friend class RigidBody ;

    std::unique_ptr<btCollisionShape> handle_ ;
};

using CollisionShapePtr = std::shared_ptr<CollisionShape> ;

class BoxCollisionShape: public CollisionShape {
public:
    BoxCollisionShape(const Eigen::Vector3f &hs): hs_(hs) {
        handle_.reset(new btBoxShape(toBulletVector(hs)));
    }

    const Eigen::Vector3f &hs() const { return hs_ ; }

private:
    Eigen::Vector3f hs_ ;
};

class CylinderCollisionShape: public CollisionShape {
public:
    enum Axis { XAxis, YAxis, ZAxis } ;
    CylinderCollisionShape(float radius, float len, Axis a = YAxis ): axis_(a), radius_(radius), len_(len) {
        switch ( a ) {
        case XAxis:
            handle_.reset(new btCylinderShapeX(btVector3(len/2.0, radius, radius)));
            break ;
        case ZAxis:
            handle_.reset(new btCylinderShapeZ(btVector3(radius, radius, len/2.0)));
            break ;
        default:
            handle_.reset(new btCylinderShape(btVector3(radius, len/2.0, radius)));
        }
    }

    Axis getAxis() const { return axis_ ; }
    float getRadius() const { return radius_ ; }
    float getLength() const { return len_ ; }

private:

    Axis axis_ ;
    float radius_, len_ ;
};


class SphereCollisionShape: public CollisionShape {
public:
    SphereCollisionShape(float radius): radius_(radius) {
        handle_.reset(new btSphereShape(radius));
    }

    float getRadius() const { return radius_ ; }

private:
    float radius_ ;
};

class TriangleMeshCollisionShape: public CollisionShape {
protected:

    void importMeshes(const aiScene *scene) ;

    struct MeshData {
        std::vector<uint> tridx_ ;
        std::vector<Eigen::Vector3f> vtx_ ;
    } ;

    virtual void create(const aiScene *scene) ;
    virtual void create(const std::string &fname) ;

    virtual void import(const aiScene *scene) ;

    std::vector<MeshData> meshes_ ;
    std::unique_ptr<btTriangleIndexVertexArray> indexed_vertex_array_ ;

    virtual btCollisionShape *makeShape(btTriangleIndexVertexArray *va)  = 0 ;
};

class ConvexHullCollisionShape: public CollisionShape {

public:
    ConvexHullCollisionShape(const std::string &path, const Eigen::Vector3f &scale = {1, 1, 1}) ;

protected:

    void create(const aiScene *scene, const Eigen::Vector3f &scale);
};

class StaticMeshCollisionShape: public TriangleMeshCollisionShape {
public:
    StaticMeshCollisionShape(const std::string &mesh) { create(mesh) ; }

    // use aiProcess_PreTransformVertices and  aiProcess_Triangulate to prepare the geometries
    StaticMeshCollisionShape(const aiScene *scene) { create(scene) ; }

private:

    virtual btCollisionShape *makeShape(btTriangleIndexVertexArray *va) override ;

};

class DynamicMeshCollisionShape: public TriangleMeshCollisionShape {
public:
    DynamicMeshCollisionShape(const std::string &mesh) { create(mesh) ; }
    DynamicMeshCollisionShape(const aiScene *scene) { create(scene) ; }

private:

    virtual btCollisionShape *makeShape(btTriangleIndexVertexArray *va) override ;

};

class GroupCollisionShape: public CollisionShape {
public:
    GroupCollisionShape() ;

    void addChild(CollisionShapePtr c, const Eigen::Affine3f &tr) ;

private:

    std::vector<CollisionShapePtr> children_ ;
};


xviz::NodePtr makeVisualShapeFromCollisionShape(const CollisionShapePtr &collision_shape, const xviz::MaterialPtr &mat);

class CollisionObject {

public:
    virtual std::string getName() const = 0 ;
    virtual ~CollisionObject() {}
    virtual Eigen::Isometry3f getWorldTransform() const = 0 ;
};

struct ContactResult {
    const CollisionObject *a_, *b_ ;
    Eigen::Vector3f pa_, pb_, normal_ ;
} ;

class GhostObject: public CollisionObject {
public:
    GhostObject(CollisionShapePtr shape) ;

    void setName(const std::string name) { name_ = name ; }
    void setWorldTransform(const Eigen::Isometry3f &tr) ;

    virtual std::string getName() const override { return name_ ; }
    virtual Eigen::Isometry3f getWorldTransform() const override ;

    bool isOverlapping() const ;
    std::vector<CollisionObject *> getOverlapingObjects() const ;

    btGhostObject *handle() const { return ghost_.get() ; }

private:
    std::string name_ ;
    CollisionShapePtr shape_ ;
    std::unique_ptr<btGhostObject> ghost_ ;
};

using GhostObjectPtr = std::shared_ptr<GhostObject> ;


class CollisionFeedback {
public:
     virtual void processContact(ContactResult &r) = 0 ;
};

class CollisionFilter {
    public:
    // override to filter out objects that should not be included in collision results
    virtual bool collide(CollisionObject *obj1, CollisionObject *obj2) = 0 ;
};


} // namespace xviz

#endif
