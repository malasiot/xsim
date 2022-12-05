#include <bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h>
#include <xsim/collision.hpp>

#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>

#include <iostream>

using namespace std ;
using namespace Eigen ;

namespace xsim {

void TriangleMeshCollisionShape::create(const std::string &fname)
{
    const aiScene *sc = aiImportFile(fname.c_str(),
                                       aiProcess_PreTransformVertices
                                     | aiProcess_Triangulate
                                     | aiProcess_JoinIdenticalVertices
                                     | aiProcess_SortByPType
                                     | aiProcess_OptimizeMeshes
                                     ) ;

    if ( !sc ) return ;

    import(sc) ;
}

void TriangleMeshCollisionShape::import(const aiScene *scene) {
    importMeshes(scene) ;
}

void TriangleMeshCollisionShape::create(const aiScene *scene) {
    import(scene) ;
}



void TriangleMeshCollisionShape::importMeshes(const aiScene *scene) {
    indexed_vertex_array_.reset(new btTriangleIndexVertexArray);

    for(int i=0 ; i<scene->mNumMeshes ; i++) {
        aiMesh *mesh = scene->mMeshes[i] ;
        if ( mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE ) continue ;

        MeshData cmesh ;

        for( int j=0 ; j<mesh->mNumFaces ; j++ ) {
            aiFace &face = mesh->mFaces[j] ;
            cmesh.tridx_.push_back(face.mIndices[0]) ;
            cmesh.tridx_.push_back(face.mIndices[1]) ;
            cmesh.tridx_.push_back(face.mIndices[2]) ;
        }

        for( int j=0 ; j<mesh->mNumVertices ; j++ ) {
            aiVector3D &v = mesh->mVertices[j] ;
            cmesh.vtx_.emplace_back(v.x, v.y, v.z) ;
        }

        btIndexedMesh bulletMesh;

        bulletMesh.m_numTriangles = mesh->mNumFaces;

        bulletMesh.m_triangleIndexBase =
                reinterpret_cast<const unsigned char*>(cmesh.tridx_.data());
        bulletMesh.m_triangleIndexStride = 3 * sizeof(uint);
        bulletMesh.m_numVertices = mesh->mNumVertices ;
        bulletMesh.m_vertexBase =
                reinterpret_cast<const unsigned char*>(cmesh.vtx_.data());
        bulletMesh.m_vertexStride = sizeof(Eigen::Vector3f);
        bulletMesh.m_indexType = PHY_INTEGER;
        bulletMesh.m_vertexType = PHY_FLOAT;

        indexed_vertex_array_->addIndexedMesh(bulletMesh, PHY_INTEGER);  // exact shape
        meshes_.emplace_back(std::move(cmesh)) ;
    }

    handle_.reset(makeShape(indexed_vertex_array_.get())) ;
}


ConvexHullCollisionShape::ConvexHullCollisionShape(const string &fname, const Vector3f &scale) {
    const aiScene *sc = aiImportFile(fname.c_str(),
                                       aiProcess_PreTransformVertices
                                     | aiProcess_Triangulate
                                     | aiProcess_JoinIdenticalVertices
                                     | aiProcess_SortByPType
                                     | aiProcess_OptimizeMeshes
                                     ) ;

    if ( !sc ) {
        cerr << "Failed to load collision shape from:"  << fname << '\n' ;
        cerr << aiGetErrorString() << endl ;
        return ;
    }

    create(sc, scale) ;
}


void ConvexHullCollisionShape::create(const aiScene *scene, const Vector3f &scale) {

    uint nv = 0 ;
    for(int i=0 ; i<scene->mNumMeshes ; i++) {
        aiMesh *mesh = scene->mMeshes[i] ;
        nv += mesh->mNumVertices ;
    }

   std::unique_ptr<btVector3 []> vertices(new btVector3 [nv]) ;

   uint k=0 ;
   for(int i=0 ; i<scene->mNumMeshes ; i++) {
       aiMesh *mesh = scene->mMeshes[i] ;

       for( int j=0 ; j<mesh->mNumVertices ; j++ ) {
           aiVector3D &v = mesh->mVertices[j] ;

           btVector3 bv(v.x * scale.x(), v.y * scale.y(), v.z * scale.z());
           vertices[k] = bv ;
           k++ ;
       }
   }

   btConvexHullShape* convexHull = new btConvexHullShape(reinterpret_cast<btScalar *>(&vertices[0]), nv, sizeof(btVector3));
   convexHull->optimizeConvexHull();
   convexHull->recalcLocalAabb();

   handle_.reset(convexHull) ;
}

btCollisionShape *StaticMeshCollisionShape::makeShape(btTriangleIndexVertexArray *va) {
    return new btBvhTriangleMeshShape(va ,true);
}

// does not work
btCollisionShape *DynamicMeshCollisionShape::makeShape(btTriangleIndexVertexArray *va) {
    btGImpactMeshShape *shape = new btGImpactMeshShape(va) ;
    shape->updateBound();
    return shape ;
}

GroupCollisionShape::GroupCollisionShape() {
    handle_.reset(new btCompoundShape()) ;
}

void GroupCollisionShape::addChild(CollisionShapePtr c, const Affine3f &tr)
{
    children_.emplace_back(c) ;
    static_cast<btCompoundShape *>(handle_.get())->addChildShape(toBulletTransform(tr), c->handle());
}

GhostObject::GhostObject(CollisionShapePtr shape): shape_(shape) {
    ghost_.reset(new btGhostObject()) ;
    ghost_->setCollisionShape(shape_->handle()) ;
    ghost_->setCollisionFlags(ghost_->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    ghost_->setUserPointer(this) ;
}

void GhostObject::setWorldTransform(const Isometry3f &tr) {
    ghost_->setWorldTransform(toBulletTransform(tr)) ;
}

Isometry3f GhostObject::getWorldTransform() const {
    return toEigenTransform(ghost_->getWorldTransform());
}

std::vector<CollisionObject *> GhostObject::getOverlapingObjects() const
{
    vector<CollisionObject *> collisions ;

    for( int i = 0; i < ghost_->getNumOverlappingObjects(); i++ ) {
        btCollisionObject *ob = ghost_->getOverlappingObject(i);
        CollisionObject *co = reinterpret_cast<CollisionObject *>(ob->getUserPointer()) ;
        if ( co )
            collisions.push_back(co) ;
    }

    return collisions ;
}

bool GhostObject::isOverlapping() const {
    return ghost_->getNumOverlappingObjects() != 0 ;
}

}
