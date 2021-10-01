#include <xsim/soft_body_shape.hpp>
#include <xsim/convert.hpp>

#include <unordered_set>

#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

using namespace Eigen ;
using namespace std ;


namespace xsim {

SoftPatch2D::SoftPatch2D(const Vector3f &c00, const Vector3f &c10, const Vector3f &c01,
                         uint nvX, uint nvY, uint flags, bool gendiags) {
#define IDX(_x_, _y_) ((_y_)*nvX + (_x_))
    /* Create nodes	*/
    assert( nvX >=2 && nvY >=2) ;
    uint numNodes = nvX * nvY ;

    nodes_.resize(numNodes) ;
    node_mass_.resize(numNodes) ;

    btScalar stepX = (c00 - c10).norm() / (btScalar)(nvX - 1) ;
    btScalar stepY = (c00 - c01).norm() / (btScalar)(nvY - 1) ;

    Vector3f deltaX = (c10 - c00).normalized() * stepX ;
    Vector3f deltaY = (c01 - c00).normalized() * stepY ;

    Vector3f py = c00 ;
    for ( uint iy = 0; iy < nvY; ++iy, py += deltaY ) {
        Vector3f px = py ;
        for ( uint ix = 0; ix < nvX; ++ix, px += deltaX ) {
            nodes_[IDX(ix, iy)] = toBulletVector(px) ;
            node_mass_[IDX(ix, iy)] = 1;
        }
    }

    if (flags & TopLeft ) node_mass_[IDX(0, 0)] = 0 ;
    if (flags & TopRight ) node_mass_[IDX(nvX - 1, 0)] = 0;
    if (flags & BottomLeft) node_mass_[IDX(0, nvY - 1)] = 0;
    if (flags & BottomRight ) node_mass_[IDX(nvX - 1, nvY - 1)] = 0;

    if ( flags & TopEdge ) {
        for( uint x=0 ; x<nvX ; x++ )
            node_mass_[IDX(x, 0)] = 0 ;
    }

    if ( flags & BottomEdge ) {
        for( uint x=0 ; x<nvX ; x++ )
            node_mass_[IDX(x, nvY-1)] = 0 ;
    }

    if ( flags & LeftEdge ) {
        for( uint y=0 ; y<nvY ; y++ )
            node_mass_[IDX(0, y)] = 0 ;
    }

    if ( flags & RightEdge ) {
        for( uint y=0 ; y<nvY ; y++ )
            node_mass_[IDX(nvX-1, y)] = 0 ;
    }

    /* Create links	and faces */
    for ( uint iy = 0; iy < nvY; ++iy) {
        for (uint ix = 0; ix < nvX; ++ix) {
            const uint idx = IDX(ix, iy);
            const bool mdx = (ix + 1) < nvX;
            const bool mdy = (iy + 1) < nvY;
            if (mdx) links_.emplace_back(idx, IDX(ix + 1, iy));
            if (mdy) links_.emplace_back(idx, IDX(ix, iy + 1));

            if (mdx && mdy) {
                if ((ix + iy) & 1)
                {
                    faces_.emplace_back(IDX(ix, iy), IDX(ix + 1, iy), IDX(ix + 1, iy + 1));
                    faces_.emplace_back(IDX(ix, iy), IDX(ix + 1, iy + 1), IDX(ix, iy + 1));

                    if (gendiags)
                        links_.emplace_back(IDX(ix, iy), IDX(ix + 1, iy + 1));

                }
                else
                {
                    faces_.emplace_back(IDX(ix, iy + 1), IDX(ix, iy), IDX(ix + 1, iy));
                    faces_.emplace_back(IDX(ix, iy + 1), IDX(ix + 1, iy), IDX(ix + 1, iy + 1));
                    if (gendiags)
                        links_.emplace_back(IDX(ix + 1, iy), IDX(ix, iy + 1));

                }
            }
        }
    }


}

struct idx_pair_hash
{
    std::size_t operator () (std::pair<uint, uint> const &pair) const
    {
        std::size_t h1 = std::hash<uint>()(pair.first);
        std::size_t h2 = std::hash<uint>()(pair.second);

        return h1 ^ h2;
    }
};

std::shared_ptr<SoftBodyShape> SoftBodyShape::fromMeshModel(const std::string &fname)
{
    const aiScene *sc = aiImportFile(fname.c_str(),
                                     aiProcess_PreTransformVertices
                                     | aiProcess_Triangulate
                                     | aiProcess_JoinIdenticalVertices
                                     | aiProcess_SortByPType
                                     | aiProcess_OptimizeMeshes
                                     ) ;

    if ( !sc ) return nullptr ;

    SoftBodyShape *shape = new SoftBodyShape ;

    assert(sc->mNumMeshes == 1) ;

    aiMesh *mesh = sc->mMeshes[0] ;
    if ( mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE ) return nullptr ;

    unordered_set<pair<uint, uint>, idx_pair_hash> link_set(100) ;

    for( int j=0 ; j<mesh->mNumFaces ; j++ ) {
        aiFace &face = mesh->mFaces[j] ;
        uint idx0 = face.mIndices[0] ;
        uint idx1 = face.mIndices[1] ;
        uint idx2 = face.mIndices[2] ;

        if ( link_set.count(make_pair(idx0, idx1)) == 0 ) {
            link_set.insert(make_pair(idx0, idx1)) ;
            shape->links_.emplace_back(idx0, idx1) ;
        }

        if ( link_set.count(make_pair(idx1, idx2)) == 0 ) {
            link_set.insert(make_pair(idx1, idx2)) ;
            shape->links_.emplace_back(idx1, idx2) ;
        }

        if ( link_set.count(make_pair(idx2, idx0)) == 0 ) {
            link_set.insert(make_pair(idx2, idx0)) ;
            shape->links_.emplace_back(idx2, idx0) ;
        }

        shape->faces_.emplace_back(idx0, idx1, idx2) ;

    }


    for( int j=0 ; j<mesh->mNumVertices ; j++ ) {
        aiVector3D &v = mesh->mVertices[j] ;
        btVector3 bvtx(v.x, v.y, v.z) ;
        shape->nodes_.push_back(bvtx) ;
    }

    return SoftBodyShapePtr(shape) ;
}


}
