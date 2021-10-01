#include <xsim/soft_body.hpp>
#include <xsim/world.hpp>
#include <xsim/convert.hpp>
#include <xsim/soft_body_shape.hpp>
#include <xviz/scene/mesh.hpp>
#include <xviz/scene/node.hpp>

using namespace Eigen ;
using namespace std ;
using namespace xviz ;

namespace xsim {

#if 0
SoftPatch2D::SoftPatch2D(const Vector3f &c00, const Vector3f &c10, const Vector3f &c01,
                         uint nvX, uint nvY, uint flags, bool gendiags) {
#define IDX(_x_, _y_) ((_y_)*nvX + (_x_))
    /* Create nodes	*/
    assert( nvX >=2 && nvY >=2) ;
    uint numNodes = nvX * nvY ;

    //        unique_ptr<btVector3 []> x(new btVector3[numNodes]) ;
    //        unique_ptr<btScalar []> m(new btScalar[numNodes]) ;

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

    //        handle_.reset(new btSoftBody(&physics.getSoftBodyWorldInfo(), numNodes, x.get(), m.get())) ;

    if ( flags & TopLeft ) node_mass_[IDX(0, 0)] = 0;
    if ( flags & TopRight ) node_mass_[IDX(nvX - 1, 0)] = 0;
    if ( flags & BottomLeft ) node_mass_[IDX(0, nvY - 1)] = 0;
    if ( flags & BottomRight ) node_mass_[IDX(nvX - 1, nvY - 1)] = 0;

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
                    {
                        links_.emplace_back(IDX(ix, iy), IDX(ix + 1, iy + 1));
                    }
                }
                else
                {
                    faces_.emplace_back(IDX(ix, iy + 1), IDX(ix, iy), IDX(ix + 1, iy));
                    faces_.emplace_back(IDX(ix, iy + 1), IDX(ix + 1, iy), IDX(ix + 1, iy + 1));

                    if (gendiags)
                    {
                        links_.emplace_back(IDX(ix + 1, iy), IDX(ix, iy + 1));
                    }
                }
            }
        }
    }

}
#endif
void SoftBody::getMesh(std::vector<Eigen::Vector3f> &vertices,
                       std::vector<Eigen::Vector3f> &normals,
                       std::vector<uint32_t> &indices) const {

    std::map<btSoftBody::Node *, uint> idxs ;

    for( uint i=0 ; i<handle_->m_nodes.size() ; i++ ) {
        auto &node = handle_->m_nodes[i] ;
        Vector3f v = toEigenVector(node.m_x) ;
        vertices.push_back(v) ;
        normals.push_back(toEigenVector(node.m_n));
        idxs[&node] = i ;
    }

    if ( indices.empty() ) {
        for( uint i=0 ; i< handle_->m_faces.size() ; i++ ) {
            const auto &face = handle_->m_faces[i] ;
            uint idx0 = idxs[face.m_n[0]] ;
            uint idx1 = idxs[face.m_n[2]] ;
            uint idx2 = idxs[face.m_n[1]] ;
            indices.push_back(idx0) ;
            indices.push_back(idx1) ;
            indices.push_back(idx2) ;
        }
    }
}

SoftBody::SoftBody(const SoftBodyBuilder &b, PhysicsWorld &world)
{
    assert( b.shape_ ) ;

    visual_ = b.visual_ ;

    const btScalar* m = ( b.shape_->node_mass_.empty() ) ? nullptr : &b.shape_->node_mass_[0] ;
    handle_.reset(new btSoftBody(&world.getSoftBodyWorldInfo(),
                                 b.shape_->nodes_.size(), &b.shape_->nodes_[0], m)) ;

    for( const auto &l: b.shape_->links_ ) {
        handle_->appendLink(l.n1_, l.n2_) ;
    }

    for( const auto &f: b.shape_->faces_ ) {
        handle_->appendFace(f.n1_, f.n2_, f.n3_) ;
    }

    name_ = b.name_ ;

    handle_->scale(btVector3(b.scale_, b.scale_, b.scale_));
    handle_->transform(toBulletTransform(b.tr_)) ;


    handle_->getCollisionShape()->setMargin(b.margin_);
    handle_->getCollisionShape()->setUserPointer((void*)this);
    handle_->generateBendingConstraints(2, handle_->appendMaterial());
    handle_->setTotalMass(b.total_mass_ );
    //  cloth->m_cfg.citerations = 100;
    //  cloth->m_cfg.diterations = 100;
    handle_->m_cfg.piterations = 5;
    handle_->m_cfg.kDP = 0.005f;
    handle_->m_cfg.collisions |= btSoftBody::fCollision::VF_SS | btSoftBody::fCollision::SDF_RS;
}

void SoftBody::updateVisualGeometry()
{
    if ( visual_ ) {
        auto &dr = visual_->drawables()[0] ;

        xviz::GeometryPtr geom = dr.geometry() ;
        geom->vertices().clear() ;
        geom->normals().clear() ;
        getMesh(geom->vertices(), geom->normals(), geom->indices());
        geom->setVerticesUpdated(true);
        geom->setNormalsUpdated(true);
    }

}

}
