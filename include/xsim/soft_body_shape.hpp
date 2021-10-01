#ifndef XSIM_PHYSICS_SOFT_BODY_SHAPE_HPP
#define XSIM_PHYSICS_SOFT_BODY_SHAPE_HPP

#include <Eigen/Geometry>
#include <bullet/btBulletCollisionCommon.h>
#include <memory>

namespace xsim {

class SoftBodyShape {
public:
    static std::shared_ptr<SoftBodyShape> fromMeshModel(const std::string &file_name) ;
protected:

    struct Link {
        Link(uint32_t n1, uint32_t n2): n1_(n1), n2_(n2) {}
        uint32_t n1_, n2_ ;
    };

    struct Face {
        Face(uint32_t n1, uint32_t n2, uint32_t n3): n1_(n1), n2_(n2), n3_(n3) {}
        uint32_t n1_, n2_, n3_ ;
    };


    friend class SoftBody ;

    std::vector<Link> links_ ;
    std::vector<Face> faces_ ;
    std::vector<btVector3> nodes_ ;
    std::vector<btScalar> node_mass_ ;
};

using SoftBodyShapePtr = std::shared_ptr<SoftBodyShape> ;


class SoftPatch2D: public SoftBodyShape {
public:

    enum Anchors {  TopLeft = 0x1, TopRight = 0x2, BottomLeft = 0x4, BottomRight = 0x8,
                    TopEdge = 0x10, BottomEdge = 0x20, LeftEdge = 0x40, RightEdge = 0x80 } ;

    SoftPatch2D(const float size, uint numX, uint numY, uint anchors) ;
    SoftPatch2D(const Eigen::Vector3f &c00, const Eigen::Vector3f &c10, const Eigen::Vector3f &c01,
                uint nvX, uint nvY, uint anchors, bool gendiags);
};

}

#endif
