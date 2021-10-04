#ifndef XSIM_PHYSICS_SOFT_BODY_HPP
#define XSIM_PHYSICS_SOFT_BODY_HPP

#include <memory>


#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletSoftBody/btSoftBody.h>

#include <xsim/collision.hpp>
#include <xsim/convert.hpp>
#include <xsim/rigid_body.hpp>

#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node.hpp>

namespace xsim {

class PhysicsWorld ;
class SoftBodyShape ;

class SoftBodyBuilder {
public:
    SoftBodyBuilder() = default ;

    SoftBodyBuilder &setName(const std::string &name) {
        name_ = name ;
        return *this ;
    }

    SoftBodyBuilder &setMass(float mass) {
        total_mass_ = mass ;
        return *this ;
    }

    SoftBodyBuilder &setShape(const std::shared_ptr<SoftBodyShape> &shape) {
        shape_ = shape ;
        return *this ;
    }

    SoftBodyBuilder &setMargin(float margin) {
        margin_ = margin ;
        return *this ;
    }

    SoftBodyBuilder &setScale(float scale) {
        scale_ = scale ;
        return *this ;
    }

    SoftBodyBuilder &setWorldTransform(const Eigen::Isometry3f &tr) {
        tr_ = tr ;
        return *this ;
    }

    // this will create an empty triangular mesh which will be filled when the softbody is added to the world
    // by a call to getMesh function in SoftBody class.
    SoftBodyBuilder &makeVisualShape(const xviz::MaterialPtr &material) {
        xviz::NodePtr node(new xviz::Node) ;
        xviz::GeometryPtr mesh(new xviz::Geometry(xviz::Geometry::Triangles));
        node->addDrawable(mesh, material) ;
        visual_ = node ;
        return *this ;
    }

private:
    friend class SoftBody ;

    std::string name_ ;
    float total_mass_ = 0 ;
    float margin_ = 0, scale_ = 1 ;
    Eigen::Isometry3f tr_ = Eigen::Isometry3f::Identity();
    std::shared_ptr<SoftBodyShape> shape_ ;
    xviz::NodePtr visual_ ;
};

class SoftBody: public CollisionObject {
protected:
    SoftBody() = default  ;
public :

    ~SoftBody() = default ;

    btSoftBody *handle() const { return handle_.get(); }

    void getMesh(std::vector<Eigen::Vector3f> &vertices,
                                   std::vector<Eigen::Vector3f> &normals,
                                   std::vector<uint32_t> &indices) const ;

    std::string getName() const override {
        return name_ ;
    }

    Eigen::Isometry3f getWorldTransform() const override { return tr_ ; }

    void addAnchor(uint node_idx, RigidBodyPtr anchor) ;

protected:

    SoftBody(const SoftBodyBuilder &b, PhysicsWorld &world) ;

    void updateVisualGeometry() ;

    friend class PhysicsWorld ;

    std::string name_ ;
    std::string total_mass_ ;
    std::unique_ptr<btSoftBody> handle_ ;
    xviz::NodePtr visual_ ;
    Eigen::Isometry3f tr_ ;
};

using SoftBodyPtr = std::shared_ptr<SoftBody> ;

} // namespace xviz


#endif
