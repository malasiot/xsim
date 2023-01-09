#include <xsim/rigid_body.hpp>
#include <xviz/scene/node_helpers.hpp>


using namespace xviz ;

namespace xsim {


// MotionState for dynamic objects that updates the transform of the associated node in the scene
class MotionState: public btMotionState {
    public:


    MotionState (RigidBody *body, const btTransform &tr): body_(body), tr_(tr) {
    }

    virtual void getWorldTransform( btTransform& centerOfMassWorldTrans ) const override {
        centerOfMassWorldTrans = tr_ ;
    }
    virtual void setWorldTransform( const btTransform& centerOfMassWorldTrans ) override {
        tr_ = centerOfMassWorldTrans ;
        if ( body_->visual_shape_ ) {
            body_->visual_shape_->setTransform(toEigenTransform(tr_)) ;
        }
    }

private:

    btTransform tr_ ;
    RigidBody *body_ ;
};

btRigidBody *RigidBody::handle() const { return handle_.get() ; }

void RigidBody::setName(const std::string &name) { name_ = name ; }

Eigen::Isometry3f RigidBody::getWorldTransform() const {
    if ( motion_state_) {
        btTransform world ;
        motion_state_->getWorldTransform(world) ;
        return toEigenTransform(world) ;
    } else {
        return toEigenTransform(handle_->getWorldTransform()) ;
    }

}

void RigidBody::setWorldTransform(const Eigen::Isometry3f &tr) {
    handle_->setWorldTransform(toBulletTransform(tr));
}

void RigidBody::disableDeactivation() {
    handle_->forceActivationState(DISABLE_DEACTIVATION);
}

void RigidBody::setActive() {
    handle_->forceActivationState(ACTIVE_TAG);
}

void RigidBody::setContactProcessingThreshold(float t) {
    handle_->setContactProcessingThreshold(t);
}

void RigidBody::setCCDSweptSphereRadius(float r) {
    handle_->setCcdSweptSphereRadius(r);
    handle_->setCcdMotionThreshold(r / 2.);
}

void RigidBody::applyExternalForce(const Eigen::Vector3f &force, const Eigen::Vector3f &pos) {
    handle_->applyForce(toBulletVector(force), toBulletVector(pos));
}

void RigidBody::applyExternalTorque(const Eigen::Vector3f &torque) {
    handle_->applyTorque(toBulletVector(torque));
}

void RigidBody::applyCentralImpulse(const Eigen::Vector3f &imp)
{
    handle_->applyCentralImpulse(toBulletVector(imp));
}

RigidBody::RigidBody(const RigidBodyBuilder &rb)
{
    assert((!rb.collision_shape_->handle() || rb.collision_shape_->handle()->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    collision_shape_ = rb.collision_shape_ ;
    visual_shape_ = rb.visual_shape_ ;

    motion_state_.reset(new MotionState(this, toBulletTransform(rb.world_transform_))) ;

    btVector3 inertia(0, 0, 0) ;
    if ( rb.local_inertia_  )
        inertia = eigenVectorToBullet(rb.local_inertia_.value()) ;
    else if ( rb.mass_ != 0.0f ) {
        collision_shape_->handle()->calculateLocalInertia(rb.mass_, inertia);
    }

    btRigidBody::btRigidBodyConstructionInfo cInfo(rb.mass_, motion_state_.get(), collision_shape_->handle(), inertia);

    if ( rb.friction_ )         cInfo.m_friction = rb.friction_.value() ;
    if ( rb.angular_damping_ )  cInfo.m_angularDamping = rb.angular_damping_.value() ;
    if ( rb.linear_damping_ )   cInfo.m_linearDamping = rb.linear_damping_.value() ;
    if ( rb.rolling_friction_ )   cInfo.m_rollingFriction = rb.rolling_friction_.value() ;
    if ( rb.spinning_friction_ )  cInfo.m_spinningFriction = rb.spinning_friction_.value() ;
    if ( rb.restitution_ )        cInfo.m_restitution = rb.restitution_.value() ;

    btRigidBody* body = new btRigidBody(cInfo);

    body->activate(true) ;

    if ( rb.mass_ == 0.0f ) { // static object
        body->setWorldTransform(toBulletTransform(rb.world_transform_));
        if ( visual_shape_ )
            visual_shape_->setTransform(rb.world_transform_);
    }

    handle_.reset(body);

    name_ = rb.name_ ;

}

NodePtr makeVisualShapeFromCollisionShape(const CollisionShapePtr &collision_shape, const MaterialPtr &mat)
{
    assert(collision_shape) ;

    if ( const BoxCollisionShape *box = dynamic_cast<BoxCollisionShape *>(collision_shape.get()) ) {
        NodePtr node(new Node) ;
        GeometryPtr geom(new Geometry(Geometry::createSolidCube(box->hs())));
        node->addDrawable(geom, mat) ;
        return node ;
    } else if ( const CylinderCollisionShape *cylinder = dynamic_cast<CylinderCollisionShape *>(collision_shape.get()) ) {
        NodePtr node(new Node) ;
        GeometryPtr geom(new Geometry(Geometry::createSolidCylinder(cylinder->getRadius(), cylinder->getLength(), 12, 10)));
        node->addDrawable(geom, mat) ;

        if ( cylinder->getAxis() == CylinderCollisionShape::YAxis ) {
            node->transform().rotate(Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitX()));
            NodePtr externalNode(new Node) ;
            externalNode->addChild(node) ;
            return externalNode ;
        } else if ( cylinder->getAxis() == CylinderCollisionShape::XAxis ) {
            node->transform().rotate(Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitY()));
            NodePtr externalNode(new Node) ;
            externalNode->addChild(node) ;
            return externalNode ;
        } else return node ;
    } else if ( const SphereCollisionShape *sphere = dynamic_cast<SphereCollisionShape *>(collision_shape.get()) ) {
        NodePtr node(new Node) ;
        GeometryPtr geom(new Geometry(Geometry::createSolidSphere(sphere->getRadius(), 12, 10)));
        node->addDrawable(geom, mat) ;
        return node ;
    }

    return nullptr ;

}


} // namespace viz

