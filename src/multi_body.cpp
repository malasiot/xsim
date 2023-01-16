#include <xsim/multi_body.hpp>
#include <xsim/world.hpp>
#include <xviz/scene/node.hpp>

#include <bullet/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <bullet/BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>


using namespace std ;
using namespace Eigen ;

namespace xsim {
bool use_urdf_inertia = false ;

void MultiBody::create(const MultiBodyBuilder &b, PhysicsWorld &physics)
{
    name_ = b.name_ ;

    for( const auto &bl: b.links_ ) {
        Link l ;
        l.name_ = bl.name_ ;
        l.mass_ = bl.mass_ ;
        if ( bl.inertia_ && use_urdf_inertia )
            l.inertia_ = toBulletVector(bl.inertia_.value()) ;
        else {
            if ( bl.shape_ && bl.mass_ != 0 ) bl.shape_->handle()->calculateLocalInertia(bl.mass_, l.inertia_) ;
        }


        l.origin_ = bl.origin_ ;
        l.shape_ = bl.shape_ ;
        l.local_inertial_frame_ = toBulletTransform(bl.local_inertial_frame_) ;
        l.parent_ = this ;
        l.visual_ = bl.visual_ ;

        if ( l.visual_ )
            physics.getVisual()->addChild(l.visual_) ;

        if ( bl.inertia_scaling_ )
            l.inertia_ *= bl.inertia_scaling_.value() ;

        l.stiffness_ = bl.stiffness_ ;
        l.damping_ = bl.damping_ ;
        l.restitution_ = bl.restitution_ ;
        l.lateral_friction_ = bl.lateral_friction_ ;
        l.rolling_friction_ = bl.rolling_friction_ ;
        l.spinning_friction_ = bl.spinning_friction_ ;
        l.has_friction_anchor_ = bl.has_friction_anchor_ ;


        links_.emplace_back(std::move(l)) ;
        link_map_.emplace(bl.name_, links_.size()-1) ;
    }

    for( const auto &bj: b.joints_ ) {
        Joint j ;
        j.name_ = bj.name_ ;
        j.type_ = bj.type_ ;
        j.parent_ = bj.parent_  ;
        j.child_ = bj.child_ ;
        j.j2p_ = toBulletTransform(bj.j2p_) ;
        j.axis_ = toBulletVector(bj.axis_) ;
        j.lower_ = bj.lower_ ;
        j.upper_ = bj.upper_ ;
        j.friction_ = bj.friction_ ;
        j.damping_ = bj.damping_ ;
        j.max_force_ = bj.max_force_ ;
        j.max_velocity_ = bj.max_velocity_ ;


        j.motor_max_force_ = bj.motor_max_force_ ;
        j.mimic_multiplier_ = bj.mimic_multiplier_ ;
        j.mimic_offset_ = bj.mimic_offset_ ;

        joints_.emplace(j.name_, std::move(j)) ;
    }

    for( const auto &bj: b.joints_ ) {
        if ( !bj.mimic_joint_.empty() ) {
            Joint *mimic = findJoint(bj.mimic_joint_) ;
            Joint *target = findJoint(bj.name_) ;
            mimic->mimic_joints_.push_back(target) ;
            target->mimic_multiplier_ = bj.mimic_multiplier_ ;
            target->mimic_offset_ = bj.mimic_offset_ ;
            target->is_mimic_ = true ;
        }
    }

    ms_ = b.ms_ ;

    buildTree() ;

    bool fixed_base = false ;
    if ( root_->mass_ == 0 || root_->name_ == "world" || b.fixed_base_ ) fixed_base = true ;

    body_.reset(new btMultiBody(links_.size() - 1, root_->mass_, root_->inertia_, fixed_base, false)) ;


    btMultiBodyDynamicsWorld *w = static_cast<btMultiBodyDynamicsWorld *>(physics.getDynamicsWorld()) ;

    buildJoints(w, findLink(root_->name_), toBulletTransform(b.base_tr_)) ;

    for( const Link &l: links_ ) {
        if ( l.collider_ == nullptr ) continue ;
        //base and fixed? -> static, otherwise flag as dynamic
        bool isDynamic = (l.mb_index_ < 0 && body_->hasFixedBase()) ? false : true;
        int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
        int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

        w->addCollisionObject(l.collider_.get(), collisionFilterGroup, collisionFilterMask);

        if ( l.mb_index_ >= 0 ) {

            // check whether a body with all fixed joints
            if ( root_->mass_ == 0 ) {
               bool all_joints_fixed = true;
               int test_link_index = l.mb_index_ ;

               do {
                   if ( body_->getLink(test_link_index).m_jointType != btMultibodyLink::eFixed) {
                        all_joints_fixed = false;
                         break;
                   }
                   test_link_index = body_->getLink(test_link_index).m_parent;
               } while ( test_link_index > 0 );

               if ( all_joints_fixed )
                    l.collider_->setCollisionFlags(l.collider_->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
            }

            body_->getLink(l.mb_index_).m_collider = l.collider_.get();
            body_->getLink(l.mb_index_).m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_ALL_PARENT_COLLISION;
        }
    }

    body_->setBaseCollider(root_->collider_.get()) ;

    body_->setBaseWorldTransform(toBulletTransform(b.base_tr_) * root_->local_inertial_frame_) ;

    if ( b.linear_damping_ )
        body_->setLinearDamping(b.linear_damping_.value()) ;

    if ( b.angular_damping_ )
        body_->setAngularDamping(b.angular_damping_.value()) ;

    body_->finalizeMultiDof() ;
    body_->setHasSelfCollision(false);

    updateTransforms() ;

    w->addMultiBody(body_.get()) ;

    // create motors


    for( const Link &l: links_ ) {
        int mb_link_idx = l.mb_index_ ;
        if ( mb_link_idx < 0 ) continue ;

        btMultibodyLink &mb_link = body_->getLink(mb_link_idx) ;

        mb_link.m_linkName = l.name_.c_str() ;
        mb_link.m_jointName = l.parent_joint_->name_.c_str();

        if ( mb_link.m_jointType == btMultibodyLink::eRevolute || mb_link.m_jointType == btMultibodyLink::ePrismatic )
        {
            btMultiBodyJointMotor *motor = new btMultiBodyJointMotor(body_.get(), mb_link_idx, 0, 0, l.parent_joint_->motor_max_force_);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(0, 1);
            l.parent_joint_->motor_ = motor;
            constraints_.emplace_back(unique_ptr<btMultiBodyConstraint>(motor)) ;
            w->addMultiBodyConstraint(motor) ;
            motor->finalizeMultiDof();
        }
    }
#if 0 // mimic joints dont seem to work
    for( const auto &pj: joints_ ) {
        const Joint &j = pj.second ;

        if ( !j.mimic_joints_.empty() ) {
            btVector3 pivot{0, 0, 0} ;

            btMatrix3x3 frame ;
            frame.setIdentity() ;
            int linkA = j.childLink()->mb_index_ ;

            for( const Joint *t: j.mimic_joints_ ) {
                int linkB = t->childLink()->mb_index_ ;
                btMultiBodyGearConstraint *constraint = new btMultiBodyGearConstraint(body_.get(), linkA, body_.get(), linkB,  pivot, pivot, frame, frame) ;
                constraint->setGearRatio(t->mimic_multiplier_) ;
                constraint->setMaxAppliedImpulse(10.0) ;
                constraints_.emplace_back(unique_ptr<btMultiBodyGearConstraint>(constraint)) ;
                w->addMultiBodyConstraint(constraint);
                constraint->finalizeMultiDof();
            }
        }
    }
#endif


}

int MultiBody::findLink(const std::string &name) const {
    auto it = link_map_.find(name) ;
    if ( it == link_map_.end() ) return -1 ;
    else return it->second ;
}

const Joint *MultiBody::findJoint(const string &name) const {
    auto it = joints_.find(name) ;
    if ( it == joints_.end() ) return nullptr ;
    else return &(it->second) ;
}

Joint *MultiBody::findJoint(const string &name) {
    auto it = joints_.find(name) ;
    if ( it == joints_.end() ) return nullptr ;
    else return &(it->second) ;
}


const std::vector<string> MultiBody::jointNames() const {
    vector<string> names ;
    for( const auto &jp: joints_ )
        names.emplace_back(jp.first) ;
    return names ;
}

void MultiBody::computeIndicesRecursive(Link *root, uint &count) {
    for( Link *child: root->child_links_ ) {
        child->mb_index_ = count ++ ;
        computeIndicesRecursive(child, count) ;
    }
}

void MultiBody::updateTransforms()
{
    btAlignedObjectArray<btQuaternion> scratch_q;
    btAlignedObjectArray<btVector3> scratch_m;
    body_->forwardKinematics(scratch_q, scratch_m);
    body_->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
}

void MultiBody::buildTree() {
    map<string, string> parent_link_tree ;

    for( auto &jp: joints_ ) {
        Joint &j = jp.second ;

        assert( !j.child_.empty() && !j.parent_.empty() )  ;

        int child_link_idx = findLink(j.child_) ;
        int parent_link_idx = findLink(j.parent_) ;

        assert(child_link_idx >=0 && parent_link_idx >=0) ;

        Link *child_link = &links_[child_link_idx] ;
        Link *parent_link = &links_[parent_link_idx] ;

        j.parent_link_ = parent_link ;
        j.child_link_ = child_link ;

        child_link->parent_link_ = parent_link ;
        child_link->parent_joint_ = &j ;
        parent_link->child_joints_.push_back(&j) ;
        parent_link->child_links_.push_back(child_link) ;
        parent_link_tree[child_link->name_] = j.parent_;
    }

    // find root

    root_ = nullptr ;

    for ( const auto &l: links_ ) {
        auto it = parent_link_tree.find(l.name_) ;
        if ( it == parent_link_tree.end() ) { // no parent thus root
            if ( root_ == nullptr ) {
                int base_link_idx = findLink(l.name_) ;
                if ( base_link_idx >=0 ) root_ = &links_[base_link_idx] ;
            }
        }
    }

    assert(root_ != nullptr) ;


    root_->mb_index_ = -1 ;

    // we need to assure that child indices are greater than their parents
    uint count = 0 ;
    computeIndicesRecursive(root_, count) ;
}

void MultiBody::buildCollisionObject(int link_idx, const btTransform &link_transform) {
    Link &link = links_[link_idx] ;

    if ( link.shape_ ) {
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(body_.get(), link.mb_index_);

        col->setUserPointer(&link) ;

        btCompoundShape *proxy = new btCompoundShape() ;

        proxy->addChildShape(  link.local_inertial_frame_.inverse() * toBulletTransform(link.origin_) , link.shape_->handle()) ;
        link.proxy_.reset(proxy) ;

        if ( link.lateral_friction_ )
            col->setFriction(link.lateral_friction_.value()) ;

        if ( link.rolling_friction_ )
            col->setRollingFriction(link.rolling_friction_.value()) ;

        if ( link.spinning_friction_ )
            col->setSpinningFriction(link.spinning_friction_.value()) ;

        if ( link.stiffness_ && link.damping_ )
            col->setContactStiffnessAndDamping(link.stiffness_.value(), link.damping_.value()) ;

        if ( link.restitution_ )
            col->setRestitution(link.restitution_.value()) ;

        if ( link.has_friction_anchor_ )
            col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);

        col->setCollisionShape(proxy);

        col->setWorldTransform(link_transform );

        link.collider_.reset(col) ;
    }
}

void MultiBody::buildJoints(btMultiBodyDynamicsWorld *w, int link_idx, const btTransform &parent_transform_in_world_space) {

    btTransform link_transform_in_world_space;
    link_transform_in_world_space.setIdentity();

    Link &link = links_[link_idx] ;

    Link *parent_link = link.parent_link_ ;

    btTransform parent2joint, linkTransform;
    parent2joint.setIdentity();

    btTransform parentTransform = parent_transform_in_world_space ;
    btTransform parent_local_inertial_frame ;
    parent_local_inertial_frame.setIdentity() ;
    btTransform local_inertial_frame = link.local_inertial_frame_ ;

    Joint *parent_joint  = nullptr ;

    if ( parent_link ) {
        parent_joint  = link.parent_joint_ ;
        parent2joint = parent_joint->j2p_ ;
        parent_local_inertial_frame = parent_link->local_inertial_frame_ ;
    }

    linkTransform = parentTransform * parent2joint  ;

    buildCollisionObject(link_idx, linkTransform) ;

    if ( parent_joint ) {
        parent_joint->body_ = body_.get() ;

        btMultibodyLink &mb_link = body_->getLink(link.mb_index_) ;
        mb_link.m_jointDamping = parent_joint->damping_;
        mb_link.m_jointFriction = parent_joint->friction_;
        mb_link.m_jointLowerLimit = parent_joint->lower_;
        mb_link.m_jointUpperLimit = parent_joint->upper_;
        mb_link.m_jointMaxForce = parent_joint->max_force_;
        mb_link.m_jointMaxVelocity = parent_joint->max_velocity_;

        btTransform offsetInA = parent_local_inertial_frame.inverse() * parent2joint ;
        btTransform offsetInB = local_inertial_frame.inverse();
        btQuaternion parentRotToThis = offsetInB.getRotation() * offsetInA.inverse().getRotation();

        if ( parent_joint->type_ == RevoluteJoint  || parent_joint->type_ == ContinuousJoint ) {
            body_->setupRevolute(link.mb_index_, link.mass_, link.inertia_, parent_link->mb_index_,
                                 parentRotToThis, quatRotate(offsetInB.getRotation(), parent_joint->axis_), offsetInA.getOrigin(),
                                 -offsetInB.getOrigin(),
                                 true) ;
          if ( parent_joint->type_ == RevoluteJoint && parent_joint->lower_ <= parent_joint->upper_ ) {
                btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(body_.get(), link.mb_index_, parent_joint->lower_, parent_joint->upper_);
                constraints_.emplace_back(unique_ptr<btMultiBodyConstraint>(con)) ;
                w->addMultiBodyConstraint(con);
            }

        } else if ( parent_joint->type_ == FixedJoint ) {
            body_->setupFixed(link.mb_index_, link.mass_, link.inertia_, parent_link->mb_index_,
                              parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin());
        } else if ( parent_joint->type_ == PrismaticJoint ) {
            body_->setupPrismatic(link.mb_index_, link.mass_, link.inertia_, parent_link->mb_index_,
                                                                            parentRotToThis, quatRotate(offsetInB.getRotation(), parent_joint->axis_), offsetInA.getOrigin(),
                                                                            -offsetInB.getOrigin(),
                                                                            true);

            if ( parent_joint->lower_ <= parent_joint->upper_ ) {
                btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(body_.get(), link.mb_index_, parent_joint->lower_, parent_joint->upper_);
                constraints_.emplace_back(unique_ptr<btMultiBodyConstraint>(con)) ;
                w->addMultiBodyConstraint(con);
            }

        } else if ( parent_joint->type_ == SphericalJoint ) {
            body_->setupSpherical(link.mb_index_, link.mass_, link.inertia_, parent_link->mb_index_,
                                        parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin(),
                                        true);
        } else if ( parent_joint->type_ == PlanarJoint ) {
           body_->setupPlanar(link.mb_index_, link.mass_, link.inertia_, parent_link->mb_index_,
                                        parentRotToThis, quatRotate(offsetInB.getRotation(), parent_joint->axis_), offsetInA.getOrigin(),
                                        true);
        }
    }

    for ( const Link *cl: link.child_links_ ) {
        buildJoints(w, findLink(cl->name_), linkTransform) ;
    }

}

void MultiBody::getLinkTransforms(std::map<string, Isometry3f> &names) const
{
    for( const Link &l: links_ ) {
        if ( l.collider_ ) {
            btTransform tr = l.collider_->getWorldTransform() ;
            names.emplace(l.name_, Isometry3f(toEigenTransform(tr))) ;
        }
    }
}

const Link *MultiBody::getLink(const string &name) const {
    int idx = findLink(name) ;
    if ( idx >= 0 ) return &links_[idx] ;
    else return nullptr ;
}

void MultiBody::setBaseWorldTransform(const Eigen::Isometry3f &tr) {
     body_->setBaseWorldTransform(toBulletTransform(tr) * root_->local_inertial_frame_) ;
}

int MultiBody::getJointIndex(const string &name) const {

    const Joint *j = findJoint(name) ;
    assert( j != nullptr ) ;
    return j->childLink()->mb_index_ ;
}

double MultiBody::getJointPosition(const string &name) const {
    assert(body_) ;
    return static_cast<double>(body_->getJointPos(getJointIndex(name))) ;
}

double MultiBody::getJointVelocity(const string &name) const {
    assert(body_) ;
    return static_cast<double>(body_->getJointVel(getJointIndex(name))) ;
}

double MultiBody::getJointTorque(const string &name) const {
    assert(body_) ;
    return static_cast<double>(body_->getJointTorque(getJointIndex(name))) ;
}

void MultiBody::setJointPosition(const string &name, double v) {
    assert(body_) ;
    body_->setJointPos(getJointIndex(name), static_cast<btScalar>(v)) ;
    body_->setJointVel(getJointIndex(name), 0) ;
    updateTransforms() ;
}

void MultiBody::setJointVelocity(const string &name, double v) {
    assert(body_) ;
    body_->setJointVel(getJointIndex(name), static_cast<btScalar>(v)) ;
}

void MultiBody::setJointTorque(const string &name, double v) {
    assert(body_) ;
    body_->addJointTorque(getJointIndex(name), static_cast<btScalar>(v)) ;
}

void MultiBody::setTargetPosition(const string &name, double v)
{
    Joint *j = findJoint(name) ;
    assert(j) ;
    j->setTargetPosition(v);
}

string MultiBody::name() const {
    return name_ ;
}

void Joint::setTargetVelocity(float v, float kd) {
    assert( motor_) ;
    motor_->setVelocityTarget(static_cast<btScalar>(v), kd) ;
    motor_->setPositionTarget(0, 0);
}

void Joint::setTargetPosition(float v, float kp) {
    assert( motor_) ;
    motor_->setVelocityTarget(0.0, 1.0);
    motor_->setPositionTarget(static_cast<btScalar>(v), kp);
}

void Joint::setMotorControl(const MotorControl &c)
{
    if ( c.max_velocity_ )
        motor_->setRhsClamp(c.max_velocity_.value()) ;
    if ( c.max_force_ )
        motor_->setMaxAppliedImpulse(c.max_force_.value());

    float kd = c.kd_.value_or(0.1f) ;
    float kp = c.kp_.value_or(0.1f) ;

    if ( c.mode_ == POSITION_CONTROL ) {
        float target_velocity = c.target_velocity_.value_or(0.0f) ;

        setTargetVelocity(target_velocity, kd) ;
        setTargetPosition(c.target_pos_.value(), kp) ;
    } else {
        float target_pos = c.target_pos_.value_or(getPosition()) ;

        setTargetVelocity(c.target_velocity_.value(), kd) ;
        setTargetPosition(0, 0);

    }

}


void Joint::setMotorMaxImpulse(float v)
{
    assert(motor_) ;
    motor_->setMaxAppliedImpulse(v);
}

float Joint::getPosition() {
    assert(body_) ;
    return static_cast<float>(body_->getJointPos(child_link_->mb_index_)) ;
}

float Joint::getVelocity() {
    assert(body_) ;
    return static_cast<float>(body_->getJointVel(child_link_->mb_index_)) ;
}

float Joint::getTorque() {
    assert(body_) ;
    return static_cast<float>(body_->getJointTorque(child_link_->mb_index_)) ;
}

void Joint::setPosition(float v) {
    assert(body_) ;
    body_->setJointPos(child_link_->mb_index_, static_cast<btScalar>(v)) ;
}

void Joint::setVelocity(float v) {
    assert(body_) ;
    body_->setJointVel(child_link_->mb_index_, static_cast<btScalar>(v)) ;
}

void Joint::setTorque(float v) {
    assert(body_) ;
    body_->addJointTorque(child_link_->mb_index_, static_cast<btScalar>(v)) ;
}

Isometry3f Link::getWorldTransform() const {
    btTransform tr = collider_->getWorldTransform();
    return toEigenTransform(tr) ;
}


}
