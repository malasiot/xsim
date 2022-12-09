#include <xsim/kinematic.hpp>

using namespace std ;
using namespace Eigen ;
using namespace xviz ;

namespace xsim {

class KRevoluteJoint: public KinematicJoint
{
public:
    KRevoluteJoint(const Vector3f &axis, double lower, double upper):
        lower_(lower), upper_(upper), axis_(axis), position_(0.0)   {
        local_ = Isometry3f::Identity() ;
    }

    double getPosition() const override {
        return position_ ;
    }

    void getLimits(double &lower, double &upper) const override {
        lower = lower_ ;
        upper = upper_ ;
    }

    void setPosition(double val, KinematicJoint::LimitCheckType lt)  {
        if ( lt == KinematicJoint::LIMIT_CHECK_CLAMP_VALUE )  {
            if ( val < lower_ ) position_ = lower_ ;
            else if ( val > upper_ ) position_ = upper_ ;
            else position_ = val ;
        }
        else if ( lt == KinematicJoint::LIMIT_CHECK_THROW_EXCEPTION ) {
            if ( val < lower_ || val > upper_  )
                throw std::runtime_error("DOF out of range") ;
            else position_ = val ;
        }
        else {
            if ( val >= lower_ && val <= upper_ ) position_ = val ;
        }
    }

    double getDistance(double v1, double v2) const override {
        return fabs(v1 - v2) ;
    }

    bool allows(double val) const override { return fabs(val) < M_PI && val >= lower_ && val <= upper_ ; }


    void updateLocalTransform() override {
        local_.setIdentity() ;
        local_.rotate(AngleAxisf(position_, axis_)) ;
    }

    KinematicJoint::JointType getType() const override { return REVOLUTE_JOINT ; }

    double lower_, upper_ ;
    Vector3f axis_ ;
    double position_ ;
};


class KPrismaticJoint: public KinematicJoint
{
public:
    KPrismaticJoint(const Vector3f &axis, double lower, double upper):
        lower_(lower), upper_(upper), axis_(axis), position_(0.0)   {
        local_ = Isometry3f::Identity() ;
    }

    double getPosition() const override {
        return position_ ;
    }

    void getLimits(double &lower, double &upper) const override {
        lower = lower_ ;
        upper = upper_ ;
    }

    void setPosition(double val, KinematicJoint::LimitCheckType lt)  {
        if ( lt == KinematicJoint::LIMIT_CHECK_CLAMP_VALUE )  {
            if ( val < lower_ ) position_ = lower_ ;
            else if ( val > upper_ ) position_ = upper_ ;
            else position_ = val ;
        }
        else if ( lt == KinematicJoint::LIMIT_CHECK_THROW_EXCEPTION ) {
            if ( val < lower_ || val > upper_  )
                throw std::runtime_error("DOF out of range") ;
            else position_ = val ;
        }
        else {
            if ( val >= lower_ && val <= upper_ ) position_ = val ;
        }
    }

    double getDistance(double v1, double v2) const override {
        return fabs(v1 - v2) ;
    }

    bool allows(double val) const override { return fabs(val) < M_PI && val >= lower_ && val <= upper_ ; }

    void updateLocalTransform() {
        local_.setIdentity() ;
        local_.translate(axis_* position_) ;
    }

    KinematicJoint::JointType getType() const { return PRISMATIC_JOINT ; }

    double lower_, upper_ ;
    Vector3f axis_ ;
    double position_ ;
};

static double normalizeCircularAngle(double theta, double l, double u) {
    if ( theta < l ) {
        double range = u - l;

        assert(range > 0) ;
        theta += range;

        while (theta < l) {
            theta += range;
        }
    }
    else if ( theta > u ) {
        double range = u - l ;
        assert( range > 0 ) ;
        theta -= range;
        while (theta > u ) {
            theta -= range;
        }
    }
    return theta;
}

static double subtractCircularAngle(double f0, double f1)
{
    return normalizeCircularAngle(f0-f1, -M_PI, M_PI);
}

class KContinuousJoint: public KinematicJoint
{
public:
    KContinuousJoint(const Vector3f &axis):
        axis_(axis), position_(0.0)   {
        local_ = Isometry3f::Identity() ;
    }

    double getPosition() const override {
        return position_ ;
    }

    void getLimits(double &lower, double &upper) const override {
        lower = -M_PI ;
        upper = M_PI ;
    }

    void setPosition(double val, KinematicJoint::LimitCheckType lt)  {
        position_ = val ;
    }

    double getDistance(double v1, double v2) const override {
        return subtractCircularAngle(v1, v2) ;
    }

    void updateLocalTransform() {
        local_.setIdentity() ;
        local_.translate(axis_* position_) ;
    }

    KinematicJoint::JointType getType() const { return CONTINUOUS_JOINT ; }

    Vector3f axis_ ;
    double position_ ;
};

class KFixedJoint: public KinematicJoint
{
public:
    KFixedJoint() {
        local_.setIdentity() ;
    }

    double getPosition() const override { return 0 ; }
    void setPosition(double, KinematicJoint::LimitCheckType) override {}
    void updateLocalTransform() override { }
    double getDistance(double val1, double val2) const override { return 0.0 ; }
    void getLimits(double &lower, double &upper) const override {
        lower =  upper = 0.0 ;
    }
    bool hasDOF() const override { return false ; }

    KinematicJoint::JointType getType() const { return FIXED_JOINT ; }
};

void KinematicModel::create(const URDFRobot &robot) {


    for( const auto &bp: robot.links_ ) {
        const string &name = bp.first ;
        const URDFLink &link = bp.second ;

        KinematicLinkPtr cl(new KinematicLink);
        cl->name_ = name ;
        links_.emplace(name, cl) ;
    }

    for( const auto &jpr: robot.joints_ ) {

        const URDFJoint &j = jpr.second ;

        KinematicJointPtr jp ;
        if ( j.type_ == "revolute" ) {
            jp.reset(new KRevoluteJoint(j.axis_, j.lower_, j.upper_)) ;
        } else if ( j.type_ == "prismatic" ) {
            jp.reset(new KPrismaticJoint(j.axis_, j.lower_, j.upper_)) ;
        } else if ( j.type_ == "continuous" ) {
            jp.reset(new KContinuousJoint(j.axis_)) ;
        } else if ( j.type_ == "fixed" ) {
            jp.reset(new KFixedJoint()) ;
        }

        jp->origin_ = j.origin_ ;
        jp->name_ = j.name_ ;

        auto it = links_.find(j.parent_) ;
        if ( it == links_.end() ) continue ;
        else jp->parent_ = it->second ;

        it = links_.find(j.child_) ;
        if ( it == links_.end() ) continue ;
        else jp->child_ = it->second ;

        joints_.emplace(j.name_, jp) ;
    }

    for( const auto &jp: joints_ ) {
        const auto joint = jp.second ;
        KinematicLinkPtr parent = joint->parent_ ;
        KinematicLinkPtr child = joint->child_ ;

        assert( parent && child ) ;

        parent->child_joints_.push_back(joint) ;
        child->parent_joint_ = joint;
    }

    for( const auto &lp: links_ ) {
        if ( lp.second->parent_joint_ == nullptr )
            root_ = lp.second ;
    }

    updateWorldTransforms();
}

void KinematicModel::updateWorldTransforms() {
    root_->updateWorldTransform();
}

void KinematicLink::updateWorldTransform() {
    if ( parent_joint_ )
        world_ = parent_joint_->parentLink()->world_ * parent_joint_->origin() * parent_joint_->local();
    for( const auto &cl: child_joints_ ) {
        cl->child_->updateWorldTransform();
    }
}

void KinematicJoint::updateLinkTransforms() {
    child_->updateWorldTransform();
}

void KinematicModel::setWorldTransform(const Isometry3f &tr) {
    root_->world_ = tr ;
    updateWorldTransforms();
}

KinematicJointPtr KinematicModel::getJoint(const std::string &name) const {
    auto it = joints_.find(name);
    if ( it == joints_.end() ) return nullptr ;
    else return (*it).second ;
}

KinematicLinkPtr KinematicModel::getLink(const std::string &name) const {
    auto it = links_.find(name) ;
    if ( it == links_.end() ) return nullptr ;
    else return (*it).second ;
}

double KinematicModel::getJointPosition(const std::string &name) const {
    KinematicJointPtr joint = getJoint(name) ;
    assert(joint) ;
    return joint->getPosition() ;
}

void KinematicModel::setJointPosition(const std::string &name, double val, KinematicJoint::LimitCheckType type) {
    KinematicJointPtr joint = getJoint(name) ;
    assert(joint) ;

    joint->setPosition(val, type) ;
    joint->updateLocalTransform() ;
    joint->updateLinkTransforms() ;
}



JointState KinematicModel::getJointState() const {
    JointState state ;
    for( const auto &jp: joints_ ) {
        auto joint = jp.second ;
        if ( joint->hasDOF() ) {
            state.emplace(jp.first, joint->getPosition()) ;
        }
    }
    return state ;
}

JointState KinematicModel::getJointState(const vector<string> &joints) const {
    JointState state ;
    for( const auto &name: joints ) {
        auto joint = getJoint(name) ;
        if ( !joint ) continue ;
        if ( joint->hasDOF() )
           state.emplace(name, joint->getPosition()) ;
     }
    return state ;
}

void KinematicModel::setJointState(const JointState &state, KinematicJoint::LimitCheckType lt) {
    for ( const auto &sp: state ) {
        const string &name = sp.first ;
        double val = sp.second ;

        auto joint = getJoint(name) ;
        if ( joint ) {
            joint->setPosition(val, lt) ;
            joint->updateLocalTransform();
        }
    }

    updateWorldTransforms() ;
}

Isometry3f KinematicModel::getLinkTransform(const string &link_name) const {
    auto link = getLink(link_name) ;
    assert(link) ;
    return link->world_ ;
}

map<string, Isometry3f> KinematicModel::getLinkTransforms() const {
    map<string, Isometry3f> trs ;
    for( const auto &lp: links_ ) {
        trs.emplace(lp.first, lp.second->world_) ;
    }
    return trs ;
}

std::vector<string> KinematicModel::getJointNames() const {
    vector<string> joints ;
    for( const auto &jp: joints_ ) {
        auto joint = jp.second ;
        if ( joint->hasDOF() ) {
            joints.push_back(jp.first) ;
        }
    }
    return joints ;
}

}
