#include <cvx/viz/physics/kinematic.hpp>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {


void ArticulatedCollisionShape::create(const urdf::Robot &robot) {

    uint count = 0 ;
    for( const auto &bp: robot.links_ ) {
        const string &name = bp.first ;
        const urdf::Link &link = bp.second ;

        Link cl ;

        if ( link.collision_geom_ ) {
            CollisionShape::Ptr cs = makeCollisionShape(link.collision_geom_.get()) ;
            if ( cs ) {
                addChild(cs, Affine3f::Identity()) ;
                cl.col_shape_index_ = count++ ;
                cl.origin_ = link.collision_geom_->origin_ ;
                links_.emplace(name, cl) ;
            }
        }
    }


    map<string, string> parent_link_tree ;

    for( const auto &jp: robot.joints_ ) {

        const urdf::Joint &j = jp.second ;

        Joint joint ;

        joint.origin_ = j.origin_ ;
        string parent_link_name = j.parent_ ;
        string child_link_name = j.child_ ;

        Link *parent_link, *child_link ; ;

        auto pl_it = links_.find(parent_link_name) ;
        if ( pl_it == links_.end() ) continue ;
        else parent_link = &pl_it->second ;

        auto cl_it = links_.find(child_link_name) ;
        if ( cl_it == links_.end() ) continue ;
        else child_link = &cl_it->second ;

        parent_link_tree[child_link_name] = parent_link_name ;

        joint.type_ = j.type_ ;
        joint.parent_ = parent_link ;
        joint.child_ = child_link ;
        joint.lower_limit_ = j.lower_ ;
        joint.upper_limit_ = j.upper_ ;
        joint.axis_ = j.axis_ ;
        joint.mimic_ = j.mimic_joint_ ;

        joints_.emplace(j.name_, joint) ;
    }
}

CollisionShape::Ptr ArticulatedCollisionShape::makeCollisionShape(const urdf::Geometry *geom) {
    float scale = 1.0 ;
    CollisionShape::Ptr shape ;

    if ( const urdf::BoxGeometry *g = dynamic_cast<const urdf::BoxGeometry *>(geom) ) {
        shape.reset(new BoxCollisionShape(g->he_))  ;
    } else if ( const urdf::CylinderGeometry *g = dynamic_cast<const urdf::CylinderGeometry *>(geom) ) {
        shape.reset(new CylinderCollisionShape(g->radius_, g->height_))  ;
    } else if ( const urdf::MeshGeometry *g = dynamic_cast<const urdf::MeshGeometry *>(geom) ) {
        shape.reset(new StaticMeshCollisionShape(g->path_));
    }

    if ( shape ) shape->setLocalScale(scale) ;
    return shape ;
}


}}
