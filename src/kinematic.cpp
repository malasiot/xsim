#include <xsim/kinematic.hpp>

using namespace std ;
using namespace Eigen ;
using namespace xviz ;

namespace xsim {

void KinematicCollisionShape::create(const URDFRobot &robot) {

    uint count = 0 ;
    for( const auto &bp: robot.links_ ) {
        const string &name = bp.first ;
        const URDFLink &link = bp.second ;

        Link cl ;

        vector<CollisionShapePtr> shapes ;
        vector<Isometry3f> origins ;

        for( const auto &geom: link.collision_geoms_ ) {

            Isometry3f col_origin = geom->origin_ ;

            origins.push_back(col_origin) ;

            CollisionShapePtr cs = makeCollisionShape(geom.get()) ;

            if ( cs ) {
                shapes.push_back(cs) ;
            }
        }

        CollisionShapePtr col_shape ;
        Isometry3f col_origin = Isometry3f::Identity() ;

        if ( shapes.empty() ) {
            col_shape.reset(new GroupCollisionShape) ;
            col_shape->setMargin(0.001);
        }   else if ( shapes.size() == 1 ) {
            col_shape = shapes[0] ;
            col_origin = origins[0] ;
        } else {
            GroupCollisionShape *gc = new GroupCollisionShape() ;

            for( uint i=0 ; i<shapes.size() ; i++ ) {
                gc->addChild(shapes[i], origins[i]) ;
            }
            col_shape.reset(gc) ;
            col_shape->setMargin(0.001);
        }


        addChild(col_shape, col_origin) ;
        cl.col_shape_index_ = count++ ;
        cl.origin_ = col_origin ;
        cl.shape_ = col_shape ;
        links_.emplace(name, cl) ;
    }

    map<string, string> parent_link_tree ;

    for( const auto &jp: robot.joints_ ) {

        const URDFJoint &j = jp.second ;

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

CollisionShapePtr KinematicCollisionShape::makeCollisionShape(const URDFGeometry *geom) {
    float scale = 1.0 ;
    CollisionShapePtr shape ;

    if ( const URDFBoxGeometry *g = dynamic_cast<const URDFBoxGeometry *>(geom) ) {
        shape.reset(new BoxCollisionShape(g->he_))  ;
    } else if ( const URDFCylinderGeometry *g = dynamic_cast<const URDFCylinderGeometry *>(geom) ) {
        shape.reset(new CylinderCollisionShape(g->radius_, g->height_/2.0, CylinderCollisionShape::ZAxis))  ;
    } else if ( const URDFMeshGeometry *g = dynamic_cast<const URDFMeshGeometry *>(geom) ) {
        shape.reset(new ConvexHullCollisionShape(g->path_, g->scale_));
    } else if ( const URDFSphereGeometry *g = dynamic_cast<const URDFSphereGeometry *>(geom) ) {
        shape.reset(new SphereCollisionShape(g->radius_));
    }

    if ( shape ) shape->setLocalScale(scale) ;
    return shape ;
}


}
