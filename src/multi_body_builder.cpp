#include <xsim/multi_body_builder.hpp>
#include <xsim/multi_body.hpp>
#include <xviz/scene/node.hpp>
#include <iostream>

using namespace std ;
using namespace Eigen ;
using namespace xviz ;


namespace xsim {

extern NodePtr createLinkGeometry(const URDFGeometry *urdf_geom, MaterialPtr mat, Vector3f &scale);

MBLink &MultiBodyBuilder::addLink(const std::string &name, float mass, CollisionShapePtr cshape, const Eigen::Isometry3f &origin)
{
    MBLink l ;
    l.name_ = name ;
    l.mass_ = mass ;
    l.origin_ = origin ;
    l.shape_ = cshape ;

    links_.emplace_back(std::move(l)) ;

    return links_.back(); ;

}

MBJoint &MultiBodyBuilder::addJoint(const std::string &name, JointType type, const std::string &parent, const std::string &child, const Eigen::Isometry3f &j2p)
{
    MBJoint j ;
    j.name_ = name ;
    j.type_ = type ;
    j.parent_ = parent  ;
    j.child_ = child ;
    j.j2p_ = j2p ;

    joints_.emplace_back(std::move(j)) ;
    return joints_.back() ;
}

MBLink *MultiBodyBuilder::findLink(const string &name) {
    return nullptr ;
}


void MultiBodyBuilder::setMimic(const URDFJoint &joint, MBJoint &j) {
    if ( !joint.mimic_joint_.empty() ) {
        j.mimic_multiplier_ = joint.mimic_multiplier_ ;
        j.mimic_offset_ = joint.mimic_offset_ ;
        j.mimic_joint_ = joint.mimic_joint_ ;
    }
}


MultiBodyBuilder & MultiBodyBuilder::loadURDF(const URDFRobot &rb) {

    base_tr_ = rb.worldTransform() ;

    map<string, MaterialPtr> materials ;

    // create materials

    for( const auto &mp: rb.materials_ ) {
        const URDFMaterial &mat = *mp.second ;
        const std::string &name = mp.first ;

        if ( mat.texture_path_.empty() ) {
            Vector3f clr = mat.diffuse_color_ ;
            PhongMaterial *material = new PhongMaterial ;
            material->setSide(Material::Side::Both) ;
            material->setShininess(0);
            material->setSpecularColor({0, 0, 0}) ;
            material->setDiffuseColor(clr) ;

            materials.emplace(name, MaterialPtr(material)) ;
        } else {
            std::shared_ptr<Image> im(new Image(mat.texture_path_)) ;
            Texture2D *s = new Texture2D(im, Sampler2D()) ;

            PhongMaterial *material = new PhongMaterial() ;
            material->setDiffuseTexture(s);
            material->setShininess(0);
            material->setSpecularColor({0, 0, 0}) ;
            material->setDiffuseColor({0, 0, 0}) ;

            materials.emplace(name, MaterialPtr(material)) ;
        }
    }


    for( const auto &lp: rb.links_ ) {
        const string &name  = lp.first ;
        const URDFLink &link = lp.second ;

        vector<CollisionShapePtr> shapes ;
        vector<Isometry3f> origins ;

        for( const auto &geom: link.collision_geoms_ ) {

            Isometry3f col_origin = Isometry3f::Identity();
            col_origin = geom->origin_ ;

            CollisionShapePtr shape ;

            if ( const URDFBoxGeometry *g = dynamic_cast<const URDFBoxGeometry *>(geom.get()) ) {
                shape.reset(new BoxCollisionShape(g->he_))  ;
            } else if ( const URDFCylinderGeometry *g = dynamic_cast<const URDFCylinderGeometry *>(geom.get()) ) {
                shape.reset(new CylinderCollisionShape(g->radius_, g->height_/2.0, CylinderCollisionShape::ZAxis))  ;
            } else if ( const URDFMeshGeometry *g = dynamic_cast<const URDFMeshGeometry *>(geom.get()) ) {
                shape.reset(new ConvexHullCollisionShape(g->path_, g->scale_));
            } else if ( const URDFSphereGeometry *g = dynamic_cast<const URDFSphereGeometry *>(geom.get()) ) {
                shape.reset(new SphereCollisionShape(g->radius_));
            }

            if ( shape ) {
                //  shape->handle()->setMargin(0.001) ;
                shapes.push_back(shape) ;
                origins.push_back(col_origin) ;
            } else {
                cout << name << endl ;
            }
        }

        float mass = 0 ;
        Isometry3f inertial_frame ;
        inertial_frame.setIdentity() ;
        Vector3f inertia(0, 0, 0) ;

        CollisionShapePtr col_shape ;
        Isometry3f col_origin ;
        col_origin.setIdentity() ;

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

        if ( link.inertial_ ) {
            mass = link.inertial_->mass_ ;
            inertial_frame = link.inertial_->origin_ ;
        }

        auto &l = addLink(name, mass, col_shape, col_origin) ;
        l.setLocalInertialFrame(inertial_frame) ;


        if ( link.inertial_ && link.inertial_->inertia_ ) {
            const auto &im = link.inertial_->inertia_.value() ;
            inertia = im.diagonal() ;
            l.setInertia(inertia) ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_INERTIA_SCALING ) {
            l.inertia_scaling_ = link.contact_info_.inertia_scaling_ ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_LATERAL_FRICTION ) {
            l.lateral_friction_ = link.contact_info_.lateral_friction_ ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_ROLLING_FRICTION ) {
            l.rolling_friction_ = link.contact_info_.rolling_friction_ ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_SPINNING_FRICTION ) {
            l.spinning_friction_ = link.contact_info_.spinning_friction_ ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_DAMPING ) {
            l.damping_ = link.contact_info_.damping_ ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_STIFFNESS ) {
            l.stiffness_ = link.contact_info_.stiffness_ ;
        }

        if ( link.contact_info_.flags_ & URDFContact::HAS_RESTITUTION ) {
            l.restitution_ = link.contact_info_.restitution_ ;
        }

        // visual

        NodePtr link_node(new Node) ;
        link_node->setName(link.name_) ;

        Isometry3f local_inertial_frame = Isometry3f::Identity() ;
        if ( link.inertial_ )
            local_inertial_frame = link.inertial_->origin_ ;

        for( const auto &geom: link.visual_geoms_ ) {

            const std::string &matref = geom->material_ref_ ;

            MaterialPtr mat ;
            auto mat_it = materials.find(matref) ;
            if ( mat_it != materials.end() )
                mat = mat_it->second ;

            Vector3f scale{1, 1, 1} ;
            NodePtr geom_node = createLinkGeometry(geom.get(), mat, scale) ;

            geom_node->setTransform(local_inertial_frame.inverse() * geom->origin_) ;

            link_node->addChild(geom_node) ;


        }

        l.setVisualShape(link_node) ;
    }



    for( const auto &jp: rb.joints_ ) {
        const string &name  = jp.first ;
        const URDFJoint &joint = jp.second ;

        if ( joint.type_ == "revolute"  ) {
            auto &j = addJoint(name, RevoluteJoint, joint.parent_, joint.child_, joint.origin_) ;
            j.setAxis(joint.axis_) ;
            j.setLimits(joint.lower_, joint.upper_) ;
            j.setDamping(joint.damping_) ;
            j.setFriction(joint.friction_) ;
            j.setMaxForce(joint.effort_) ;
            j.setMaxVelocity(joint.velocity_) ;
            setMimic(joint, j) ;
        } else if ( joint.type_ == "continuous" ) {
            auto &j = addJoint(name, ContinuousJoint, joint.parent_, joint.child_, joint.origin_) ;
            j.setAxis(joint.axis_) ;
            j.setDamping(joint.damping_) ;
            j.setFriction(joint.friction_) ;
            j.setMaxForce(joint.effort_) ;
            j.setMaxVelocity(joint.velocity_) ;
            setMimic(joint, j) ;
        } else if ( joint.type_ == "fixed" ) {
            auto &j = addJoint(name, FixedJoint, joint.parent_, joint.child_, joint.origin_) ;
        } else if ( joint.type_ == "prismatic" ) {
            auto &j = addJoint(name, PrismaticJoint, joint.parent_, joint.child_, joint.origin_) ;
            j.setAxis(joint.axis_) ;
            j.setLimits(joint.lower_, joint.upper_) ;
            j.setDamping(joint.damping_) ;
            j.setFriction(joint.friction_) ;
            j.setMaxForce(joint.effort_) ;
            j.setMaxVelocity(joint.velocity_) ;
            setMimic(joint, j) ;
        } else if ( joint.type_ == "planar" ) {
            auto &j = addJoint(name, PlanarJoint, joint.parent_, joint.child_, joint.origin_) ;
            j.setAxis(joint.axis_) ;
            j.setDamping(joint.damping_) ;
            j.setFriction(joint.friction_) ;
            j.setMaxForce(joint.effort_) ;
            j.setMaxVelocity(joint.velocity_) ;
        } else if ( joint.type_ == "floating" ) {
            assert("unsupported") ;
        }
    }

    return *this ;
}

void MultiBodyUpdateScene::notify(const std::map<string, Isometry3f> &link_transforms) {
    node_->updateTransforms(link_transforms);
}



}
