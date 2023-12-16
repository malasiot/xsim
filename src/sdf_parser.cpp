#include "sdf_parser.hpp"

#include <xsim/sdf_world.hpp>

using namespace Eigen ;
using namespace std ;
using namespace pugi ;

namespace xsim {

static std::string parentPath(const std::string &fpath) {
    size_t pos = fpath.find_last_of('/') ;
    return fpath.substr(0, pos) ;
}

static string resolveUri(const std::string &uri, const std::string &path) {
    string rpath = parentPath(path) + '/' + uri ;
    return rpath;
}

static Eigen::Vector3f parse_vec3(const std::string &s) {
    istringstream strm(s) ;
    float x, y, z ;
    strm >> x >> y >> z ;
    return {x, y, z} ;
}

static Eigen::Vector4f parse_vec4(const std::string &s) {
    istringstream strm(s) ;
    float x, y, z, w ;
    strm >> x >> y >> z >> w;
    return {x, y, z, w} ;
}

static bool parse_boolean(const std::string &s) {
    return true ;
}

static std::string parseRequiredAttribute(xml_node &node, const char *name) {
    string a = node.attribute(name).as_string() ;
    if ( a.empty() )
        throw SDFParseException(std::string("Required attribute ") + name + " missing") ;
    return a ;
}

static std::string parseRequiredText(xml_node &node, const char *name) {
    if ( xml_node n = node.child(name) )
        return n.text().as_string() ;
    else
        throw SDFParseException(std::string("Required element ") + name + " missing") ;
}

static Isometry3f parse_pose(const std::string &text) {

    istringstream strm(text) ;

    Isometry3f tr ;
    tr.setIdentity() ;

    Vector3f t, r ;

    strm  >> t.x() >> t.y() >> t.z() ;

    tr.translate(t) ;

    strm  >> r.x() >> r.y() >> r.z() ;

    Quaternionf q ;

    q = AngleAxisf(r.z(), Vector3f::UnitZ()) * AngleAxisf(r.y(), Vector3f::UnitY()) * AngleAxisf(r.x(), Vector3f::UnitX());

    tr.rotate(q) ;

    return tr ;
}

Matrix3f parseInertia(const xml_node &node) {
    float ixx, ixy, ixz, iyy, iyz, izz ;

    if ( xml_node n = node.child("ixx" ) ) {
        ixx = n.text().as_float(1.0) ;
    }

    if ( xml_node n = node.child("ixy" ) ) {
        ixy = n.text().as_float(0.0) ;
    }

    if ( xml_node n = node.child("ixz" ) ) {
        ixz = n.text().as_float(0.0) ;
    }

    if ( xml_node n = node.child("iyy" ) ) {
        iyy = n.text().as_float(1.0) ;
    }

    if ( xml_node n = node.child("iyz" ) ) {
        iyz = n.text().as_float(0.0) ;
    }

    if ( xml_node n = node.child("izz" ) ) {
        izz = n.text().as_float(1.0) ;
    }

    Matrix3f i ;
    i << ixx, ixy, ixz,
        ixy, iyy, iyz,
        ixz, iyz, izz ;

    return i ;
}


void SDFParser::parse(const std::string &sdf_file, const std::string &world_name) {
    xml_document doc ;

    xml_parse_result result = doc.load_file(sdf_file.c_str()) ;

    if ( !result )
        throw SDFParseException(result.description()) ;

    xml_node root = doc.child("sdf") ;
    if ( root ) {
        version_ = root.attribute("version").as_string("1.11") ;
    } else
        throw SDFParseException("No SDF root element found") ;

    path_ = parentPath(sdf_file) ;

    for( xml_node &n: root.children("world") )
        if ( parseWorld(n, world_name) ) break ;

}


bool SDFParser::parseWorld(pugi::xml_node &node, const std::string &wn) {
    string name = node.attribute("name").as_string() ;
    if ( name.empty() ) return false ;
    if ( !wn.empty() && name != wn ) return false ;

    if ( xml_node gravity_node = node.child("gravity") ) {
        world_.gravity_ = parse_vec3(gravity_node.text().as_string("0 0 -9.8")) ;
    }

    for( xml_node &n: node.children("model") ) {
        SDFModel model = parseModel(n) ;
        world_.models_.emplace_back(std::move(model)) ;
    }

    return true ;

}


SDFModel SDFParser::parseModel(xml_node &node) {

    SDFModel model ;

    string name = node.attribute("name").as_string() ;
    if ( name.empty() )
        throw SDFParseException("No name provided for model") ;

    model.name_ = name ;

    if ( xml_node static_node = node.child("static") ) {
        model.is_static_ = parse_boolean(static_node.text().as_string("0")) ;
    }

    if ( xml_node pose_node = node.child("pose") ) {
        model.pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    for( xml_node &n: node.children("link") ) {
        SDFLink link = parseLink(n);
        model.links_.emplace_back(std::move(link)) ;
    }

    for( xml_node &n: node.children("joint") ) {
        SDFJoint joint = parseJoint(n);
        model.joints_.emplace_back(std::move(joint)) ;

    }

    for( xml_node &n: node.children("model") ) {
        SDFModel child = parseModel(n);
        model.children_.emplace_back(std::move(child)) ;
    }

    return model ;
}

SDFLink SDFParser::parseLink(pugi::xml_node &node) {
    SDFLink link ;
    string name = node.attribute("name").as_string() ;
    if ( name.empty() )
        throw SDFParseException("No name provided for link") ;

    link.name_ = name ;

    if ( xml_node pose_node = node.child("pose") ) {
        link.pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    if ( xml_node inertial_node = node.child("inertial") ) {
        if ( xml_node inertia_node = inertial_node.child("inertia") )
            link.inertial_.inertia_ = parseInertia(inertia_node) ;
        if ( xml_node pose_node = inertial_node.child("pose") )
            link.inertial_.pose_ = parse_pose(pose_node.text().as_string()) ;
        if ( xml_node mass_node = inertial_node.child("mass") )
            link.inertial_.mass_ = mass_node.text().as_double(1.0) ;
    }

    for( xml_node &collision_node: node.children("collision") ) {
        SDFCollision collision = parseCollision(collision_node);
        link.collisions_.emplace_back(std::move(collision)) ;
    }

    for( xml_node &visual_node: node.children("visual") ) {
        SDFVisual visual = parseVisual(visual_node) ;
        link.visuals_.emplace_back(std::move(visual)) ;
    }

    return link ;
}

SDFCollision SDFParser::parseCollision(pugi::xml_node &node) {
    SDFCollision collision ;
    collision.name_ = node.attribute("name").as_string() ;

    if ( xml_node pose_node = node.child("pose") ) {
        collision.pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    if ( xml_node geom_node = node.child("geometry") ) {
        SDFGeometry *geom = parseGeometry(geom_node) ;
        collision.geometry_.reset(geom) ;
    }

    return collision ;
}

SDFGeometry *SDFParser::parseGeometry(xml_node &node) {

    if ( xml_node n = node.child("empty") ) {
        return new SDFEmptyGeometry() ;
    } else if ( xml_node n = node.child("box") ) {
        SDFBoxGeometry *geom = new SDFBoxGeometry() ;
        if ( xml_node size_node = n.child("size") ) {
            geom->sz_ = parse_vec3(size_node.text().as_string()) ;
        }
        return geom ;

    } else if ( xml_node n = node.child("mesh") ) {
        SDFMeshGeometry *geom = new SDFMeshGeometry() ;
        if ( xml_node uri_node = n.child("uri") ) {
            geom->uri_ = uri_node.text().as_string() ;
        }

        if ( xml_node scale_node = n.child("scale") ) {
            geom->scale_ = parse_vec3(scale_node.text().as_string()) ;
        }
        return geom ;
    } else return nullptr ;
}


SDFJoint SDFParser::parseJoint(pugi::xml_node &node)
{
    SDFJoint joint ;

    joint.name_ = parseRequiredAttribute(node, "name") ;
    joint.type_ = parseRequiredAttribute(node, "type") ;
    joint.parent_ = parseRequiredText(node, "parent") ;
    joint.child_  = parseRequiredText(node, "child") ;

    if ( xml_node n = node.child("axis") ) {
        if ( xml_node xyz_node = n.child("xyz") ) {
            joint.axis_ = parse_vec3(xyz_node.text().as_string()) ;
        }

        if ( xml_node limits_node = n.child("limits") ) {
            if ( xml_node lower_node = limits_node.child("lower") ) {
                joint.lower_ = lower_node.text().as_float() ;
            }
            if ( xml_node upper_node = limits_node.child("upper") ) {
                joint.upper_ = upper_node.text().as_float() ;
            }
            if ( xml_node effort_node = limits_node.child("effort") ) {
                joint.max_effort_ = effort_node.text().as_float() ;
            }
            if ( xml_node velocity_node = limits_node.child("velocity") ) {
                joint.max_velocity_ = velocity_node.text().as_float() ;
            }
        }

        if ( xml_node dynamics_node = n.child("dynamics") ) {
            if ( xml_node damping_node = dynamics_node.child("damping") ) {
                joint.damping_ = damping_node.text().as_float() ;
            }

            if ( xml_node friction_node = dynamics_node.child("friction") ) {
                joint.friction_ = friction_node.text().as_float() ;
            }
        }



    }


    return joint ;
}

SDFVisual SDFParser::parseVisual(pugi::xml_node &node)
{
    SDFVisual visual ;
    visual.name_ = node.attribute("name").as_string() ;

    if ( xml_node pose_node = node.child("pose") ) {
        visual.pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    if ( xml_node geom_node = node.child("geometry") ) {
        SDFGeometry *geom = parseGeometry(geom_node) ;
        visual.geometry_.reset(geom) ;
    }

    if ( xml_node material_node = node.child("material") ) {
        SDFMaterial *material = new SDFMaterial ;
        if ( xml_node ambient_node = material_node.child("ambient") ) {
            material->ambient_ = parse_vec4(ambient_node.text().as_string()) ;
        }

        if ( xml_node diffuse_node = material_node.child("diffuse") ) {
            material->diffuse_ = parse_vec4(diffuse_node.text().as_string()) ;
        }

        if ( xml_node specular_node = material_node.child("specular") ) {
            material->specular_ = parse_vec4(specular_node.text().as_string()) ;
        }

        if ( xml_node emissive_node = material_node.child("emissive") ) {
            material->emissive_ = parse_vec4(emissive_node.text().as_string()) ;
        }

        visual.material_.reset(material) ;
    }

    return visual ;
}


}
