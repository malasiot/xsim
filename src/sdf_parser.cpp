#include <xsim/sdf_parser.hpp>
#include <xsim/world.hpp>

using namespace std ;
using namespace pugi ;
using namespace Eigen ;

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
    float ixx = node.attribute("ixx").as_float(1) ;
    float ixy = node.attribute("ixy").as_float(0) ;
    float ixz = node.attribute("ixz").as_float(0) ;
    float iyy = node.attribute("iyy").as_float(1) ;
    float iyz = node.attribute("iyz").as_float(0) ;
    float izz = node.attribute("izz").as_float(1) ;

    Matrix3f i ;
    i << ixx, ixy, ixz,
        ixy, iyy, iyz,
        ixz, iyz, izz ;

    return i ;
}


SDFWorld::SDFWorld() {

}

void SDFWorld::parse(const std::string &sdf_file, const std::string &world_name) {


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


bool SDFWorld::parseWorld(pugi::xml_node &node, const std::string &wn) {
    string name = node.attribute("name").as_string() ;
    if ( name.empty() ) return false ;
    if ( !wn.empty() && name != wn ) return false ;

    if ( xml_node gravity_node = node.child("gravity") ) {
        gravity_ = parse_vec3(gravity_node.text().as_string("0 0 -9.8")) ;
    }

    for( xml_node &n: node.children("model") ) {
        SDFModel model ;

        model.parse(node, path_) ;

        models_.emplace_back(std::move(model)) ;
    }

    return true ;

}


void SDFModel::parse(xml_node &node, const std::string &path) {

    string name = node.attribute("name").as_string() ;
    if ( name.empty() )
        throw SDFParseException("No name provided for model") ;

    name_ = name ;

    if ( xml_node static_node = node.child("static") ) {
        is_static_ = parse_boolean(static_node.text().as_string("0")) ;
    }

    if ( xml_node pose_node = node.child("pose") ) {
        pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    for( xml_node &n: node.children("link") ) {
        SDFLink link ;
        link.parse(n, path) ;
        links_.emplace_back(std::move(link)) ;
    }

    for( xml_node &n: node.children("joint") ) {
        SDFJoint joint ;
        joint.parse(n) ;
        joints_.emplace_back(std::move(joint)) ;

    }

    for( xml_node &n: node.children("model") ) {
        SDFModel child ;
        child.parse(n, path) ;
        children_.emplace_back(std::move(child)) ;
    }
}

void SDFLink::parse(pugi::xml_node &node, const std::string &path) {
    string name = node.attribute("name").as_string() ;
    if ( name.empty() )
        throw SDFParseException("No name provided for link") ;

    name_ = name ;

    if ( xml_node pose_node = node.child("pose") ) {
        pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    if ( xml_node inertial_node = node.child("inertial") ) {
        if ( xml_node inertia_node = inertial_node.child("inertia") )
            inertial_.inertia_ = parseInertia(inertia_node) ;
        if ( xml_node pose_node = inertial_node.child("pose") )
            inertial_.pose_ = parse_pose(pose_node.text().as_string()) ;
        if ( xml_node mass_node = inertial_node.child("mass") )
            inertial_.mass_ = mass_node.text().as_double(1.0) ;
    }

    for( xml_node &collision_node: node.children("collision") ) {
        SDFCollision collision ;

        collision.parse(collision_node, path) ;
        collisions_.emplace_back(std::move(collision)) ;
    }

    for( xml_node &visual_node: node.children("visual") ) {
        SDFVisual visual ;

        visual.parse(visual_node, path) ;
        visuals_.emplace_back(std::move(visual)) ;
    }
}

void SDFCollision::parse(pugi::xml_node &node, const std::string &path) {
    name_ = node.attribute("name").as_string() ;

    if ( xml_node pose_node = node.child("pose") ) {
        pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    if ( xml_node geom_node = node.child("geometry") ) {
        SDFGeometry *geom = SDFGeometry::parse(geom_node) ;
        geometry_.reset(geom) ;
    }

}

SDFGeometry *SDFGeometry::parse(xml_node &node) {
    if ( xml_node n = node.child("empty") ) {
        return new SDFEmptyGeometry() ;
    } else if ( xml_node n = node.child("box") ) {
        SDFBoxGeometry *geom = new SDFBoxGeometry() ;
        geom->parse(n) ;
        return geom ;

    } else if ( xml_node n = node.child("mesh") ) {
        SDFMeshGeometry *geom = new SDFMeshGeometry() ;
        geom->parse(n) ;
        return geom ;
    } else return nullptr ;


}


void SDFJoint::parse(pugi::xml_node &node)
{
    string name = node.attribute("name").as_string() ;
    if ( name.empty() )
        throw SDFParseException("No name provided for joint") ;

    name_ = name ;




}

void SDFMeshGeometry::parse(pugi::xml_node &node)
{
    if ( xml_node uri_node = node.child("uri") ) {
        uri_ = uri_node.text().as_string() ;
    }

    if ( xml_node scale_node = node.child("scale") ) {
        scale_ = parse_vec3(scale_node.text().as_string()) ;
    }

}

void SDFBoxGeometry::parse(pugi::xml_node &node)
{
    if ( xml_node size_node = node.child("size") ) {
        sz_ = parse_vec3(size_node.text().as_string()) ;
    }
}

void SDFVisual::parse(pugi::xml_node &node, const std::string &path)
{
    name_ = node.attribute("name").as_string() ;

    if ( xml_node pose_node = node.child("pose") ) {
        pose_ = parse_pose(pose_node.text().as_string()) ;
    }

    if ( xml_node geom_node = node.child("geometry") ) {
        SDFGeometry *geom = SDFGeometry::parse(geom_node) ;
        geometry_.reset(geom) ;
    }

    if ( xml_node material_node = node.child("material") ) {
        material_.reset(new SDFMaterial()) ;
        material_->parse(material_node) ;
    }
}

void SDFMaterial::parse(pugi::xml_node &node)
{
    if ( xml_node ambient_node = node.child("ambient") ) {
        ambient_ = parse_vec4(ambient_node.text().as_string()) ;
    }

    if ( xml_node diffuse_node = node.child("diffuse") ) {
        diffuse_ = parse_vec4(diffuse_node.text().as_string()) ;
    }

    if ( xml_node specular_node = node.child("specular") ) {
        specular_ = parse_vec4(specular_node.text().as_string()) ;
    }

    if ( xml_node emissive_node = node.child("emissive") ) {
        emissive_ = parse_vec4(emissive_node.text().as_string()) ;
    }
}

}
