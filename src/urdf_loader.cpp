#include <xsim/urdf_loader.hpp>
#include <xviz/scene/drawable.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/material.hpp>

#include <set>
#include <random>

using namespace std ;
using namespace pugi ;

using namespace Eigen ;

namespace xsim {

URDFRobot URDFLoader::parse(const string &urdf_file) {
    URDFRobot robot ;

    xml_document doc ;

    xml_parse_result result = doc.load_file(urdf_file.c_str()) ;

    if ( !result )
        throw URDFLoadException(result.description()) ;

    if ( xml_node root = doc.child("robot") ) { // URDF
        robot.name_ = prefix_ + root.attribute("name").as_string() ;
        parseRobot(root, robot, urdf_file) ;
    } else
        throw URDFLoadException("No URDF root element found") ;


    buildTree(robot) ;

    return robot ;
}

void URDFLoader::parseRobot(const xml_node &node, URDFRobot &rb, const string &path) {

    for( const xml_node &n: node.children("material") )
        parseMaterial(n, rb, path) ;

    for( const xml_node &n: node.children("link") )
        parseLink(n, rb, path) ;

    for( const xml_node &n: node.children("joint") )
        parseJoint(n, rb) ;
}

void URDFLoader::parseLink(const xml_node &node, URDFRobot &rb, const string &path) {

    URDFLink link ;

    string name = prefix_ + node.attribute("name").as_string() ;

    if ( name.empty() )
        throw URDFLoadException("Attribute \"name\" missing from <link>") ;

    link.name_ = name ;

    for ( xml_node visual_node: node.children("visual") ) {
        Isometry3f tr ;
        tr.setIdentity() ;

        if ( xml_node origin_node = visual_node.child("origin") )
            tr = parseOrigin(origin_node) ;

        string matid ;

        if ( xml_node material_node = visual_node.child("material") ) {
            matid = parseMaterial(material_node, rb, path) ;
        }

        Vector3f scale{1, 1, 1} ;

        URDFGeometry *geom = nullptr ;

        if ( xml_node geom_node = visual_node.child("geometry") )
            geom = parseGeometry(geom_node, scale, path) ;
        else
            throw URDFLoadException("<geometry> element is missing from <visual>") ;

        tr.linear() *= scale.asDiagonal() ;
        geom->origin_ = tr ;

        if ( !matid.empty() ) {
            geom->material_ref_ = matid ;
        }

        link.visual_geoms_.emplace_back(geom) ;
    }

    for ( xml_node collision_node: node.children("collision") ) {
        Isometry3f tr ;
        tr.setIdentity() ;

        if ( xml_node origin_node = collision_node.child("origin") )
            tr = parseOrigin(origin_node) ;

        if ( xml_node pose_node = collision_node.child("pose") )
            tr = parsePose(pose_node) ;

        Vector3f scale{1, 1, 1} ;

        URDFGeometry *geom ;

        if ( xml_node geom_node = collision_node.child("geometry") )
            geom = parseGeometry(geom_node, scale, path) ;
        else
            throw URDFLoadException("<geometry> element is missing from <collision>") ;

        tr.linear() *= scale.asDiagonal() ;
        geom->origin_ = tr ;

        link.collision_geoms_.emplace_back(geom) ;
    }

    if ( xml_node inertial_node = node.child("inertial") ) {

        URDFInertial inr ;

        if ( xml_node origin_node = inertial_node.child("origin") )
            inr.origin_ = parseOrigin(origin_node) ;

        if ( xml_node pose_node = inertial_node.child("pose") )
            inr.origin_ = parsePose(pose_node) ;

        if ( xml_node mass_node = inertial_node.child("mass") ) {
            inr.mass_ = mass_node.attribute("value").as_float() ;
        }

        if ( xml_node inertia_node = inertial_node.child("inertia") ) {
            Matrix3f im = parseInertia(inertia_node) ;
            inr.inertia_ = im ;
        }

        link.inertial_.reset(new URDFInertial(std::move(inr)));
    }


    if ( xml_node contact_node = node.child("contact") ) {
        if ( xml_node damping_node = contact_node.child("damping") ) {
            link.contact_info_.damping_ = damping_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_DAMPING ;
        }

        if ( xml_node stiffness_node = contact_node.child("stiffness") ) {
            link.contact_info_.stiffness_ = stiffness_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_STIFFNESS ;
        }

        if ( xml_node anchor_node = contact_node.child("friction_anchor") ) {
            link.contact_info_.flags_ |= URDFContact::HAS_FRICTION_ANCHOR ;
        }

        if ( xml_node restitution_node = contact_node.child("restitution") ) {
            link.contact_info_.restitution_ = restitution_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_RESTITUTION ;
        }

        if ( xml_node rolling_friction_node = contact_node.child("rolling_friction") ) {
            link.contact_info_.rolling_friction_ = rolling_friction_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_ROLLING_FRICTION ;
        }

        if ( xml_node spinning_friction_node = contact_node.child("spinning_friction") ) {
            link.contact_info_.spinning_friction_ = spinning_friction_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_SPINNING_FRICTION ;
        }

        if ( xml_node lateral_friction_node = contact_node.child("lateral_friction") ) {
            link.contact_info_.lateral_friction_ = lateral_friction_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_LATERAL_FRICTION ;
        }

        if ( xml_node inertia_scaling_node = contact_node.child("inertia_scaling") ) {
            link.contact_info_.inertia_scaling_ = inertia_scaling_node.attribute("value").as_float(0.f);
            link.contact_info_.flags_ |= URDFContact::HAS_INERTIA_SCALING ;
        }

    }

    rb.links_.insert(std::make_pair(name, std::move(link))) ;
}

static Vector3f parse_vec3(const std::string &s) {
    istringstream strm(s) ;
    float x, y, z ;
    strm >> x >> y >> z ;
    return {x, y, z} ;
}

static Vector4f parse_vec4(const std::string &s) {
    istringstream strm(s) ;
    float x, y, z, w ;
    strm >> x >> y >> z >> w;
    return {x, y, z, w} ;
}


void URDFLoader::parseJoint(const xml_node &node, URDFRobot &rb) {
    string name = prefix_ + node.attribute("name").as_string() ;
    string type = node.attribute("type").as_string() ;

    if ( name.empty() )
        throw URDFLoadException("<joint> is missing \"name\" attribute") ;

    URDFJoint j ;

    j.type_ = type ;
    j.name_ = name ;

    if ( xml_node origin_node = node.child("origin") )
        j.origin_ = parseOrigin(origin_node) ;

    if ( xml_node parent_node = node.child("parent") ) {
        string link_name = parent_node.attribute("link").as_string() ;
        j.parent_ = link_name ;
    }

    if ( xml_node child_node = node.child("child") ) {
        string link_name = child_node.attribute("link").as_string() ;
        j.child_ = link_name ;
    }


    if ( xml_node axis_node = node.child("axis") ) {
        string axis_str = axis_node.attribute("xyz").as_string() ;
        j.axis_ = parse_vec3(axis_str) ;
    }

    if ( xml_node limits_node = node.child("limit") ) {
        j.lower_ = limits_node.attribute("lower").as_float(0) ;
        j.upper_ = limits_node.attribute("upper").as_float(0) ;
        j.effort_ = limits_node.attribute("effort").as_float(0) ;
        j.velocity_ = limits_node.attribute("velocity").as_float(0) ;
    }

    if ( xml_node mimic_node = node.child("mimic") ) {
        j.mimic_joint_ = mimic_node.attribute("joint").as_string() ;
        j.mimic_offset_ = mimic_node.attribute("offset").as_float(0.0) ;
        j.mimic_multiplier_ = mimic_node.attribute("multiplier").as_float(1.0) ;
    }

    if ( xml_node dynamics_node = node.child("dynamics") ) {
        j.friction_ = dynamics_node.attribute("friction").as_float() ;
        j.damping_ = dynamics_node.attribute("damping").as_float() ;
    }

    rb.joints_.emplace(name, j) ;

}

bool URDFLoader::buildTree(URDFRobot &rb) {
    map<string, string> parent_link_tree ;

    for( auto &jp: rb.joints_ ) {
        URDFJoint &j = jp.second ;

        if ( j.child_.empty() || j.parent_.empty() ) return false ;

        URDFLink *child_link = rb.get_link(j.child_) ;
        URDFLink *parent_link = rb.get_link(j.parent_) ;

        if ( child_link == nullptr || parent_link == nullptr ) return false ;

        child_link->parent_link_ = parent_link ;
        child_link->parent_joint_ = &j ;
        parent_link->child_joints_.push_back(&j) ;
        parent_link->child_links_.push_back(child_link) ;
        parent_link_tree[child_link->name_] = j.parent_;
    }

    // find root

    rb.root_ = nullptr ;

    for ( const auto &lp: rb.links_ ) {
        const string &name = lp.first ;
        const URDFLink &l = lp.second ;
        auto it = parent_link_tree.find(name) ;
        if ( it == parent_link_tree.end() ) { // no parent thus root
            if ( rb.root_ == nullptr ) {
                URDFLink *l = rb.get_link(name) ;
                rb.root_ = l ;
            }
            else return false ;
        }
    }

    if ( rb.root_ == nullptr ) return false ;


    return true ;
}

Isometry3f URDFLoader::parseOrigin(const xml_node &node) {

    string xyz = node.attribute("xyz").as_string() ;
    string rpy = node.attribute("rpy").as_string() ;

    Isometry3f tr ;
    tr.setIdentity() ;

    if ( !xyz.empty() ) {
        Vector3f t = parse_vec3(xyz) ;
        tr.translate(t) ;
    }

    if ( !rpy.empty() ) {
        Vector3f r = parse_vec3(rpy) ;
        Quaternionf q ;

        q = AngleAxisf(r.z(), Vector3f::UnitZ()) * AngleAxisf(r.y(), Vector3f::UnitY()) * AngleAxisf(r.x(), Vector3f::UnitX());

        tr.rotate(q) ;
    }

    return tr ;
}

Isometry3f URDFLoader::parsePose(const xml_node &node) {

    string text(node.text().as_string()) ;

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

Matrix3f URDFLoader::parseInertia(const xml_node &node) {
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

static std::string parentPath(const std::string &fpath) {
    size_t pos = fpath.find_last_of('/') ;
    return fpath.substr(0, pos) ;
}

string URDFLoader::resolveUri(const std::string &uri, const std::string &path) {

    string rpath ;

    if ( uri.substr(0, 10) == "package://" ) {
        size_t pos = uri.find_first_of('/', 10) ;
        if ( pos == string::npos ) return rpath ;
        string package_str = uri.substr(10, pos-10) ;
        string package_subpath = uri.substr(pos+1) ;

        auto it = package_map_.find(package_str) ;
        if ( it == package_map_.end() ) return rpath ;
        rpath = it->second + '/' + package_subpath ;
    } else {
        rpath = parentPath(path) + '/' + uri ;
    }

    return rpath;

}

URDFGeometry *URDFLoader::parseGeometry(const xml_node &node, Vector3f &sc, const string &path) {

    if ( xml_node mesh_node = node.child("mesh") ) {

        URDFMeshGeometry *geom = new URDFMeshGeometry() ;

        string uri = mesh_node.attribute("filename").as_string() ;

        string rpath = resolveUri(uri, path) ;

        if ( rpath.empty() ) return nullptr ;

        geom->path_ = rpath ;

        string scale = mesh_node.attribute("scale").as_string() ;

        if ( !scale.empty() ) {
            sc = parse_vec3(scale) ;
        }

        geom->scale_ = sc ;

        return geom ;
    } else if ( xml_node box_node = node.child("box") ) {
        string sz = box_node.attribute("size").as_string() ;
        Vector3f hs = parse_vec3(sz)/2 ;

        URDFBoxGeometry *geom = new URDFBoxGeometry(hs) ;

        return geom ;
    } else if ( xml_node cylinder_node = node.child("cylinder") ) {
        float radius = cylinder_node.attribute("radius").as_float(0) ;
        float length = cylinder_node.attribute("length").as_float(0) ;

        URDFCylinderGeometry *geom = new URDFCylinderGeometry(radius, length) ;

        return geom ;
    } else if ( xml_node cylinder_node = node.child("sphere") ) {
        float radius = cylinder_node.attribute("radius").as_float(0) ;

        URDFSphereGeometry *geom = new URDFSphereGeometry(radius) ;

        return geom ;
    }

    return nullptr ;
}

std::string URDFLoader::parseMaterial(const xml_node &node, URDFRobot &rb, const string &path)
{
    string name = node.attribute("name").as_string() ;

    if ( xml_node clr_node = node.child("color") ) {
        string rgba = clr_node.attribute("rgba").as_string() ;
        if ( !rgba.empty() ) {
            Vector4f clr = parse_vec4(rgba) ;

            URDFMaterial *mat = new URDFMaterial ;
            mat->diffuse_color_ = clr.block<3, 1>(0, 0) ;
            std::shared_ptr<URDFMaterial> mat_ptr(mat) ;
            rb.materials_.emplace(name, mat_ptr) ;
            return name ;
        }
    }

    if ( xml_node texture_node = node.child("texture") ) {
        string uri = texture_node.attribute("filename").as_string() ;

        string rpath = resolveUri(uri, path) ;
        if ( !rpath.empty() ) {

            URDFMaterial *mat = new URDFMaterial ;
            mat->texture_path_ = rpath ;

            std::shared_ptr<URDFMaterial> mat_ptr(mat) ;
            rb.materials_.emplace(name, mat_ptr) ;
            return name;
        }
    }

    return name ;
}

}


