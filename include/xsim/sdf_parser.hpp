#pragma once

#include <string>
#include <stdexcept>
#include <memory>

#include <pugi/pugixml.hpp>
#include <Eigen/Geometry>

namespace xsim {

class PhysicsWorld ;

class SDFModel ;

class SDFWorld {
public:
    SDFWorld() ;
    void parse(const std::string &fpath, const std::string &world_name = {}) ;

private:

    bool parseWorld(pugi::xml_node &node, const std::string &wname) ;
    void parseModel(pugi::xml_node &node) ;

private:

    std::string version_, name_ ;
    std::string path_ ;
    Eigen::Vector3f gravity_{0, 0, -9.8};
    std::vector<SDFModel> models_ ;
};

struct SDFLink ;
struct SDFJoint ;

struct SDFModel {
public:
    SDFModel() = default ;
    SDFModel(const SDFModel&) = delete;
    SDFModel(SDFModel&&) = default;

    void parse(pugi::xml_node &node, const std::string &path) ;

    std::string name_ ;
    Eigen::Isometry3f pose_ ;
    bool is_static_ ;

    std::vector<SDFModel> children_ ;
    std::vector<SDFLink> links_ ;
    std::vector<SDFJoint> joints_ ;
};

struct SDFInertial {
    Eigen::Isometry3f pose_ ;
    double mass_ ;
    Eigen::Matrix3f inertia_ ;
    bool auto_ ;
};

struct SDFGeometry {
    virtual ~SDFGeometry() {}

    static SDFGeometry *parse(pugi::xml_node &node) ;
};

struct SDFMeshGeometry: public SDFGeometry {
    std::string uri_ ;
    Eigen::Vector3f scale_ ;

    void parse(pugi::xml_node &node) ;
};

struct SDFEmptyGeometry: public SDFGeometry {
};

struct SDFBoxGeometry: public SDFGeometry {
    void parse(pugi::xml_node &node) ;

    Eigen::Vector3f sz_ ;
};

struct SDFMaterial {
    void parse(pugi::xml_node &node) ;

    Eigen::Vector4f ambient_, diffuse_, specular_, emissive_ ;
};

struct SDFCollision {

    SDFCollision() = default ;
    SDFCollision(const SDFCollision&) = delete;
    SDFCollision(SDFCollision&&) = default;

    std::string name_ ;
    Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity() ;
    std::unique_ptr<SDFGeometry> geometry_ ;

    void parse(pugi::xml_node &node, const std::string &path) ;
};


struct SDFVisual {

    SDFVisual() = default ;
    SDFVisual(const SDFVisual&) = delete;
    SDFVisual(SDFVisual&&) = default;

    std::string name_ ;
    Eigen::Isometry3f pose_ ;

    std::unique_ptr<SDFGeometry> geometry_ ;
    std::unique_ptr<SDFMaterial> material_ ;

    void parse(pugi::xml_node &node, const std::string &path) ;
};

struct SDFLink {

    SDFLink() = default ;
    SDFLink(const SDFLink&) = delete;
    SDFLink(SDFLink&&) = default;

    void parse(pugi::xml_node &node, const std::string &path) ;

    std::string name_ ;
    SDFInertial inertial_ ;
    Eigen::Isometry3f pose_ ;
    std::vector<SDFCollision> collisions_ ;
    std::vector<SDFVisual> visuals_ ;
};

struct SDFJoint {

    void parse(pugi::xml_node &node) ;

    std::string name_ ;
};
}
