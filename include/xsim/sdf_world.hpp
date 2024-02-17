#pragma once

#include <string>
#include <stdexcept>
#include <memory>

#include <Eigen/Geometry>

namespace xsim {

class SDFModel ;

class SDFWorld {
public:
    SDFWorld() = default;

    static SDFWorld load(const std::string &fpath, const std::string &wn = {}) ;

private:

    friend class SDFParser ;
    friend class PhysicsWorld ;

    std::string name_ ;
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

    std::string name_ ;
    Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity() ;
    bool is_static_ = false;

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
};

struct SDFMeshGeometry: public SDFGeometry {
    std::string uri_ ;
    Eigen::Vector3f scale_ ;
};

struct SDFEmptyGeometry: public SDFGeometry {
};

struct SDFBoxGeometry: public SDFGeometry {
    Eigen::Vector3f sz_ ;
};

struct SDFMaterial {
    Eigen::Vector4f ambient_, diffuse_, specular_, emissive_ ;
};

struct SDFCollision {

    SDFCollision() = default ;
    SDFCollision(const SDFCollision&) = delete;
    SDFCollision(SDFCollision&&) = default;

    std::string name_ ;
    Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity() ;
    std::unique_ptr<SDFGeometry> geometry_ ;
};


struct SDFVisual {

    SDFVisual() = default ;
    SDFVisual(const SDFVisual&) = delete;
    SDFVisual(SDFVisual&&) = default;

    std::string name_ ;
    Eigen::Isometry3f pose_ ;

    std::unique_ptr<SDFGeometry> geometry_ ;
    std::unique_ptr<SDFMaterial> material_ ;
};

struct SDFLink {

    SDFLink() = default ;
    SDFLink(const SDFLink&) = delete;
    SDFLink(SDFLink&&) = default;

    std::string name_ ;
    SDFInertial inertial_ ;
    Eigen::Isometry3f pose_ ;
    std::vector<SDFCollision> collisions_ ;
    std::vector<SDFVisual> visuals_ ;
};

struct SDFJoint {
    std::string name_, type_, parent_, child_ ;
    Eigen::Vector3f axis_{0, 0, 1} ;
    float damping_ = 0, friction_ = 0 ;
    float lower_ = -std::numeric_limits<float>::max(),
          upper_ = -std::numeric_limits<float>::max(),
          max_effort_ = std::numeric_limits<float>::max(),
          max_velocity_ = std::numeric_limits<float>::max(),
          max_stiffness_ = 10000 ;
};

class SDFParseException: public std::runtime_error {
public:
    SDFParseException(const std::string &what): std::runtime_error(what) {}
};

}
