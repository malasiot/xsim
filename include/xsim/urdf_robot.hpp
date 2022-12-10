#ifndef XSIM_URDF_ROBOT_HPP
#define XSIM_URDF_ROBOT_HPP

#include <map>
#include <vector>
#include <Eigen/Geometry>
#include <memory>
#include <optional>

namespace xsim {

struct URDFJoint {
    std::string parent_, child_, type_, mimic_joint_, name_ ;
    Eigen::Vector3f axis_ ;
    Eigen::Isometry3f origin_ = Eigen::Isometry3f::Identity() ;
    float upper_, lower_, mimic_offset_, mimic_multiplier_, effort_, velocity_ ;
    float damping_, friction_ ;
    float position_ = 0;

    Eigen::Isometry3f getMatrix() const ;
};

struct URDFMaterial ;

struct URDFGeometry {
    virtual ~URDFGeometry() {}
    std::string material_ref_ ;
    Eigen::Isometry3f origin_ = Eigen::Isometry3f::Identity() ;
} ;

struct URDFMeshGeometry: public URDFGeometry {
    std::string path_ ;
    Eigen::Vector3f scale_ ;
};

struct URDFBoxGeometry: public URDFGeometry {
    URDFBoxGeometry(const Eigen::Vector3f &he): he_(he) {}
    Eigen::Vector3f he_ ;
};

struct URDFSphereGeometry: public URDFGeometry {
    URDFSphereGeometry(float r): radius_(r) {}
    float radius_ ;
};

struct URDFCylinderGeometry: public URDFGeometry {
    URDFCylinderGeometry(float radius, float height): radius_(radius), height_(height) {}
    float radius_, height_ ;
};

struct URDFMaterial {
    std::string texture_path_ ;
    Eigen::Vector3f diffuse_color_, specular_color_{1, 1, 1} ;
};

struct URDFInertial {
   Eigen::Isometry3f origin_ = Eigen::Isometry3f::Identity() ;
   Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity() ;
   float mass_ = 0.0f ;
   std::optional<Eigen::Matrix3f> inertia_ ;
} ;

struct URDFContact {
    enum Flags { HAS_INERTIA_SCALING = 0x1,
                 HAS_LATERAL_FRICTION = 0x2,
                 HAS_ROLLING_FRICTION = 0x4,
                 HAS_SPINNING_FRICTION = 0x8,
                 HAS_RESTITUTION = 0x10,
                 HAS_DAMPING = 0x20,
                 HAS_FRICTION_ANCHOR = 0x40,
                 HAS_STIFFNESS = 0x80 };

    float inertia_scaling_, lateral_friction_,
        rolling_friction_, restitution_, spinning_friction_, damping_, stiffness_ ;
    unsigned int flags_ = 0;
};

struct URDFLink {
    std::string name_ ;

    std::unique_ptr<URDFInertial> inertial_ ;
    std::vector<std::unique_ptr<URDFGeometry>> visual_geoms_, collision_geoms_ ;

    std::vector<URDFLink *> child_links_ ;
    std::vector<URDFJoint *> child_joints_ ;
    URDFLink *parent_link_ = nullptr ;
    URDFJoint *parent_joint_ = nullptr ;
    URDFContact contact_info_ ;
};

struct URDFRobot {

    // loads URDF models
    static URDFRobot load(const std::string &fname, const std::string &name_prefix = {}, const std::map<std::string, std::string> package_map = {}) ;

    const URDFLink *getLink(const std::string &name) const  ;
    const URDFJoint *findJoint(const std::string &name) const  ;

    float setJointPosition(const std::string &jname, float pos) ;
    void setWorldTransform(const Eigen::Isometry3f &global) { global_ = global ; }
    const Eigen::Isometry3f &worldTransform() const { return global_ ; }

    std::string name_ ;
    std::map<std::string, URDFJoint> joints_ ;
    std::map<std::string, URDFLink> links_ ;
    std::map<std::string, std::shared_ptr<URDFMaterial>> materials_ ;
    URDFLink *root_ ;
    Eigen::Isometry3f global_ = Eigen::Isometry3f::Identity();

public:

    void computeLinkTransforms(std::map<std::string, Eigen::Isometry3f> &transforms) const ;
    void computeLinkTransformRecursive(std::map<std::string, Eigen::Isometry3f> &transforms,
                                       const URDFLink *link, const Eigen::Isometry3f &parent) const;

private:

    friend class URDFLoader ;

    URDFLink *get_link(const std::string &name) ;
    URDFJoint *get_joint(const std::string &name) ;

};

class URDFLoadException: public std::runtime_error {
public:
    URDFLoadException(const std::string &msg): std::runtime_error(msg) {}
};

} // namespace xviz


#endif
