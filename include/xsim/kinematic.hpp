#ifndef XVIZ_PHYSICS_KINEMATIC_HPP
#define XVIZ_PHYSICS_KINEMATIC_HPP

#include <memory>

#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>

#include <xsim/collision.hpp>
#include <xsim/convert.hpp>
#include <xviz/robot/urdf_robot.hpp>
#include <xsim/rigid_body.hpp>

namespace xviz {

class ArticulatedCollisionShape: public GroupCollisionShape {
public :

    ArticulatedCollisionShape(const URDFRobot &robot) {
        create(robot) ;
    }

private:

    struct Link {
        uint col_shape_index_ ;
        Eigen::Isometry3f origin_ ;
    };

    struct Joint {
        Eigen::Isometry3f origin_ ;
        Eigen::Vector3f axis_ ;
        float lower_limit_, upper_limit_ ;
        std::string type_, mimic_ ;
        Link *parent_, *child_ ;
        float position_ ;
    };

    std::map<std::string, Link> links_ ;
    std::map<std::string, Joint> joints_ ;

    void create(const URDFRobot &robot) ;
    static CollisionShapePtr makeCollisionShape(const URDFGeometry *geom) ;
};

} // namespace xviz


#endif
