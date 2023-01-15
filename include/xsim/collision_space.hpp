#ifndef COLLISION_SPACE_HPP
#define COLLISION_SPACE_HPP

#include <xsim/collision.hpp>
#include <xsim/urdf_robot.hpp>

namespace xsim {
class CollisionDispatcher ;
struct CollisionFilterCallback ;

struct CollisionObjectPrivate ;

class CollisionSpace
{
public:
    CollisionSpace();
    ~CollisionSpace() ;

    void addCollisionObject(const std::string &name, const CollisionShapePtr &shape, const Eigen::Isometry3f &wtr) ;
    void addRobot(const URDFRobot &rb, float collision_margin = 0.04, bool disable_self_collisions = true);

    bool hasCollision();
    void disableCollision(const std::string &l1, const std::string &l2) ;
    void updateObjectTransform(const std::string &name, const Eigen::Isometry3f &tr) ;

    void updateObjectTransforms(const std::map<std::string, Eigen::Isometry3f> &trs);
    void enableCollision(const std::string &l1, const std::string &l2);
private:
    std::unique_ptr<btBroadphaseInterface> broadphase_ ;
    std::unique_ptr<btCollisionDispatcher> dispatcher_ ;
    std::unique_ptr<btDefaultCollisionConfiguration> config_ ;
    std::unique_ptr<btCollisionWorld> world_ ;
    std::unique_ptr<CollisionFilterCallback> filter_cb_ ;

    std::map<std::string, std::shared_ptr<CollisionObjectPrivate>> objects_ ;

    CollisionShapePtr makeCollisionShape(const URDFGeometry *geom);
};

} // namespace xsim

#endif // COLLISIONSPACE_HPP
