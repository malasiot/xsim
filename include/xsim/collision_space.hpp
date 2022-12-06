#ifndef COLLISION_SPACE_HPP
#define COLLISION_SPACE_HPP

#include <xsim/collision.hpp>
#include <xviz/robot/urdf_robot.hpp>

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
    void addRobot(const xviz::URDFRobot &rb, bool self_collisions = false);
    bool hasCollision();
    void disableCollision(const std::string &l1, const std::string &l2) ;

private:
    std::unique_ptr<btBroadphaseInterface> broadphase_ ;
    std::unique_ptr<btCollisionDispatcher> dispatcher_ ;
    std::unique_ptr<btDefaultCollisionConfiguration> config_ ;
    std::unique_ptr<btCollisionWorld> world_ ;
    std::unique_ptr<CollisionFilterCallback> cb_ ;
    std::vector<std::shared_ptr<CollisionObjectPrivate>> objects_ ;

    CollisionShapePtr makeCollisionShape(const xviz::URDFGeometry *geom);
};

} // namespace xsim

#endif // COLLISIONSPACE_HPP
