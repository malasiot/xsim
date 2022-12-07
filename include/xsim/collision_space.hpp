#ifndef COLLISION_SPACE_HPP
#define COLLISION_SPACE_HPP

#include <xsim/collision.hpp>

namespace xsim {
class CollisionDispatcher ;
class CollisionFilterCallback ;
class CollisionData ;

class CollisionSpace
{
public:
    CollisionSpace();
    ~CollisionSpace() ;

    void addCollisionShape(const CollisionShapePtr &shape, const Eigen::Isometry3f &tr) ;
    bool hasCollision();

private:
    std::unique_ptr<CollisionDispatcher> dispatcher_ ;
    std::unique_ptr<btDefaultCollisionConfiguration> config_ ;
    std::unique_ptr<btCollisionWorld> world_ ;
    std::unique_ptr<CollisionFilterCallback> cb_ ;
    std::unique_ptr<btBroadphaseInterface> broadphase_ ;
    std::vector<CollisionShapePtr> shapes_ ;
    std::vector<std::unique_ptr<CollisionData>> user_data_ ;
};

} // namespace xsim

#endif // COLLISIONSPACE_HPP
