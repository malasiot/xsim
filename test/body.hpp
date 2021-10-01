#ifndef BODY_HPP
#define BODY_HPP

#include <Eigen/Geometry>

#include <memory>

class CollisionAspect ;
class VisualAspect ;
class Geometry ;

class CollisionAspect {
    CollisionAspect &setGeometry(const std::shared_ptr<Geometry> &geom) { geom = geom_ ; return *this ; }
public:
    std::shared_ptr<Geometry> geom_ ;
    float threshold_ ;

};

class VisualAspect {
    VisualAspect &setGeometry(const std::shared_ptr<Geometry> &geom) { geom = geom_ ; return *this ; }
public:
    std::shared_ptr<Geometry> geom_ ;
};

class Body {
public:

public:

    CollisionAspect &collision() { return collision_ ; }
    VisualAspect &visual() { return visual_ ; }

    Body &setMass(float m) { mass_ = m ; return *this ;}
    Body &setInertia(const Eigen::Vector3f &inertia) { inertia_ = inertia ; return *this ;}
    Body &setStatic(bool is_static) { is_static_ = is_static ; return *this ; }

    float getMass() const { return mass_ ; }
    Eigen::Vector3f getInertia() const { return inertia_ ; }
    bool isStatic() const { return is_static_ ; }

private:

    CollisionAspect collision_ ;
    VisualAspect visual_ ;

    float mass_ = 0.0f ;
    Eigen::Vector3f inertia_ = { 0, 0, 0} ;
    bool is_static_ = false;
};



#endif
