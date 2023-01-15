#include <xsim/goal_region.hpp>

#include <ompl/util/RandomNumbers.h>
#include <iostream>
using namespace Eigen ;
using namespace std ;

namespace xsim {

SimplePoseGoal::SimplePoseGoal(const Eigen::Isometry3f &pose): pose_(pose) {
    xyz_tol_ << default_xyz_tol_, default_xyz_tol_, default_xyz_tol_ ;
    rpy_tol_ << default_rpy_tol_, default_rpy_tol_, default_rpy_tol_ ;
    computeBounds() ;
}

void SimplePoseGoal::computeBounds() {

    Vector3f euler = pose_.linear().eulerAngles(0, 1, 2) ;
    Vector3f t = pose_.translation() ;

    Vector3f tmin = t - xyz_tol_ ;
    Vector3f tmax = t + xyz_tol_ ;

    Vector3f rmin, rmax ;
    rmin.x() = euler.x() - rpy_tol_.x() ;
    rmax.x() = euler.x() + rpy_tol_.x() ;

    rmin.y() = euler.y() - rpy_tol_.y() ;
    rmax.y() = euler.y() + rpy_tol_.y() ;

    rmin.z() = euler.z() - rpy_tol_.z() ;
    rmax.z() = euler.z() + rpy_tol_.z() ;

    lower_bounds_[0] = tmin.x() ; upper_bounds_[0] = tmax.x() ;
    lower_bounds_[1] = tmin.y() ; upper_bounds_[1] = tmax.y() ;
    lower_bounds_[2] = tmin.z() ; upper_bounds_[2] = tmax.z() ;
    lower_bounds_[3] = rmin.x() ; upper_bounds_[3] = rmax.x() ;
    lower_bounds_[4] = rmin.y() ; upper_bounds_[4] = rmax.y() ;
    lower_bounds_[5] = rmin.z() ; upper_bounds_[5] = rmax.z() ;
}

ompl::RNG g_rng ;

void SimplePoseGoal::sample(vector<double> &xyz_rpy) const {
    double X = g_rng.uniformReal(lower_bounds_[0], upper_bounds_[0]) ;
    double Y = g_rng.uniformReal(lower_bounds_[1], upper_bounds_[1]) ;
    double Z = g_rng.uniformReal(lower_bounds_[2], upper_bounds_[2]) ;
    double r = g_rng.uniformReal(lower_bounds_[3], upper_bounds_[3]) ;
    double p = g_rng.uniformReal(lower_bounds_[4], upper_bounds_[4]) ;
    double y = g_rng.uniformReal(lower_bounds_[5], upper_bounds_[5]) ;


    xyz_rpy.push_back(X) ;
    xyz_rpy.push_back(Y) ;
    xyz_rpy.push_back(Z) ;
    xyz_rpy.push_back(r) ;
    xyz_rpy.push_back(p) ;
    xyz_rpy.push_back(y) ;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SimpleShapeRegion::computeOrientationBounds(const Vector3f &rpy) {
    lower_bounds_[3] = rpy.x() - rpy_tol_.x() ;
    lower_bounds_[4] = rpy.y() - rpy_tol_.y() ;
    lower_bounds_[5] = rpy.z() - rpy_tol_.z() ;
    upper_bounds_[3] = rpy.x() + rpy_tol_.x() ;
    upper_bounds_[4] = rpy.y() + rpy_tol_.y() ;
    upper_bounds_[5] = rpy.z() + rpy_tol_.z() ;
}

Vector3f SimpleShapeRegion::sampleOrientation() const {
    double roll  = g_rng.uniformReal(lower_bounds_[3], upper_bounds_[3]) ;
    double pitch = g_rng.uniformReal(lower_bounds_[4], upper_bounds_[4]) ;
    double yaw   = g_rng.uniformReal(lower_bounds_[5], upper_bounds_[5]) ;

    return Vector3f{roll, pitch, yaw};
}

BoxShapedRegion::BoxShapedRegion(const Vector3f &c, const Vector3f &hsz, const Vector3f &rpy, const Vector3f &rpy_tol):
    SimpleShapeRegion(rpy_tol), orig_(c), rpy_(rpy), hsz_(hsz) {
    computeOrientationBounds(rpy);
    lower_bounds_[0] = c.x() - hsz.x() ;
    lower_bounds_[1] = c.y() - hsz.y() ;
    lower_bounds_[2] = c.z() - hsz.z() ;
    upper_bounds_[0] = c.x() + hsz.x() ;
    upper_bounds_[1] = c.y() + hsz.y() ;
    upper_bounds_[2] = c.z() + hsz.z() ;
}

void BoxShapedRegion::sample(std::vector<double> &xyz_rpy) const {
    Vector3f rpy = sampleOrientation() ;
    double X  = g_rng.uniformReal(lower_bounds_[0], upper_bounds_[0]) ;
    double Y  = g_rng.uniformReal(lower_bounds_[1], upper_bounds_[1]) ;
    double Z   = g_rng.uniformReal(lower_bounds_[2], upper_bounds_[2]) ;

    xyz_rpy.push_back(X) ;
    xyz_rpy.push_back(Y) ;
    xyz_rpy.push_back(Z) ;
    xyz_rpy.push_back(rpy.x()) ;
    xyz_rpy.push_back(rpy.y()) ;
    xyz_rpy.push_back(rpy.z()) ;
}



}
