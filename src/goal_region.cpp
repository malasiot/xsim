#include <xsim/goal_region.hpp>

#include <ompl/util/RandomNumbers.h>

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
#if 0
SimpleShapeRegion::SimpleShapeRegion(): GoalRegion()
{
    roll_min = yaw_min = pitch_min = -M_PI ;
    roll_max = yaw_max = pitch_max = M_PI ;
}

void SimpleShapeRegion::computeOrientationBounds()
{
    roll_min = std::max(roll_min, -M_PI) ;
    roll_max = std::min(roll_max, M_PI) ;

    pitch_min = std::max(pitch_min, -M_PI) ;
    pitch_max = std::min(pitch_max, M_PI) ;

    yaw_min = std::max(yaw_min, -M_PI) ;
    yaw_max = std::min(yaw_max, M_PI) ;


    lower_bounds.push_back(roll_min) ;
    lower_bounds.push_back(pitch_min) ;
    lower_bounds.push_back(yaw_min) ;

    upper_bounds.push_back(roll_max) ;
    upper_bounds.push_back(pitch_max) ;
    upper_bounds.push_back(yaw_max) ;
}

void SimpleShapeRegion::computeBounds()
{
    computePositionBounds();
    computeOrientationBounds();
}

void SimpleShapeRegion::setShapePose(const Vector3 &orig, const Vector3 &rpy)
{
    double qx, qy, qz, qw ;

    rpy_to_quat(rpy.x, rpy.y, rpy.z, qx, qy, qz, qw) ;

    Rotation3D rot(qx, qy, qz, qw) ;
    Translation3D trans(orig.x, orig.y, orig.z) ;

    t = trans * rot ;
}

void SimpleShapeRegion::sample(vector<double> &xyz_rpy)
{
    if ( lower_bounds.empty() ) computeBounds() ;

    double X_, Y_, Z_ ;
    samplePosition(X_, Y_, Z_) ;

    Vector4 pt =   t * Vector4(X_, Y_, Z_, 1);

    double X = pt.x ;
    double Y = pt.y ;
    double Z = pt.z ;

    double Roll  = g_rng.uniformReal(lower_bounds[3], upper_bounds[3]) ;
    double Pitch = g_rng.uniformReal(lower_bounds[4], upper_bounds[4]) ;
    double Yaw   = g_rng.uniformReal(lower_bounds[5], upper_bounds[5]) ;

    xyz_rpy.push_back(X) ;
    xyz_rpy.push_back(Y) ;
    xyz_rpy.push_back(Z) ;
    xyz_rpy.push_back(Roll) ;
    xyz_rpy.push_back(Pitch) ;
    xyz_rpy.push_back(Yaw) ;


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoxShapedRegion::BoxShapedRegion(const Vector3 &c_, const Vector3 &sz_, const Vector3 &rpy_): SimpleShapeRegion(), orig(c_), rpy(rpy_), sz(sz_)
{
    setShapePose(orig, rpy) ;
}

void BoxShapedRegion::computePositionBounds()
{
    double xmin = -fabs(sz.x) ;
    double ymin = -fabs(sz.y) ;
    double zmin = -fabs(sz.z) ;

    double xmax = fabs(sz.x) ;
    double ymax = fabs(sz.y) ;
    double zmax = fabs(sz.z) ;

    lower_bounds.push_back(xmin) ;
    lower_bounds.push_back(xmin) ;
    lower_bounds.push_back(zmin) ;

    upper_bounds.push_back(xmax) ;
    upper_bounds.push_back(ymax) ;
    upper_bounds.push_back(zmax) ;
}

void BoxShapedRegion::samplePosition(double &X, double &Y, double &Z)
{
    X = g_rng.uniformReal(lower_bounds[0], upper_bounds[0]) ;
    Y = g_rng.uniformReal(lower_bounds[1], upper_bounds[1]) ;
    Z = g_rng.uniformReal(lower_bounds[2], upper_bounds[2]) ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CylinderShapedRegion::CylinderShapedRegion(double length, double radius, const Vector3 &c_, const Vector3 &rpy_): SimpleShapeRegion(), orig(c_), rpy(rpy_), rad(radius), len(length)
{
    setShapePose(orig, rpy) ;
}

void CylinderShapedRegion::computePositionBounds()
{
    double lmin = -len/2.0 ;
    double tmin = -M_PI ;
    double rmin = 0 ;

    double lmax = len/2.0 ;
    double tmax = M_PI ;
    double rmax = rad ;

    lower_bounds.push_back(lmin) ;
    lower_bounds.push_back(tmin) ;
    lower_bounds.push_back(rmin) ;

    upper_bounds.push_back(lmax) ;
    upper_bounds.push_back(tmax) ;
    upper_bounds.push_back(rmax) ;
}

void CylinderShapedRegion::samplePosition(double &X, double &Y, double &Z)
{
    double l = g_rng.uniformReal(lower_bounds[0], upper_bounds[0]) ;
    double t = g_rng.uniformReal(lower_bounds[1], upper_bounds[1]) ;
    double r = g_rng.uniformReal(lower_bounds[2], upper_bounds[2]) ;

    Z = l ;
    X = r * cos(t) ;
    Y = r * sin(t) ;
}



#endif


}
