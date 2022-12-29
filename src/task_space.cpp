#include <xsim/task_space.hpp>
#include <iostream>

using namespace std ;
using namespace Eigen ;

namespace xsim {
const double tol = 1.0e-3 ;
RPY_XYZ_TaskSpace::RPY_XYZ_TaskSpace(): TaskSpace() {
    upper_[TX] = 2 ; lower_[TX] = -2 ;
    upper_[TY] = 2 ; lower_[TY] = -2 ;
    upper_[TZ] = 2 ; lower_[TZ] = -2 ;

    upper_[ROLL] = M_PI + tol; lower_[ROLL] = -M_PI - tol ;
    upper_[PITCH] = M_PI + tol; lower_[PITCH] = -M_PI - tol;
    upper_[YAW] = M_PI + tol; lower_[YAW] = -M_PI - tol ;
}

void RPY_XYZ_TaskSpace::taskSpaceToPose(const std::vector<double> &state, Isometry3f &pose) const {
    double x = state[TX] ;
    double y = state[TY] ;
    double z = state[TZ] ;

    double roll = state[ROLL] ;
    double pitch = state[PITCH] ;
    double yaw = state[YAW] ;

    Quaternionf q = AngleAxisf(roll, Vector3f::UnitX())
            * AngleAxisf(pitch, Vector3f::UnitY())
            * AngleAxisf(yaw, Vector3f::UnitZ());

    pose.setIdentity() ;
    pose.linear() = q.toRotationMatrix() ;
    pose.translation() = Vector3f(x, y, z) ;
}

void rpy_to_quat(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw)
{
    double phi, the, psi;
    phi = roll / 2.0;
    the = pitch / 2.0;
    psi = yaw / 2.0;

    qx = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    qy = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    qz = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
    qw = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

    double sc = sqrt(qx * qx + qy * qy + qz * qz + qw * qw) ;

    qx /= sc ; qy /= sc ; qz /= sc ; qw /= sc ;
}
/*
static void poseToXYZRPY(const Isometry3f &pose, double &X, double &Y, double &Z, double &roll, double &pitch, double &yaw)
{
    Transform3D t(pose) ;

    RotationMatrix r = t.GetUpper3x3() ;
    Vector3 p = t.GetVector3Col() ;

    r.GetEulerAngles(yaw, pitch, roll, RotationMatrix::ZYX) ;

    roll = NormalizeCircularAngle(roll, -M_PI, M_PI) ;
    pitch = NormalizeCircularAngle(pitch, -M_PI, M_PI) ;
    yaw = NormalizeCircularAngle(yaw, -M_PI, M_PI) ;

    X = p.x ; Y = p.y ; Z = p.z ;


}*/

// transform pose of the end-effector to state in task space
void RPY_XYZ_TaskSpace::poseToTaskSpace(const Isometry3f &pose, std::vector<double> &state) const {
    Matrix3f r = pose.linear() ;
    Vector3f t = pose.translation() ;

    auto euler = r.eulerAngles(0, 1, 2);

    state.push_back(t.x()) ;
    state.push_back(t.y()) ;
    state.push_back(t.z()) ;
    state.push_back(euler.x()) ;
    state.push_back(euler.y()) ;
    state.push_back(euler.z()) ;
}

static Matrix3f lookAt(const Vector3f& eye, const Vector3f &center, const Vector3f &up) {
    Vector3f f = (center - eye).normalized();
    Vector3f u = up.normalized();
    Vector3f s = f.cross(u).normalized();
    u = s.cross(f);
    Matrix3f mat = Matrix3f::Zero() ;

    mat(0,0) = s.x();
    mat(0,1) = s.y();
    mat(0,2) = s.z();
    mat(1,0) = u.x();
    mat(1,1) = u.y();
    mat(1,2) = u.z();
    mat(2,0) = -f.x();
    mat(2,1) = -f.y();
    mat(2,2) = -f.z();

    return mat;
}


Vector3f perpendicular(const Vector3f &a) {
    float ax = fabs(a.x()), ay = fabs(a.y()), az = fabs(a.z()) ;
    if ( ax < ay )
        return ax < az ? Vector3f{0, -a.z(), a.y()} : Vector3f{-a.y(), a.x(), 0};
    else
        return ay < az ? Vector3f{a.z(), 0, -a.x()} : Vector3f{-a.y(), a.x(), 0};
}

Matrix3f basis(const Vector3f &a) {
    Matrix3f r ;
    Vector3f z = a.normalized() ;
    Vector3f x = perpendicular(z) ;
    Vector3f y = z.cross(x) ;
    r.col(0) = x;
    r.col(1) = y;
    r.col(2) = z;
    return r ;
}

MoveRelativeTaskSpace::MoveRelativeTaskSpace(const Isometry3f &pose, const Vector3f &dp, double pos_tol,
                                   const Vector3f &rpy_tol) {
    c0_ = pose.translation() ;
    a0_ = pose.linear().eulerAngles(0, 1, 2);

    Vector3f c1 = c0_ + dp ;

    // We define a coordinate system with the Z axis pointing towards the target point

    Matrix3f r = basis(dp) ;
  /*  if ( fabs(1 - dp.normalized().y() < 0.01 ))
        r = lookAt(c0_, c1, Vector3f::UnitX()) ;
    else
        r = lookAt(c0_, c1, Vector3f::UnitY()) ;
*/
    frame_ = r  ;
    iframe_ = r.inverse() ;

    // we use a cylinder parameterization of the position

    lower_[0] = 0.0      ; upper_[0] = dp.norm() + pos_tol; // cylinder length
    lower_[1] = -M_PI    ; upper_[1] = M_PI ; // polar angle
    lower_[2] = 0.0      ; upper_[2] = pos_tol ; // radius

    const double tol = 0.001 ;

    double roll_min = a0_.x() - rpy_tol.x() ;
    double roll_max = a0_.x() + rpy_tol.x() ;

    double pitch_min = a0_.y() - rpy_tol.y();
    double pitch_max = a0_.y() + rpy_tol.y() ;

    double yaw_min = a0_.z() - rpy_tol.z() ;
    double yaw_max = a0_.z() + rpy_tol.z() ;

    upper_[3] = roll_max     ; lower_[3] = roll_min ;
    upper_[4] = pitch_max    ; lower_[4] = pitch_min ;
    upper_[5] = yaw_max      ; lower_[5] = yaw_min ;
}



void MoveRelativeTaskSpace::taskSpaceToPose(const std::vector<double> &state, Isometry3f &pose) const
{
    double l = state[0] ;
    double theta = state[1] ;
    double r = state[2] ;

    double x = r * cos(theta) ;
    double y = r * sin(theta) ;
    double z = l  ;

    Vector3f pt = frame_ * Vector3f(x, y, z)  + c0_ ;

    double roll = state[3] ;
    double pitch = state[4] ;
    double yaw = state[5] ;

    Quaternionf q = AngleAxisf(roll, Vector3f::UnitX())
            * AngleAxisf(pitch, Vector3f::UnitY())
            * AngleAxisf(yaw, Vector3f::UnitZ());

    pose.setIdentity() ;
    pose.linear() = q.toRotationMatrix() ;
    pose.translation() = pt;
}

extern double normalizeCircularAngle(double theta, double l, double u) ;

void MoveRelativeTaskSpace::poseToTaskSpace(const Isometry3f &pose, std::vector<double> &state) const
{
    auto c = pose.translation() ;
    auto euler = pose.linear().eulerAngles(0, 1, 2);

    double roll = euler[0], pitch = euler[1], yaw = euler[2]  ;

    Vector3f pt = iframe_ * ( c - c0_ )  ;

    double l = pt.z()  ;
    double t = normalizeCircularAngle(atan2(pt.y(), pt.x()), -M_PI, M_PI) ;
    double r = sqrt(pt.x() * pt.x() + pt.y() * pt.y()) ;

    state.push_back(l) ;
    state.push_back(t) ;
    state.push_back(r) ;

    state.push_back(roll) ;
    state.push_back(pitch) ;
    state.push_back(yaw) ;
}






}
