#include "kuka_iiwa_ik_solver.hpp"
#include <Eigen/Geometry>
#include <cassert>

using namespace Eigen ;
using namespace std ;

#define UR5_PARAMS

const char *KukaIKSolver::s_joint_names[7] = {
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6",
    "joint_a7"
};

static double l0 = 0.36 ;
static double l1 = 0.42 ;
static double l2 = 0.40 ;
static double l3 = 0.126 ;

static double dh_a[7] = {  -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, 0 } ;
static double dh_l[7] = { l0, 0, l1, 0, l2, 0, l3 } ;

static Matrix4d dh_calc(double a, double alpha, double d, double theta) {
    Vector3d v{ a*cos(theta), a*sin(theta), d } ;
    double Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =  sin(theta) * sin(alpha);
    double Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy = -cos(theta) * sin(alpha);
    double Xz = 0.0       , Yz =  sin(alpha),  Zz =  cos(alpha);

    Matrix4d tmp ;
    tmp << Xx, Yx, Zx, v.x(),
           Xy, Yy, Zy, v.y(),
           Xz, Yz, Zz, v.z(),
           0,  0,  0,  1;
    return tmp ;
}

Isometry3d KukaIKSolver::forward(const xsim::JointState &state) {
    double joints[7] = {0}, dh_t[7] = {0} ;

    for( uint i=0 ; i<7 ; i++ ) {
        auto it = state.find(s_joint_names[i]) ;
        if ( it != state.end() ) {
            joints[i] = it->second ;
            dh_t[i] = it->second ;
        }

    }

    Eigen::Matrix4d tr[7] ;
    /*
    %Rotation Matrix applied with Denavit-Hartenberg parameters [same as (3)]
    %R = [Xx,Yx,Zx,   --  Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =  sin(theta) * sin(alpha)
    %     Xy,YY,Zy,   --  Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy = -cos(theta) * sin(alpha)
    %     Xz,Yz,Zz];  --  Xz = 0.0,        Yz =  sin(alpha),              Zz =  cos(alpha)
    */

    for( uint i=0 ; i<7 ; i++ ) {
        double a = 0.0, alpha = dh_a[i], d = dh_l[i], theta = dh_t[i] ;

        Vector3d v{ a*cos(theta), a*sin(theta), d } ;
        double Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =  sin(theta) * sin(alpha);
        double Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy = -cos(theta) * sin(alpha);
        double Xz = 0.0       , Yz =  sin(alpha),  Zz =  cos(alpha);

        Matrix4d tmp ;
        tmp << Xx, Yx, Zx, v.x(),
               Xy, Yy, Zy, v.y(),
               Xz, Yz, Zz, v.z(),
               0,  0,  0,  1;

        if ( i == 0 )
            tr[i] = tmp;
        else
            tr[i] = tr[i-1] * tmp;
    }

    Vector3d xs = tr[0].block<3, 1>(0, 3);  // shoulder position from base
    Vector3d xe = tr[3].block<3, 1>(0, 3);  // elbow position from base
    Vector3d xw = tr[5].block<3, 1>(0, 3);  // wrist position from base

    Vector3d xsw = xw-xs;       // wrist position from shoulder

    Isometry3d pose(tr[6]) ;// end-effector transformation from base

    return pose ;

}


double KukaIKSolver::limits_min_[] = { -170 * M_PI/180.0, -120 * M_PI/180.0, -170 * M_PI/180, -120 * M_PI/180.0, -170 * M_PI/180, -120 * M_PI/180.0, -175 * M_PI / 180} ;
double KukaIKSolver::limits_max_[] = {  170 * M_PI/180.0,  120 * M_PI/180.0,  170 * M_PI/180,  120 * M_PI/180.0,  170 * M_PI/180,  120 * M_PI/180.0, 175 * M_PI / 180} ;

typedef std::pair<int, double> idx_double;
bool comparator(const idx_double& l, const idx_double& r)
{ return l.second < r.second; }

static void getJointState(const xsim::JointState &src, double s[]) {
    for( int i=0 ; i<7 ; i++ ) {
        auto it = src.find(KukaIKSolver::s_joint_names[i]) ;
        assert(it != src.end()) ;
        s[i] = it->second ;
    }
}

static void setJointState(const vector<double> &s, xsim::JointState &dst ) {
    for( int i=0 ; i<7 ; i++ ) {
        dst.emplace(KukaIKSolver::s_joint_names[i], s[i]) ;
    }
}

xsim::JointState makeJointState(double j[7]) {
    xsim::JointState js ;
    for( uint i=0 ; i<7 ; i++ )
        js[KukaIKSolver::s_joint_names[i]] = j[i] ;
    return js ;
}
static const uint J_CONF_ARM = 1 ;
static const uint J_CONF_ELBOW = 2 ;
static const uint J_CONF_WRIST = 4 ;

bool KukaIKSolver::solve(const Eigen::Isometry3f &target, double psi, vector<xsim::JointState> &solutions) {
    for( uint r = 0 ; r < 8 ; r ++ ) {
        double js[7] ;
        if ( solve(target, psi, r, js) &&
             checkLimits(js) ) {
            solutions.push_back(makeJointState(js)) ;
        }
    }

    return !solutions.empty() ;
}

static bool getReferencePlane( const Matrix4d &pose, int elbow,
                       Vector3d &ref_plane_vector, Matrix3d &rot_base_elbow, double joints[7] ) {

    Matrix4d smat[3] = {Matrix4d::Zero()}, wmat[3] = {Matrix4d::Zero()};
    Vector3d xend = pose.block<3, 1>(0, 3) ; // end-effector position from base
    Vector3d xs0{0,  0,  l0} ;// shoulder position from base
    Vector3d xwt{0,  0,  l3} ; // end-effector position from wrist
    Vector3d xw0 = xend - pose.block<3, 3>(0, 0) * xwt; // wrist position from base
    Vector3d xsw = xw0 - xs0; // shoulder to wrist vector
    Vector3d usw = xsw.normalized();

    double lbs = l0;
    double lse = l1; // upper arm length (shoulder to elbow)
    double lew = l2; // lower arm length (elbow to wrist)

    //%Check if pose is within arm+forearm reach
    if (xsw.norm() > lse + lew || xsw.norm() < lse - lew ) return false ; // "Specified pose outside reacheable workspace");

    const double tol = 1.0e-8 ;
    // -- Joint 4 --
    // Elbow joint can be directly calculated since it does only depend on the
    // robot configuration and the xsw vector
    if(fabs((xsw.squaredNorm() - lse * lse - lew * lew)-(2*lse*lew)) < tol ) return false ; //&& "Elbow singularity. Tip at reach limit.");
    // Cosine law - According to our robot, joint 4 rotates backwards
    joints[3] = elbow * acos((xsw.squaredNorm() - lse * lse - lew * lew)/(2*lse*lew));

    //Added

    Matrix4d T34 = dh_calc(0, dh_a[3], dh_l[3], joints[3]);
    Matrix3d R34 = T34.block<3,3>(0, 0);

    // These are the vectors corresponding to our DH parameters
    Vector3d xse{0, lse, 0};
    Vector3d xew{0, 0, lew};
    //% m = member between parentisis. Check equation (14)
    VectorXd m = xse + R34*xew;

    /// -- Joint 1 --
    // Since joint3 is locked as 0, the only joint to define the orientation of
    // the xsw vector in the xy-plane is joint 1. Therefore and since we are
    // only interested in the transformation T03 (disregarding joint limits), we
    // chose to simply set joint 1 as the atan of xsw y and x coordinates
    // (even if if goes beyond the joint limit).

    //Cannot be this because if x and y are 0, then it is not defined.
    if ( xsw.cross(Vector3d::UnitZ()).norm() > tol )
        joints[0] = atan2(xsw[1],xsw[0]);
    else
        joints[0] = 0;

    // -- Joint 2 --
    // Can be found through geometric relations
    // Let phi be the angle E-S-W, and theta2 the angle (z-axis)-S-E.
    // Then, theta2 = atan2(r,xsw(3)) -/+ phi.
    // phi can be calculated, as a function of theta3:
    //   atan2(lew*sin(theta4),lse+lew*cos(theta4))
    // z-axis
    //   ^
    //   |  E O------------O W
    //   |   /        .
    //   |  /      .
    //   | /    .    xsw
    //   |/  .
    // S O___________________ r-axis
    //
    double r = hypot(xsw[0], xsw[1]);
    double dsw = xsw.norm();
    double phi = acos((lse * lse + dsw * dsw - lew * lew)/(2*lse*dsw));

    joints[1] = atan2(r, xsw[2]) + elbow * phi;

    // Lower arm transformation
    auto T01 = dh_calc(0, dh_a[0],dh_l[0], joints[0]);
    auto T12 = dh_calc(0, dh_a[1],dh_l[1], joints[1]);
    auto T23 = dh_calc(0, dh_a[2],dh_l[2], 0);
    T34 = dh_calc(0, dh_a[3],dh_l[3], joints[3]);
    auto T04 = T01*T12*T23*T34;

    rot_base_elbow = T01.block<3, 3>(0, 0) * T12.block<3, 3>(0, 0) * T23.block<3, 3>(0, 0);

    // With T03 we can calculate the reference elbow position and with it the
    // vector normal to the reference plane.
    Vector3d x0e = T04.block<3, 1>(0, 3); // reference elbow position
    Vector3d v1 = (x0e - xs0).normalized(); // unit vector from shoulder to elbow
    Vector3d v2 = (xw0 - xs0).normalized(); // unit vector from shoulder to wrist

    ref_plane_vector = v1.cross(v2) ;

}

static Matrix3d skew(const Vector3d &v) {
    Matrix3d r ;

    r << 0, -v[2], v[1],
         v[2], 0, -v[0],
         -v[1], v[0], 0 ;
    return r ;
}

bool KukaIKSolver::solve(const Eigen::Isometry3f &target, double nsparam, uint rconf, double joints[7])
{
    Matrix4d pose = target.matrix().cast<double>() ;

    Matrix3d R = pose.block<3, 3>(0, 0) ;

    int arm = ( rconf & J_CONF_ARM ) ? -1 : 1 ;
    int elbow = ( rconf & J_CONF_ELBOW ) ? -1 : 1 ;
    int wrist = ( rconf & J_CONF_WRIST ) ? -1 : 1 ;


    Matrix3d smat[3] = {Matrix3d::Zero()}, wmat[3] = {Matrix3d::Zero()};
    Vector3d xend = pose.block<3, 1>(0, 3) ; // end-effector position from base
    Vector3d xs{0,  0,  l0} ;// shoulder position from base
    Vector3d xwt{0,  0,  l3} ; // end-effector position from wrist
    Vector3d xw = xend - R * xwt; // wrist position from base
    Vector3d xsw = xw - xs; // shoulder to wrist vector
    Vector3d usw = xsw.normalized();

    double lbs = l0;
    double lse = l1; // upper arm length (shoulder to elbow)
    double lew = l2; // lower arm length (elbow to wrist)

    //%Check if pose is within arm+forearm reach
    if ( xsw.norm() > lse + lew || xsw.norm() < lse - lew ) return false ;

    const double tol = 1.0e-8 ;
    // -- Joint 4 --
    // Elbow joint can be directly calculated since it does only depend on the
    // robot configuration and the xsw vector
    if(fabs((xsw.squaredNorm() - lse * lse - lew * lew)-(2*lse*lew)) < tol ) return false ; // "Elbow singularity. Tip at reach limit.");
    // Cosine law - According to our robot, joint 4 rotates backwards
    joints[3] = elbow * acos((xsw.squaredNorm() - lse * lse - lew * lew)/(2*lse*lew));

    //Added

    Matrix4d T34 = dh_calc(0, dh_a[3], dh_l[3], joints[3]);
    Matrix3d R34 = T34.block<3,3>(0, 0);

    Matrix3d R03_o ;
    double jv[7] = {0};
    Vector3d jp ;
    getReferencePlane(pose, elbow, jp, R03_o, jv) ;

    Matrix3d skew_usw = skew(usw);
    // Following eq. (15), the auxiliary matrixes As Bs and Cs can be calculated
    // by substituting eq. (6) into (9).
    // R0psi = I3 + sin(psi)*skew_usw + (1-cos(psi))*skew_usw²    (6)
    // R03 = R0psi * R03_o                                         (9)
    // Substituting (distributive prop.) we get:
    // R03 = R03_o*skew_usw*sin(psi) + R03_o*(-skew_usw²)*cos(psi) + R03_o(I3 + skew_usw²)
    // R03 =      As       *sin(psi) +        Bs         *cos(psi) +          Cs
    Matrix3d As = skew_usw * R03_o;
    Matrix3d Bs = -skew_usw * skew_usw * R03_o;
    Matrix3d Cs = (usw*usw.transpose()) * R03_o;

    double psi = nsparam;
    Matrix3d R03 = As*sin(psi) + Bs*cos(psi) + Cs;

    joints[0] = atan2(arm * R03(1, 1), arm * R03(0, 1));
    joints[1] = arm * acos(R03(2, 1));
    joints[2] = atan2(arm * -R03(2,2), arm * -R03(2,0));

    Matrix3d Aw = R34.transpose() * As.transpose() * pose.block<3, 3>(0, 0);
    Matrix3d Bw = R34.transpose() * Bs.transpose() *  pose.block<3, 3>(0, 0);
    Matrix3d Cw = R34.transpose() * Cs.transpose() *  pose.block<3, 3>(0, 0);

    Matrix3d R47 = Aw*sin(psi) + Bw*cos(psi) + Cw;

    joints[4] = atan2(wrist * R47(1, 2), wrist * R47(0, 2));
    joints[5] = wrist * acos(R47(2, 2));
    joints[6] = atan2(wrist * R47(2, 1), wrist * -R47(2,0));

    // Grouping Shoulder and Wrist matrices that will be used by the joint
    // limit algorithms
    smat[0] = As;
    smat[1] = Bs;
    smat[2] = Cs;
    wmat[0] = Aw;
    wmat[1] = Bw;
    wmat[2] = Cw;

    return true ;
}

bool KukaIKSolver::checkLimits(double js[7])
{
    for( uint i=0 ; i<7 ; i++ ) {
        if ( js[i] < limits_min_[i] ) return false ;
        if ( js[i] > limits_max_[i] ) return false ;
    }
    return true ;
}

double jvalue(const xsim::JointState &s, const std::string &name) {
    auto it = s.find(name) ;
    assert(it != s.end()) ;
    return it->second ;
}
bool KukaIKSolver::solve(const Eigen::Isometry3f &target, double psi, const xsim::JointState &seed, xsim::JointState &solution)
{
    vector<xsim::JointState> solutions ;
    if ( !solve(target, psi, solutions) ) return false ;

    // use weighted absolute deviations to determine the solution closest the seed state
    std::vector<idx_double> weighted_diffs;
    for(uint16_t i=0; i<solutions.size(); i++) {
        double cur_weighted_diff = 0;
        for(uint16_t j=0; j<7; j++) {
            string jname = s_joint_names[j] ;
            // solution violates the consistency_limits, throw it out
            double abs_diff = std::fabs(jvalue(seed, jname) - jvalue(solutions[i], jname));
            cur_weighted_diff += abs_diff;
        }
        weighted_diffs.push_back(idx_double(i, cur_weighted_diff));
    }

    std::sort(weighted_diffs.begin(), weighted_diffs.end(), comparator);


    int cur_idx = weighted_diffs[0].first;
    solution = solutions[cur_idx];

    return true ;
}
