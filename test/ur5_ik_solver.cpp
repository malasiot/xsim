#include "ur5_ik_solver.hpp"
#include <Eigen/Geometry>

using namespace Eigen ;
using namespace std ;

#define UR5_PARAMS

const char *UR5IKSolver::ur5_joint_names[6] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

const double ZERO_THRESH = 0.00000001;

inline int SIGN(double x) {
    return (x > 0) - (x < 0);
}
const double PI = M_PI;

#ifdef UR10_PARAMS
const double d1 =  0.1273;
const double a2 = -0.612;
const double a3 = -0.5723;
const double d4 =  0.163941;
const double d5 =  0.1157;
const double d6 =  0.0922;
#endif

#ifdef UR5_PARAMS
const double d1 =  0.089159;
const double a2 = -0.42500;
const double a3 = -0.39225;
const double d4 =  0.10915;
const double d5 =  0.09465;
const double d6 =  0.0823;
#endif

#ifdef UR3_PARAMS
const double d1 =  0.1519;
const double a2 = -0.24365;
const double a3 = -0.21325;
const double d4 =  0.11235;
const double d5 =  0.08535;
const double d6 =  0.0819;
#endif


static void forward(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
    double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
    *T = c234*c1*s5 - c5*s1; T++;
    *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
    *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
    *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
    *T = c1*c5 + c234*s1*s5; T++;
    *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
    *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
    *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
    *T = -s234*s5; T++;
    *T = -c234*s6 - s234*c5*c6; T++;
    *T = s234*c5*s6 - c234*c6; T++;
    *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
}

static void forward_all(const double* q, double* T1, double* T2, double* T3,
                        double* T4, double* T5, double* T6) {
    double s1 = sin(*q), c1 = cos(*q); q++; // q1
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
    q234 += *q; q++; // q4
    double s5 = sin(*q), c5 = cos(*q); q++; // q5
    double s6 = sin(*q), c6 = cos(*q); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);

    if(T1 != NULL) {
        *T1 = c1; T1++;
        *T1 = 0; T1++;
        *T1 = s1; T1++;
        *T1 = 0; T1++;
        *T1 = s1; T1++;
        *T1 = 0; T1++;
        *T1 = -c1; T1++;
        *T1 = 0; T1++;
        *T1 =       0; T1++;
        *T1 = 1; T1++;
        *T1 = 0; T1++;
        *T1 =d1; T1++;
        *T1 =       0; T1++;
        *T1 = 0; T1++;
        *T1 = 0; T1++;
        *T1 = 1; T1++;
    }

    if(T2 != NULL) {
        *T2 = c1*c2; T2++;
        *T2 = -c1*s2; T2++;
        *T2 = s1; T2++;
        *T2 =a2*c1*c2; T2++;
        *T2 = c2*s1; T2++;
        *T2 = -s1*s2; T2++;
        *T2 = -c1; T2++;
        *T2 =a2*c2*s1; T2++;
        *T2 =         s2; T2++;
        *T2 = c2; T2++;
        *T2 = 0; T2++;
        *T2 =   d1 + a2*s2; T2++;
        *T2 =               0; T2++;
        *T2 = 0; T2++;
        *T2 = 0; T2++;
        *T2 =                 1; T2++;
    }

    if(T3 != NULL) {
        *T3 = c23*c1; T3++;
        *T3 = -s23*c1; T3++;
        *T3 = s1; T3++;
        *T3 =c1*(a3*c23 + a2*c2); T3++;
        *T3 = c23*s1; T3++;
        *T3 = -s23*s1; T3++;
        *T3 = -c1; T3++;
        *T3 =s1*(a3*c23 + a2*c2); T3++;
        *T3 =         s23; T3++;
        *T3 = c23; T3++;
        *T3 = 0; T3++;
        *T3 =     d1 + a3*s23 + a2*s2; T3++;
        *T3 =                    0; T3++;
        *T3 = 0; T3++;
        *T3 = 0; T3++;
        *T3 =                                     1; T3++;
    }

    if(T4 != NULL) {
        *T4 = c234*c1; T4++;
        *T4 = s1; T4++;
        *T4 = s234*c1; T4++;
        *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
        *T4 = c234*s1; T4++;
        *T4 = -c1; T4++;
        *T4 = s234*s1; T4++;
        *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
        *T4 =         s234; T4++;
        *T4 = 0; T4++;
        *T4 = -c234; T4++;
        *T4 =                  d1 + a3*s23 + a2*s2; T4++;
        *T4 =                         0; T4++;
        *T4 = 0; T4++;
        *T4 = 0; T4++;
        *T4 =                                                  1; T4++;
    }

    if(T5 != NULL) {
        *T5 = s1*s5 + c234*c1*c5; T5++;
        *T5 = -s234*c1; T5++;
        *T5 = c5*s1 - c234*c1*s5; T5++;
        *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
        *T5 = c234*c5*s1 - c1*s5; T5++;
        *T5 = -s234*s1; T5++;
        *T5 = - c1*c5 - c234*s1*s5; T5++;
        *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
        *T5 =                           s234*c5; T5++;
        *T5 = c234; T5++;
        *T5 = -s234*s5; T5++;
        *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
        *T5 =                                                   0; T5++;
        *T5 = 0; T5++;
        *T5 = 0; T5++;
        *T5 =                                                                                 1; T5++;
    }

    if(T6 != NULL) {
        *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
        *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
        *T6 = c5*s1 - c234*c1*s5; T6++;
        *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
        *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
        *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
        *T6 = - c1*c5 - c234*s1*s5; T6++;
        *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
        *T6 =                                       c234*s6 + s234*c5*c6; T6++;
        *T6 = c234*c6 - s234*c5*s6; T6++;
        *T6 = -s234*s5; T6++;
        *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
        *T6 =                                                                                                   0; T6++;
        *T6 = 0; T6++;
        *T6 = 0; T6++;
        *T6 =                                                                                                                                            1; T6++;
    }
}

static int inverse(const double* T, double* q_sols, double q6_des) {
    int num_sols = 0;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++;
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++;
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
        double A = d6*T12 - T13;
        double B = d6*T02 - T03;
        double R = A*A + B*B;
        if(fabs(A) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
                div = -SIGN(d4)*SIGN(B);
            else
                div = -d4/B;
            double arcsin = asin(div);
            if(fabs(arcsin) < ZERO_THRESH)
                arcsin = 0.0;
            if(arcsin < 0.0)
                q1[0] = arcsin + 2.0*PI;
            else
                q1[0] = arcsin;
            q1[1] = PI - arcsin;
        }
        else if(fabs(B) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
                div = SIGN(d4)*SIGN(A);
            else
                div = d4/A;
            double arccos = acos(div);
            q1[0] = arccos;
            q1[1] = 2.0*PI - arccos;
        }
        else if(d4*d4 > R) {
            return num_sols;
        }
        else {
            double arccos = acos(d4 / sqrt(R)) ;
            double arctan = atan2(-B, A);
            double pos = arccos + arctan;
            double neg = -arccos + arctan;
            if(fabs(pos) < ZERO_THRESH)
                pos = 0.0;
            if(fabs(neg) < ZERO_THRESH)
                neg = 0.0;
            if(pos >= 0.0)
                q1[0] = pos;
            else
                q1[0] = 2.0*PI + pos;
            if(neg >= 0.0)
                q1[1] = neg;
            else
                q1[1] = 2.0*PI + neg;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
        for(int i=0;i<2;i++) {
            double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
            double div;
            if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
                div = SIGN(numer) * SIGN(d6);
            else
                div = numer / d6;
            double arccos = acos(div);
            q5[i][0] = arccos;
            q5[i][1] = 2.0*PI - arccos;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
        for(int i=0;i<2;i++) {
            for(int j=0;j<2;j++) {
                double c1 = cos(q1[i]), s1 = sin(q1[i]);
                double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                double q6;
                ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                if(fabs(s5) < ZERO_THRESH)
                    q6 = q6_des;
                else {
                    q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
                               SIGN(s5)*(T00*s1 - T10*c1));
                    if(fabs(q6) < ZERO_THRESH)
                        q6 = 0.0;
                    if(q6 < 0.0)
                        q6 += 2.0*PI;
                }
                ////////////////////////////////////////////////////////////////////////////////

                double q2[2], q3[2], q4[2];
                ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                double c6 = cos(q6), s6 = sin(q6);
                double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
                double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
                double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
                        T03*c1 + T13*s1;
                double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

                double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
                if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
                    c3 = SIGN(c3);
                else if(fabs(c3) > 1.0) {
                    // TODO NO SOLUTION
                    continue;
                }
                double arccos = acos(c3);
                q3[0] = arccos;
                q3[1] = 2.0*PI - arccos;
                double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
                double s3 = sin(arccos);
                double A = (a2 + a3*c3), B = a3*s3;
                q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
                q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
                double c23_0 = cos(q2[0]+q3[0]);
                double s23_0 = sin(q2[0]+q3[0]);
                double c23_1 = cos(q2[1]+q3[1]);
                double s23_1 = sin(q2[1]+q3[1]);
                q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
                q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
                ////////////////////////////////////////////////////////////////////////////////
                for(int k=0;k<2;k++) {
                    if(fabs(q2[k]) < ZERO_THRESH)
                        q2[k] = 0.0;
                    else if(q2[k] < 0.0) q2[k] += 2.0*PI;
                    if(fabs(q4[k]) < ZERO_THRESH)
                        q4[k] = 0.0;
                    else if(q4[k] < 0.0) q4[k] += 2.0*PI;
                    q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k];
                    q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k];
                    q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6;
                    num_sols++;
                }

            }
        }
    }
    return num_sols;
}

double UR5IKSolver::limits_min_[] = { -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI } ;
double UR5IKSolver::limits_max_[] = {  M_PI,  M_PI,  M_PI,  M_PI,  M_PI,  M_PI } ;

typedef std::pair<int, double> idx_double;
bool comparator(const idx_double& l, const idx_double& r)
{ return l.second < r.second; }

static void getJointState(const xsim::JointState &src, double s[]) {
    for( int i=0 ; i<6 ; i++ ) {
        auto it = src.find(UR5IKSolver::ur5_joint_names[i]) ;
        assert(it != src.end()) ;
        s[i] = it->second ;
    }
}

static void setJointState(const vector<double> &s, xsim::JointState &dst ) {
    for( int i=0 ; i<6 ; i++ ) {
        dst.emplace(UR5IKSolver::ur5_joint_names[i], s[i]) ;
    }
}
bool UR5IKSolver::solve(const Eigen::Isometry3f &target, vector<xsim::JointState> &solutions)
{
    Matrix4d mat = target.matrix().cast<double>() ;

    double t[16], q_sols[8][6] ;
    uint c = 0 ;
    for ( uint i=0 ; i<4 ; i++)
        for( uint j=0 ; j<4 ; j++ )
            t[c++] = mat(i, j) ;

    int num_sols = inverse(t, (double *)q_sols, 0);

    int num_valid_sols;
    std::vector< std::vector<double> > q_valid_sols;

    for( int i=0; i<num_sols; i++)
    {
        bool valid = true;
        std::vector<double> valid_solution;
        valid_solution.assign(6, 0.0);

        for ( int j=0; j<6; j++ )
        {
            if((q_sols[i][j] <= limits_max_[j]) && (q_sols[i][j] >= limits_min_[j]))
            {
                valid_solution[j] = q_sols[i][j];
                valid = true;
                continue;
            }
            else if ((q_sols[i][j] > limits_max_[j]) && (q_sols[i][j]-2*M_PI > limits_min_[j]))
            {
                valid_solution[j] = q_sols[i][j]-2*M_PI;
                valid = true;
                continue;
            }
            else if ((q_sols[i][j] < limits_min_[j]) && (q_sols[i][j]+2*M_PI < limits_max_[j]))
            {
                valid_solution[j] = q_sols[i][j]+2*M_PI;
                valid = true;
                continue;
            }
            else
            {
                valid = false;
                break;
            }
        }

        if(valid)
        {
            q_valid_sols.push_back(valid_solution);
        }
    }

    if ( q_valid_sols.empty() ) return false;

    for( const auto &sol: q_valid_sols ) {
        xsim::JointState state ;
        setJointState(sol, state) ;
        solutions.emplace_back(state) ;
    }


    return true ;
}

double jvalue(const xsim::JointState &s, const std::string &name) {
    auto it = s.find(name) ;
    assert(it != s.end()) ;
    return it->second ;
}
bool UR5IKSolver::solve(const Eigen::Isometry3f &target, const xsim::JointState &seed, xsim::JointState &solution)
{
    vector<xsim::JointState> solutions ;
    if ( !solve(target, solutions) ) return false ;

    // use weighted absolute deviations to determine the solution closest the seed state
    std::vector<idx_double> weighted_diffs;
    for(uint16_t i=0; i<solutions.size(); i++) {
        double cur_weighted_diff = 0;
        for(uint16_t j=0; j<6; j++) {
            string jname = ur5_joint_names[j] ;
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
