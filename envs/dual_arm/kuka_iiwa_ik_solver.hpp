#pragma once

#include <Eigen/Geometry>
#include <xsim/multi_body.hpp>
#include <xsim/kinematic.hpp>

using JointCoeffs = std::array<double, 7> ;

class KukaIKSolver {
    public:


    // analytic IK for Kuka IWAA 7 dof arm

    struct Problem {

        Problem(const Eigen::Isometry3f &target): target_(target) {}

        void setSeedState(const JointCoeffs &seed) {
            seed_ = seed ;
            has_seed_ = true ;
        }

        void setPsi(double psi) {
            psi_ = psi ;
            has_psi_ = true ;
        }

        void setPsiSamples(uint ns) {
            n_psi_ = ns ;
        }


        Eigen::Isometry3f target_ ;
        JointCoeffs seed_ ;
        double psi_ ;
        size_t n_psi_ = 8;
        bool has_psi_ = false, has_seed_ = false ;
    };

    bool solve(const Problem &pr, std::vector<JointCoeffs> &solutions) ;

    bool solve(const Eigen::Isometry3f &target, double psi, const xsim::JointState &seed, xsim::JointState &solution) ;
    bool solve(const Eigen::Isometry3f &target, double psi, std::vector<JointCoeffs> &solutions) ;

    static double limits_min_[7], limits_max_[7] ;
    static const char *s_joint_names[7] ;

    Eigen::Isometry3d forward(const xsim::JointState &state);

    bool solve(const Eigen::Isometry3f &target, double nsparam, uint rconf, JointCoeffs &js) ;

    bool checkLimits(const JointCoeffs &js) ;
};



