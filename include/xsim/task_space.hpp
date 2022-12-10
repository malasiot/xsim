#pragma once

#include <vector>
#include <Eigen/Geometry>

namespace xsim {

class TaskSpace
{
public:
    TaskSpace() {}

    // get the dimension of the task space
    virtual int getDimension() const = 0 ;

    // get upper-lower limits of each dimension
    virtual double getUpperLimit(int dim) const = 0 ;
    virtual double getLowerLimit(int dim) const = 0 ;

    // transform a state from task space to a pose of the end-effector(s) in world coordinates
    virtual void taskSpaceToPose(const std::vector<double> &state, Eigen::Isometry3f &pose) const = 0;
    // transform pose of the end-effector(s) to state in task space
    virtual void poseToTaskSpace(const Eigen::Isometry3f &pose, std::vector<double> &state) const = 0;

};

// This is the most common task space defined by the pose of the end-effector

class RPY_XYZ_TaskSpace: public TaskSpace {
    public:

    RPY_XYZ_TaskSpace() ;

    int getDimension() const override { return 6 ; }

    double getUpperLimit(int dim) const override { return upper_[dim]; }
    double getLowerLimit(int dim) const override { return lower_[dim]; }

    void taskSpaceToPose(const std::vector<double> &state, Eigen::Isometry3f &pose) const override ;
    void poseToTaskSpace(const Eigen::Isometry3f &pose, std::vector<double> &state) const override ;

private:

    enum { TX = 0, TY = 1, TZ = 2, ROLL = 3, PITCH = 4, YAW = 5 } ;

    double upper_[6], lower_[6] ;

};

// This may be used to  move the tip linearly from the current position to a target position while keeping the orientation within limits

class MoveRelativeTaskSpace: public TaskSpace {
    public:

    /*
     * pose:    The current pose of the end-effector
     * dp:      The displacement of the end-effector
     * pos_tol: This is the positional tolerance of the end-effector with respect to the line between the start and end point
     * rpy_minus, rpy_plus: Negative and positive offset of end-effector orientation with respect to the current orientation
    */

    MoveRelativeTaskSpace(const Eigen::Isometry3f &pose, const Eigen::Vector3f &dp, double pos_tol,
                     const Eigen::Vector3f &rpy_minus, const Eigen::Vector3f &rpy_plus) ;

    int getDimension() const override { return 6 ; }

    double getUpperLimit(int dim) const override { return upper_[dim]; }
    double getLowerLimit(int dim) const override { return lower_[dim]; }

    void taskSpaceToPose(const std::vector<double> &state, Eigen::Isometry3f &pose) const override ;
    void poseToTaskSpace(const Eigen::Isometry3f &pose, std::vector<double> &state) const override ;

private:

    double upper_[6], lower_[6] ;
    Eigen::Vector3f c0_, a0_ ;
    Eigen::Matrix3f frame_, iframe_ ;
    double travel_dist_ ;

};





}
