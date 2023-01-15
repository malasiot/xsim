#pragma once

#include <Eigen/Geometry>
#include <memory>

namespace xsim {

class PlanningInterface ;

class GoalRegion {

public:
    GoalRegion() = default ;

    // sampling function

    virtual void sample(std::vector<double> &xyz_rpy) const = 0;
};


// This class defines a tolerance zone around a desired pose

class SimplePoseGoal: public GoalRegion {

public:
    // The constructor initializes the tolerance values to threshold
    // The user has then to expand tolerance for specific dimensions

    SimplePoseGoal(const Eigen::Isometry3f &pose) ;

    void setXYZTolerance(const Eigen::Vector3f &tol) {
        xyz_tol_ = tol ;
        computeBounds() ;
    }

    void setRPYTolerance(const Eigen::Vector3f &tol) {
        rpy_tol_ = tol ;
        computeBounds() ;
    }

    void sample(std::vector<double> &xyz_rpy) const override ;


private:

    static constexpr double default_xyz_tol_ = 0.005;
    static constexpr double default_rpy_tol_ = 0.005;

    void computeBounds() ;

    Eigen::Vector3f xyz_tol_, rpy_tol_ ;

    double lower_bounds_[6], upper_bounds_[6] ;
    Eigen::Isometry3f pose_ ;
};

// This is the base class for goal regions that constrain the position of the end-effector inside a shape
// The orientation region is a box in roll-yaw-pitch space

class SimpleShapeRegion: public GoalRegion {
public:

    SimpleShapeRegion(const Eigen::Vector3f &rpy_tol): rpy_tol_(rpy_tol) {}

  protected:

    void computeOrientationBounds(const Eigen::Vector3f &rpy) ;
    Eigen::Vector3f sampleOrientation() const;

    double lower_bounds_[6], upper_bounds_[6] ;
    Eigen::Vector3f rpy_tol_ ;
};

class BoxShapedRegion: public SimpleShapeRegion {

public:
    BoxShapedRegion(const Eigen::Vector3f &c, const Eigen::Vector3f &hsz,
                    const Eigen::Vector3f &rpy,
                    const Eigen::Vector3f &rpy_tol) ;

    void sample(std::vector<double> &xyz_rpy) const override;
protected:
    Eigen::Vector3f orig_, rpy_, hsz_ ;
};

class CylinderShapedRegion: public SimpleShapeRegion {

public:
    CylinderShapedRegion(float length, float radius, const Eigen::Vector3f &c, const Eigen::Vector3f &rpy) ;

protected:

    virtual void computePositionBounds() ;
    virtual void samplePosition(double &X, double &Y, double &Z) ;

private:

    Eigen::Vector3f orig_, rpy_ ;
    float len_, rad_ ;
};

// Define a goal region as the union of regions

class CompositeGoalRegion: public GoalRegion {
public:

    CompositeGoalRegion() ;

    void addRegion(GoalRegion *reg) ;

    virtual void sample(std::vector<double> &xyz_rpy) ;


private:
    std::vector<std::unique_ptr<GoalRegion>> regions_ ;

};










}
