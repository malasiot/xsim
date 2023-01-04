#pragma once

#include <xviz/scene/scene_fwd.hpp>
#include <xviz/gui/viewer.hpp>

#include <xsim/world.hpp>

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QElapsedTimer>

#include <iostream>

class SimulationGui : public xviz::SceneViewer
{
public:
    SimulationGui(xsim::PhysicsWorld *physics) ;
    SimulationGui(xsim::PhysicsWorld *physics, const Eigen::Vector3f &c, float r);
protected:

    xsim::PhysicsWorld *physics_ ;
    xsim::RayPicker picker_ ;

    bool picking_ = false ;


protected:
    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    virtual void onUpdate(float delta) override {
      // physics_->stepSimulation(delta/1000.0f);
      //  physics_.stepSimulation(0.0001);

    }
};
