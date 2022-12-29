#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "robot.hpp"
#include "world.hpp"

#include <xsim/world.hpp>
#include <xsim/collision.hpp>
#include <xsim/ompl_planner.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/gui/manipulator.hpp>

#include <QTimer>

class TrajectoryExecutionManager: public QObject {
    Q_OBJECT
public:
    TrajectoryExecutionManager(Robot *robot,  QObject *parent = nullptr): QObject(parent), robot_(robot) {}

    void execute(const xsim::JointTrajectory &traj) ;



public slots:

    void timerTicked();

signals:
    void trajectoryExecuted() ;
    void robotStateChanged(const xsim::JointState &state) ;
private:

    xsim::JointTrajectory traj_ ;
    int current_ ;
    Robot *robot_ ;
    QTimer timer_ ;
};

class GUI: public SimulationGui, xsim::CollisionFeedback {
    Q_OBJECT
public:
    GUI(World *world);

    void onUpdate(float delta) override;

    void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void processContact(xsim::ContactResult &r) override;

private:
    std::shared_ptr<xviz::TransformManipulator> gizmo_;
    xviz::NodePtr target_ ;
    std::shared_ptr<Robot> robot_ ;
    QTimer timer_ ;
    JointState start_state_ ;
    TrajectoryExecutionManager *texec_ ;


public slots:
    void changeControlValue(const std::string &jname, float v);
    void updateControls(const xsim::JointState &state) ;
    void moveRelative();
};


#endif // GUI_HPP
