#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "robot.hpp"

#include <xsim/world.hpp>
#include <xsim/collision.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/gui/manipulator.hpp>

#include <QTimer>

class GUI: public SimulationGui, xsim::CollisionFeedback {
    Q_OBJECT
public:
    GUI(xsim::PhysicsWorld &physics, Robot &rb);

    void openGripper();

    void closeGripper();

    void onUpdate(float delta) override;

    void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void processContact(xsim::ContactResult &r) override;

private:
    std::shared_ptr<xviz::TransformManipulator> gizmo_;
    xviz::NodePtr target_ ;
    Robot &robot_ ;
    QTimer timer_ ;
    std::map<std::string, float> start_state_ ;

Q_SIGNALS:
    void robotStateChanged(const std::map<std::string, float> &) ;

public slots:
    void changeControlValue(const std::string &jname, float v);

    void executionTimerTicked() ;


};


#endif // GUI_HPP
