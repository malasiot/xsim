#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include <iostream>

#include <QTimer>
#include <QDebug>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(PhysicsWorld &physics, Robot &robot):
    SimulationGui(physics), robot_(robot) {


    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

    auto world = physics.getVisual() ;
    auto robot_node = world->findNodeByName("base_link") ;

    //    physics.setCollisionFeedback(this);

    Quaternionf rot{0, -1, 0, 1};
    rot.normalize() ;

    Isometry3f pose ;
    pose.setIdentity() ;
    pose.linear() = rot.matrix() ;
    pose.translation() = Vector3f{ 0.25, 0.25, 0.2} ;

 //   robot_.openGripper() ;

    Vector3f ik_center(0, 0, 0.2) ;

    target_.reset(new Node) ;
    target_->setTransform(Isometry3f(Translation3f(ik_center))) ;
    GeometryPtr geom(new BoxGeometry({0.01, 0.01, 0.01})) ;
    PhongMaterial *material = new PhongMaterial({1, 0, 1}, 0.5) ;
    MaterialPtr mat(material) ;
    target_->addDrawable(geom, mat) ;
    robot_node->addChild(target_) ;

    texec_ = new TrajectoryExecutionManager(&robot_, this) ;
    QObject::connect(texec_, &TrajectoryExecutionManager::robotStateChanged, this, &GUI::updateControls);
    QObject::connect(texec_, &TrajectoryExecutionManager::trajectoryExecuted, this, &GUI::moveRelative) ;


    gizmo_.reset(new TransformManipulator(camera_, 0.15)) ;
    gizmo_->gizmo()->show(true) ;
    gizmo_->gizmo()->setOrder(2) ;

    gizmo_->setCallback([this, rot](TransformManipulatorEvent e, const Affine3f &f) {
        if ( e == TRANSFORM_MANIP_MOTION_ENDED )  {
            Isometry3f p = Isometry3f::Identity() ;
            p.translation() = f.translation()  ;
            p.linear() = rot.matrix() ;
            robot_.getJointState(start_state_) ;

            //robot_.moveTo(p) ;//ik(*robot_mb, Isometry3f(  p.matrix()), M_PI/4) ;
            JointTrajectory traj ;
            if ( robot_.plan(p, traj) ) {
                texec_->execute(traj) ;
            }

        } else if ( e == TRANSFORM_MANIP_MOVING ) {
  //          cout << f.translation().adjoint() << endl ;
        }

    });

    gizmo_->attachTo(target_.get());
    gizmo_->setLocalTransform(true);



}

void GUI::onUpdate(float delta) {
    SimulationGui::onUpdate(delta) ;
    //     vector<ContactResult> results ;
    //    physics.contactPairTest(target_, table_mb->getLink("baseLink"), 0.01, results) ;
}

void GUI::keyPressEvent(QKeyEvent *event) {
    if ( event->key() == Qt::Key_Q ) {
 //       robot_.openGripper();
    } else if ( event->key() == Qt::Key_W ) {
//        robot_.closeGripper() ;
    } else if ( event->key() == Qt::Key_L )
        gizmo_->setLocalTransform(true) ;
    else if ( event->key() == Qt::Key_G )
        gizmo_->setLocalTransform(false) ;
    else SimulationGui::keyPressEvent(event) ;

    update() ;

}

void GUI::mousePressEvent(QMouseEvent *event) {
    if ( gizmo_->onMousePressed(event) ) {
        update() ;
        return ;
    }

    SimulationGui::mousePressEvent(event) ;
}

void GUI::mouseReleaseEvent(QMouseEvent *event) {
    if ( gizmo_->onMouseReleased(event) ) {
        update() ;
        return ;
    }

    SimulationGui::mouseReleaseEvent(event) ;
}

void GUI::mouseMoveEvent(QMouseEvent *event) {
    if ( gizmo_->onMouseMoved(event) ) {
        update() ;
        return ;
    }

    SimulationGui::mouseMoveEvent(event) ;
}

void GUI::processContact(ContactResult &r) {
    if ( r.a_ == nullptr || r.b_ == nullptr ) return ;
    if ( r.a_->getName() == "ground" || r.b_->getName() == "ground" ) return  ;
    cout << r.a_->getName() << ' ' << r.b_->getName() << endl ;
}

void GUI::changeControlValue(const std::string &jname, float v) {
    cout << jname << ' ' << v << endl ;
    //robot_mb->setJointPosition(jname, v) ;
    robot_.setJointState(jname, v) ;
}

void GUI::updateControls(const JointState &state)
{
    MainWindow::instance()->updateControls(state);
}

void GUI::moveRelative() {
    JointTrajectory traj ;
        if ( robot_.planRelative({0.2, 0, 0}, traj) ) {
            texec_->execute(traj) ;

            QObject::disconnect(texec_, &TrajectoryExecutionManager::trajectoryExecuted, this, &GUI::moveRelative);
        }

}

void TrajectoryExecutionManager::timerTicked()
{
    JointState state ;
    robot_->getJointState(state) ;

    const auto &target = traj_.points().back() ;

    bool changed = false ;
    for( const auto &sp: state ) {
        float start_v = target.find(sp.first)->second;
        float v = state[sp.first] ;

        if ( fabs(v - start_v) > 0.01 ) {
            changed = true ;
            break ;
        }
    }
    emit robotStateChanged(state) ;

    if ( !changed ) {
        timer_.stop() ;
        robot_->stop() ;
        emit trajectoryExecuted();
    }


}

void TrajectoryExecutionManager::execute(const xsim::JointTrajectory &traj)
{
    traj_ = traj ;
    timer_.setInterval(100) ;
    connect(&timer_, &QTimer::timeout, this, &TrajectoryExecutionManager::timerTicked);

    timer_.start() ;
    robot_->moveTo(traj_.points().back());
}
