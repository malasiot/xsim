#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include "world.hpp"
#include <iostream>

#include <QTimer>
#include <QDebug>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(World *physics):
    SimulationGui(physics), robot_(physics->controller_) {

    physics->stepSimulation(0.005);

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

    auto world = physics->getVisual() ;
    auto robot_node = world->findNodeByName("base_link") ;

    //    physics.setCollisionFeedback(this);

    Quaternionf rot{0, -1, 0, 1};
    rot.normalize() ;

    Matrix3f m(AngleAxisf(M_PI, Vector3f::UnitY())) ;

    Isometry3f pose ;
    pose.setIdentity() ;
    pose.linear() = rot.matrix() ;
    pose.translation() = Vector3f{ 0.25, 0.25, -0.075} ;

 //   robot_.openGripper() ;

    Vector3f ik_center(0, 0, 0.2) ;

    target_.reset(new Node) ;
 //   target_->setTransform(Isometry3f(Translation3f(ik_center))) ;
    GeometryPtr geom(new BoxGeometry({0.01, 0.01, 0.01})) ;
    PhongMaterial *material = new PhongMaterial({1, 0, 1}, 0.5) ;
    MaterialPtr mat(material) ;
    target_->addDrawable(geom, mat) ;
    robot_node->addChild(target_) ;

    gizmo_.reset(new TransformManipulator(camera_, 0.15)) ;
    gizmo_->gizmo()->show(true) ;
    gizmo_->gizmo()->setOrder(2) ;

    gizmo_->setCallback([this, physics, m](TransformManipulatorEvent e, const Affine3f &f) {
        if ( e == TRANSFORM_MANIP_MOTION_ENDED )  {
            Isometry3f p = Isometry3f::Identity() ;
            p.translation() = f.translation()  ;
            p.linear() = m ;

            robot_->getJointState(start_state_) ;
        } else if ( e == TRANSFORM_MANIP_MOVING ) {
  //          cout << f.translation().adjoint() << endl ;
        }

    });

   gizmo_->attachTo(target_.get());
    gizmo_->setLocalTransform(false);

    runenv() ;
}

void GUI::runenv() {
    ExecuteEnvironmentThread *workerThread = new ExecuteEnvironmentThread((World *)physics_) ;
    connect(workerThread, &ExecuteEnvironmentThread::updateScene, this, [this]() { update();});
    connect(workerThread, &ExecuteEnvironmentThread::finished, workerThread, &QObject::deleteLater);
    workerThread->start();
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
