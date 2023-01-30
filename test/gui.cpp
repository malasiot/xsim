#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include "planning_interface.hpp"

#include <xviz/scene/node_helpers.hpp>

#include <iostream>

#include <QTimer>
#include <QDebug>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(World *w): SceneViewer(w->getVisual()),
     world_(w) {

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

    auto world = w->getVisual() ;
    auto robot_node = world->findNodeByName("base_link") ;

    traj_node_.reset(new Node) ;
     for( int i=0 ;i<100 ; i++ ) {
         traj_points_[i] =xviz::NodeHelpers::makeSphere(0.01, Vector4f(0, 0, 1, 1));
         traj_node_->addChild(traj_points_[i]) ;
     }
 #if 0
     scene_->children()[0]->setVisible(false);

     auto cs = world_->collisionScene() ;
     KinematicModel &fk = static_cast<UR5Planning *>(world_->iplan())->fk() ;

     fk.setJointState(world_->controller()->getJointState()) ;
     map<string , Isometry3f> trs = fk.getLinkTransforms();
     cs->updateTransforms(trs);

     cout << trs["upper_arm_link"].matrix() << endl ;

     scene_->addChild(world_->collisionScene()) ;
 #endif
     scene_->addChild(traj_node_) ;
     traj_node_->setVisible(false) ;
     //    physics.setCollisionFeedback(this);

     Matrix3f m(AngleAxisf(M_PI, Vector3f::UnitY())) ;

     target_.reset(new Node) ;
     GeometryPtr geom(new BoxGeometry({0.01, 0.01, 0.01})) ;
     PhongMaterial *material = new PhongMaterial({1, 0, 1}, 0.5) ;
     MaterialPtr mat(material) ;
     target_->addDrawable(geom, mat) ;
     robot_node->addChild(target_) ;

     gizmo_.reset(new TransformManipulator(camera_, 0.15)) ;
     gizmo_->gizmo()->show(true) ;
     gizmo_->gizmo()->setOrder(2) ;

     gizmo_->setCallback([this, m](TransformManipulatorEvent e, const Affine3f &f) {
         if ( e == TRANSFORM_MANIP_MOTION_ENDED )  {
             Isometry3f p = Isometry3f::Identity() ;
             p.translation() = f.translation()  ;
             p.linear() = m ;

             cout << p.translation().adjoint() << endl ;
             JointTrajectory traj ;
             if ( world_->planner()->plan(world_->controller()->getJointState(), p, traj) ) {
                 trajectory(traj) ;
             }
 #if 0
             vector<JointState> solutions ;

            if ( world_->iplan()->solveIK(p, solutions) ) {
                for( const auto solution: solutions ) {
                    if ( world_->iplan()->isStateValid(solution) ) {
                         world_->controller()->setJointState(solution) ;
                         world_->stepSimulation(0.05);
                         break ;
                    }
                }
            } else  return ;
 #endif
 return ;
         } else if ( e == TRANSFORM_MANIP_MOVING ) {
    //          cout << f.translation().adjoint() << endl ;
          }



      });


    gizmo_->attachTo(target_.get());
    gizmo_->setLocalTransform(true);



}

void GUI::onUpdate(float delta) {
    SceneViewer::onUpdate(delta) ;
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
    else SceneViewer::keyPressEvent(event) ;

    update() ;

}

void GUI::mousePressEvent(QMouseEvent *event) {
    if ( gizmo_->onMousePressed(event) ) {
        update() ;
        return ;
    }

    SceneViewer::mousePressEvent(event) ;
}

void GUI::mouseReleaseEvent(QMouseEvent *event) {
    if ( gizmo_->onMouseReleased(event) ) {
        update() ;
        return ;
    }

    SceneViewer::mouseReleaseEvent(event) ;
}

void GUI::mouseMoveEvent(QMouseEvent *event) {
    if ( gizmo_->onMouseMoved(event) ) {
        update() ;
        return ;
    }

    SceneViewer::mouseMoveEvent(event) ;
}

void GUI::processContact(ContactResult &r) {
    if ( r.a_ == nullptr || r.b_ == nullptr ) return ;
    if ( r.a_->getName() == "ground" || r.b_->getName() == "ground" ) return  ;
    cout << r.a_->getName() << ' ' << r.b_->getName() << endl ;
}

void GUI::changeControlValue(const std::string &jname, float v) {
    cout << jname << ' ' << v << endl ;
    //robot_mb->setJointPosition(jname, v) ;
    world_->controller()->setJointState(jname, v) ;
}

void GUI::updateControls(const JointState &state)
{
   update() ;
}


void GUI::trajectory(const JointTrajectory &traj) {
    showTrajectory(traj);
    if ( traj_thread_ ) return ;

    ExecuteTrajectoryThread *workerThread = new ExecuteTrajectoryThread(world_, traj) ;
    connect(workerThread, &ExecuteTrajectoryThread::updateScene, this, [this]() { update();});
    connect(workerThread, &ExecuteTrajectoryThread::finished, workerThread, [this](){ delete traj_thread_ ; traj_thread_ = nullptr;});
    workerThread->start();
    traj_thread_ = workerThread ;
}

void GUI::showTrajectory(const xsim::JointTrajectory &traj) {

    for( int i=0 ; i<100 ; i++ ) {
        float t = i/100.0f ;
        JointState state = traj.getState(t, world_->iplan()) ;
        auto pose = world_->iplan()->getToolPose(state) ;
        Isometry3f tp = Isometry3f::Identity() ;
        tp.translation() = pose.translation() ;
        traj_points_[i]->setTransform(tp) ;
    }

    traj_node_->setVisible(true) ;
}
