#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include "world.hpp"
#include "planning_interface.hpp"

#include <iostream>

#include <QTimer>
#include <QDebug>

#include <xviz/scene/node_helpers.hpp>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(World *w):
    SimulationGui(w), world_(w) {

    physics_->stepSimulation(0.005);

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

    auto world = physics_->getVisual() ;
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
           if ( world_->planner()->planRelative(world_->controller()->getJointState(), Vector3f{0, 0.2, 0}, traj )) {
               trajectory(traj) ;
           }


#if 0
            Vector3f p1{ -0.1,   0.39,      0.02} ;
            Vector3f p2{ -0.1,   0.475,      0.02};

            p.translation() = p1 ;

            JointState seed, solution ;
            seed = world_->controller()->getJointState() ;
           if ( world_->iplan()->solveIK(p, seed, solution) ) {
                world_->controller()->setJointState(solution) ;
                world_->stepSimulation(0.05);
           }

           auto start_state = world_->controller()->getJointState() ;

            JointTrajectory traj ;

            world_->disableToolCollisions("box_0_0");

            if ( world_->planner()->planRelative(start_state, p2-p1, traj)) {
                cout << traj << endl ;
                trajectory(traj) ;
            }
           #endif
/*
            auto start_state = world_->controller()->getJointState() ;
            if ( world_->planner()->plan(start_state, p, traj) ) {
               trajectory(traj) ;
            }
*/
  //        -0.133433   0.49995      0.02

        } else if ( e == TRANSFORM_MANIP_MOVING ) {
  //          cout << f.translation().adjoint() << endl ;
        }

    });

   gizmo_->attachTo(target_.get());
    gizmo_->setLocalTransform(false);

  //runenv() ;

    qRegisterMetaType<JointTrajectory>("xsim::JointTrajectory");
}

void GUI::runenv() {
    ExecuteEnvironmentThread *workerThread = new ExecuteEnvironmentThread(world_) ;
    connect(workerThread, &ExecuteEnvironmentThread::updateScene, this, [this]() { update();});
    connect(workerThread, SIGNAL(showTrajectory(xsim::JointTrajectory)), this, SLOT(showTrajectory(xsim::JointTrajectory)));
    connect(workerThread, &ExecuteEnvironmentThread::finished, workerThread, [&](){ delete workerThread ;});
    workerThread->start();
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
