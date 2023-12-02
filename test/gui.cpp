#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include "planning_interface.hpp"

#include <xviz/scene/node_helpers.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/scene.hpp>

#include <iostream>

#include <QTimer>
#include <QDebug>
#include <QGuiApplication>
#include <QDir>
#include <QPainter>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(World *w): SceneViewer(w->getVisual()),
     world_(w) {

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;



    NodePtr circle = NodeHelpers::makeCircle({0, 0.3, 0.01}, {0, 0, 1}, 0.05, {1, 0, 0}) ;

    scene_->addChild(circle) ;

    auto world = w->getVisual() ;
    auto robot_node = world->findNodeByName("base_link") ;

    traj_node_.reset(new Node) ;
     for( int i=0 ;i<100 ; i++ ) {
         traj_points_[i] =xviz::NodeHelpers::makeSphere(0.01, Vector4f(0, 0, 1, 1));
         traj_node_->addChild(traj_points_[i]) ;
     }

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

     gizmo_.reset(new TransformManipulator(camera_, 0.05)) ;
     gizmo_->gizmo()->show(false) ;
     gizmo_->gizmo()->setOrder(2) ;

     gizmo_->setCallback([this, m](TransformManipulatorEvent e, const Affine3f &f) {
         if ( e == TRANSFORM_MANIP_MOTION_ENDED )  {
             Isometry3f p = Isometry3f::Identity() ;
             p.translation() = f.translation()  ;
             p.linear() = m ;
            p.translation() = Vector3f{0, 0.4, 0.05};

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

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateSensors()));
    timer->start(30);

    auto cam = world_->camera()->makeVisual(0.5) ;
    scene_->addChild(cam) ;



}

void GUI::onUpdate(float delta) {
    SceneViewer::onUpdate(delta) ;

    //     vector<ContactResult> results ;
    //    physics.contactPairTest(target_, table_mb->getLink("baseLink"), 0.01, results) ;
}

void GUI::keyPressEvent(QKeyEvent *event) {
    if ( event->key() == Qt::Key_G ) {

            uint width = 1024, height = 1024 ;
            OffscreenSurface os(QSize(width, height)) ;


            PerspectiveCamera *pcam = new PerspectiveCamera(1, // aspect ratio
                                                            50*M_PI/180,   // fov
                                                            0.00001,        // zmin
                                                            10           // zmax
                                                            ) ;


         //   OrthographicCamera *pcam = new OrthographicCamera(-0.6*r, 0.6*r, 0.6*r, -0.6*r,0.0001, 10*r) ;

            CameraPtr cam(pcam) ;

            cam->setBgColor({1, 0, 0, 1});

            // position camera to look at the center of the object

          //  pcam->viewSphere(c, r) ;
            pcam->lookAt({0, 0, 2}, {0, 0.5, 0}, {0, 0, 1}) ;

            // set camera viewpot

            pcam->setViewport(width, height)  ;


            Renderer rdr ;

            rdr.render(scene_, cam) ;
            auto im = os.getImage() ;
            im.saveToPNG("/tmp/im.png") ;

        }

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

bool done = false ;
void GUI::trajectory(const JointTrajectory &traj) {
    if ( done ) return ;
    showTrajectory(traj);
    if ( traj_thread_ ) return ;

    ExecuteTrajectoryThread *workerThread = new ExecuteTrajectoryThread(world_, traj) ;
    connect(workerThread, &ExecuteTrajectoryThread::updateScene, this, [this]() { update();});
    connect(workerThread, &ExecuteTrajectoryThread::finished, workerThread, [this](){
        JointTrajectory mtraj ;
        delete traj_thread_ ; traj_thread_ = nullptr;
        world_->disableToolCollisions();
        if ( world_->planner()->planRelative(world_->controller()->getJointState(), Vector3f{0, 0.2, 0}, mtraj)) {
            trajectory(mtraj) ;
            done = true ;
        }
        world_->enableToolCollisions();
    });
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

void GUI::paintGL()
{
    SceneViewer::paintGL() ;
}

#include "imconv.hpp"
void GUI::updateSensors() {

    emit imageCaptured(imageToQImage(world_->camera()->capture().second)) ;
}


