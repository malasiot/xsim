#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include "world.hpp"
#include "kuka_iiwa_ik_solver.hpp"

#include <iostream>

#include <QTimer>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTcpSocket>
#include <QTcpServer>
#include <QGuiApplication>
#include <QWindow>
#include <QScreen>
#include <QStandardPaths>
#include <QDir>
#include <QPainter>

#include <xviz/scene/node_helpers.hpp>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(World *world): SimulationGui(world), world_(world) {

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

    auto viz = world->getVisual() ;
    auto robot1_node = viz->findNodeByName("r1_iiwa_base") ;

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &GUI::grabScreen);
    count_ = 0;
    title_ = MainWindow::instance()->windowTitle() ;

    Matrix3f m1(AngleAxisf(-M_PI/2, Vector3f::UnitX())) ;
    Matrix3f m2(AngleAxisf(-M_PI/2, Vector3f::UnitX())) ;

    target1_.reset(new Node) ;
    GeometryPtr geom(new BoxGeometry({0.01, 0.01, 0.01})) ;
    PhongMaterial *material = new PhongMaterial({1, 0, 1}, 0.5) ;
    MaterialPtr mat(material) ;
    target1_->addDrawable(geom, mat) ;
    robot1_node->addChild(target1_) ;

    gizmo1_.reset(new TransformManipulator(camera_, 0.1)) ;
    gizmo1_->gizmo()->show(true) ;
    gizmo1_->gizmo()->setOrder(2) ;

    gizmo1_->setCallback([this, m1](TransformManipulatorEvent e, const Affine3f &f) {
        if ( e == TRANSFORM_MANIP_MOTION_ENDED )  {
            Isometry3f p = Isometry3f::Identity() ;
            p.translation() = f.translation()  ;
            p.linear() = m1 ;
          //  p.translation() = Vector3f{0, 0.45, 0.05};


            KukaIKSolver solver ;

            KukaIKSolver::Problem ik(p ) ;

            string prefix("r1_") ;

            std::vector<JointCoeffs> solutions ;
            if ( solver.solve(ik,  solutions) ) {
                for( const auto solution: solutions ) {
                    JointState state ;
                    for( uint j=0 ; j<7 ; j++ ) {
                        state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
                    }

                    if ( world_->isStateValid(state, world_->getJointState(World::R2)) ) {
                        world_->setJointState(World::R1, state) ;
                         world_->stepSimulation(0.05);
                         break ;
                    }
                }
            }
        } else if ( e == TRANSFORM_MANIP_MOVING ) {
            //          cout << f.translation().adjoint() << endl ;
        }



    });


    gizmo1_->attachTo(target1_.get());
    gizmo1_->setLocalTransform(true);

    auto robot2_node = viz->findNodeByName("r2_iiwa_base") ;

    target2_.reset(new Node) ;
    target2_->addDrawable(geom, mat) ;
    robot2_node->addChild(target2_) ;

    gizmo2_.reset(new TransformManipulator(camera_, 0.05)) ;
    gizmo2_->gizmo()->show(true) ;
    gizmo2_->gizmo()->setOrder(2) ;

    gizmo2_->setCallback([this, m2](TransformManipulatorEvent e, const Affine3f &f) {
        if ( e == TRANSFORM_MANIP_MOTION_ENDED )  {
            Isometry3f p = Isometry3f::Identity() ;
            p.translation() = f.translation()  ;
            p.linear() = m2 ;
          //  p.translation() = Vector3f{0, 0.45, 0.05};


            KukaIKSolver solver ;

            KukaIKSolver::Problem ik(p ) ;

            string prefix("r2_") ;

            std::vector<JointCoeffs> solutions ;
            if ( solver.solve(ik,  solutions) ) {
                for( const auto solution: solutions ) {
                    JointState state ;
                    for( uint j=0 ; j<7 ; j++ ) {
                        state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
                    }

                    if ( world_->isStateValid( world_->getJointState(World::R1), state) ) {
                        world_->setJointState(World::R2, state) ;
                         world_->stepSimulation(0.05);
                         break ;
                    }
                }
            }
        } else if ( e == TRANSFORM_MANIP_MOVING ) {
            //          cout << f.translation().adjoint() << endl ;
        }



    });


    gizmo2_->attachTo(target2_.get());
    gizmo2_->setLocalTransform(true);

}



void GUI::setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius)
{
    auto target = physics_->findRigidBody(box) ;
    NodePtr tnode = target->visual() ;

    PhongMaterial *mat = static_cast<PhongMaterial *>(tnode->drawables()[0].material().get()) ;
    mat->setDiffuseColor({0.7, 0.2, 0.2}) ;


    NodePtr circle = NodeHelpers::makeCircle({pos.x(), pos.y(), 0.01}, {0, 0, 1}, radius, {1, 0, 0}) ;

    scene_->addChild(circle) ;

}


void GUI::onUpdate(float delta) {
    SimulationGui::onUpdate(delta) ;

    //     vector<ContactResult> results ;
    //    physics.contactPairTest(target_, table_mb->getLink("baseLink"), 0.01, results) ;
}

void GUI::mousePressEvent(QMouseEvent *event) {
    if ( gizmo1_->onMousePressed(event) ) {
        update() ;
        return ;
    }

    if ( gizmo2_->onMousePressed(event) ) {
        update() ;
        return ;
    }

    SceneViewer::mousePressEvent(event) ;
}

void GUI::mouseReleaseEvent(QMouseEvent *event) {
    if ( gizmo1_->onMouseReleased(event) ) {
        update() ;
        return ;
    }

    if ( gizmo2_->onMouseReleased(event) ) {
        update() ;
        return ;
    }

    SceneViewer::mouseReleaseEvent(event) ;
}

void GUI::mouseMoveEvent(QMouseEvent *event) {
    if ( gizmo1_->onMouseMoved(event) ) {
        update() ;
        return ;
    }

    if ( gizmo2_->onMouseMoved(event) ) {
        update() ;
        return ;
    }
    SceneViewer::mouseMoveEvent(event) ;
}


void GUI::startRecording() {
    stopRecording() ;
    count_ = 0 ;
    timer_->start(200) ;
    MainWindow::instance()->setWindowTitle(title_ + " (recording)");

}


void GUI::grabScreen() {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (const QWindow *window = windowHandle())
        screen = window->screen();
    if (!screen)
        return;

    auto image = grabFramebuffer();

    if (grab_frame_path_.isEmpty()) {
        grab_frame_path_ = QDir::currentPath();
        grab_frame_path_ += "/grab" ;
    }

    QString file_name = QString("%1_%2.png").arg(grab_frame_path_).arg((int)count_, 4, 10, QLatin1Char('0'));


    if (!image.save(file_name)) {
        qDebug() << "The image could not be saved to " << QDir::toNativeSeparators(file_name);
    }

    count_++;
}

void GUI::stopRecording() {
    timer_->stop();
    MainWindow::instance()->setWindowTitle(title_);
}


void GUI::keyPressEvent(QKeyEvent *event) {
    int key = event->key() ;
    if ( key == Qt::Key_R ) {
        startRecording() ;
    } else if ( key == Qt::Key_S ) {
        stopRecording() ;
    } else
        SceneViewer::keyPressEvent(event);
}

