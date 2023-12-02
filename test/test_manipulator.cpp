#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>

#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xsim/robot_scene.hpp>
#include <xviz/gui/manipulator.hpp>
#include <xviz/gui/viewer.hpp>

#include <iostream>
#include <thread>

#include <xsim/world.hpp>

#include <xsim/multi_body.hpp>
#include <xsim/soft_body.hpp>
#include <xsim/soft_body_shape.hpp>

#include <QApplication>
#include <QMainWindow>

#include "mainwindow.hpp"
#include <xsim/joint_state_planner.hpp>
#include <xsim/collision_space.hpp>
#include <xsim/kinematic.hpp>

#include "gui.hpp"
#include "world.hpp"
#include "robot.hpp"

using namespace xsim ;
using namespace xviz ;
using namespace std ;
using namespace Eigen ;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;

    MainWindow window ;
    window.setWindowTitle("test_manipulator");

    World *world = new World("/home/malasiot/source/xsim/data/") ;

    GUI *gui = new GUI(world) ;
    gui->setGrabFramePath("/tmp/grab");

    window.setGui(gui) ;
    window.resize(1024, 1024) ;


    QObject::connect(&window, &MainWindow::controlValueChanged, gui, &GUI::changeControlValue) ;
    QObject::connect(gui, &GUI::imageCaptured, &window, &MainWindow::updateImage) ;


    for( const auto &jname: world->iplan()->getJointChain() ) {
        double l, u ;
        world->iplan()->getLimits(jname, l, u) ;
        window.addSlider(jname, l, u) ;
    }
    window.endSliders() ;

    window.show() ;
    return app.exec();
}
