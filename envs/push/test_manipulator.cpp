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

#include <cvx/math/rng.hpp>
#include <cvx/misc/format.hpp>

#include <QApplication>
#include <QMainWindow>

#include "mainwindow.hpp"
#include "environment.hpp"

#include "gui.hpp"
#include "world.hpp"

cvx::RNG rng ;

using namespace xsim ;
using namespace xviz ;
using namespace std ;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;
    // ResourceLoader::instance().setLocalPath("/home/malasiot/source/xviz/data/physics/models/");

    // load URDFs
    string path = "/home/malasiot/source/xsim/data/" ;

    World::Parameters params ;
    params.data_dir_ = path ;

    World *world = new World(params) ;
#if 0
    Environment env(world) ;

    vector<string> boxes = env.getBoxNames() ;

    for( int  i = 0 ; i<100 ; i++ ) {
        State state = env.getState() ;
        PushAction a ;
        a.box_id_ = rng.choice(boxes) ; ;
        a.loc_ = rng.uniform(0, 11) ;
        cv::Mat im = env.renderState(a, state) ;
   //     cv::imwrite("/tmp/state.png", im) ;
        cv::imwrite(cvx::format("/tmp/state_{:03d}.png", i), im) ;
        if ( !env.apply(state, a) ) continue ;

    }
#endif
    GUI *gui = new GUI(world) ;

    MainWindow window ;
    window.setGui(gui) ;
    window.resize(1024, 1024) ;

    window.show() ;
    return app.exec();


    delete world ;
}
