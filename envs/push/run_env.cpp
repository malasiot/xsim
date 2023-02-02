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
#include <xsim/robot_scene.hpp>

#include <cvx/math/rng.hpp>
#include <cvx/misc/format.hpp>

#include <QApplication>
#include <QMainWindow>

#include "mainwindow.hpp"

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

    auto params = cvx::Variant::fromConfigFile(path + "/envs/push.cfg") ;

    auto env_params = params["environment"] ;
    std::unique_ptr<World> world(new World(params["world"])) ;
    std::unique_ptr<Player> env(new Player(env_params, world.get())) ;

    MainWindow window ;
    GUI *gui = new GUI(env.get()) ;
    gui->setTarget(env->params().target_,
                   { env_params.value("target.pos.x", 0.0).as<float>(),
                     env_params.value("target.pos.y", 0.0).as<float>() },
                     env_params.value("target.radius", 0.0).as<float>() ) ;
    gui->setGrabFramePath("/tmp/grab");


    window.setGui(gui) ;
    window.resize(1024, 1024) ;

    window.show() ;
    return app.exec();

}
