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
#include "server.hpp"
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

    std::unique_ptr<World> world(new World(params["world"])) ;
    std::unique_ptr<Environment> env(new Environment(params["environment"], world.get())) ;
    std::unique_ptr<DQNAgent> agent(new DQNAgent(env.get(), params["agent"])) ;
    std::unique_ptr<Trainer> trainer(new Trainer(agent.get(), params["trainer"]));

    GUI *gui = new GUI(world.get()) ;
    gui->setTarget(env->params().target_, env->params().target_pos_, env->params().target_radius_) ;
#if 0
    ExecuteEnvironmentThread *workerThread = new ExecuteEnvironmentThread(env.get()) ;
    QObject::connect(workerThread, &ExecuteEnvironmentThread::updateScene, gui, [gui]() { gui->update();});
    QObject::connect(workerThread, &ExecuteEnvironmentThread::finished, workerThread, [workerThread](){ delete workerThread ;});
workerThread->start();
#endif
    TrainingThread *workerThread = new TrainingThread(trainer.get()) ;
    QObject::connect(workerThread, &TrainingThread::updateScene, gui, [gui]() { gui->update();});
    QObject::connect(workerThread, &TrainingThread::finished, workerThread, [workerThread](){ delete workerThread ;});
    workerThread->start() ;

    MainWindow window ;
    window.setGui(gui) ;
    window.resize(1024, 1024) ;

    SimulationServer server ;

    window.show() ;
    return app.exec();

}
