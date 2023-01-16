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

    runenv() ;

}

void GUI::runenv() {
    ExecuteEnvironmentThread *workerThread = new ExecuteEnvironmentThread(world_) ;
    connect(workerThread, &ExecuteEnvironmentThread::updateScene, this, [this]() { update();});
    connect(workerThread, &ExecuteEnvironmentThread::finished, workerThread, [workerThread](){ delete workerThread ;});
    workerThread->start();
}


void GUI::onUpdate(float delta) {
    SimulationGui::onUpdate(delta) ;
    //     vector<ContactResult> results ;
    //    physics.contactPairTest(target_, table_mb->getLink("baseLink"), 0.01, results) ;
}
