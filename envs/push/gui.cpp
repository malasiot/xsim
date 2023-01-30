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

GUI::GUI(xsim::PhysicsWorld *world):
    SimulationGui(world) {

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

 //   runenv() ;

}


void GUI::onUpdate(float delta) {
    SimulationGui::onUpdate(delta) ;
    //     vector<ContactResult> results ;
    //    physics.contactPairTest(target_, table_mb->getLink("baseLink"), 0.01, results) ;
}
