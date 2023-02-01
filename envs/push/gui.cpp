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
