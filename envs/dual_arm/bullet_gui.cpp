#include "bullet_gui.hpp"
#include <xviz/scene/scene.hpp>
#include <QTimer>

using namespace xviz ;
using namespace xsim ;


SimulationGui::SimulationGui(PhysicsWorld *physics): SceneViewer(physics->getVisual()), physics_(physics), picker_(*physics) {

    auto scene = physics->getVisual() ;

    auto c = scene->geomCenter();
    initCamera(c, scene->geomRadius(c)) ;

    camera_->setBgColor({1, 1, 1, 1}) ;

    startAnimations() ;
}

SimulationGui::SimulationGui(PhysicsWorld *physics, const Eigen::Vector3f &c, float r): SceneViewer(physics->getVisual()), physics_(physics), picker_(*physics) {
    initCamera(c, r) ;

    camera_->setBgColor({1, 1, 1, 1}) ;

    startAnimations() ;
}

void SimulationGui::mousePressEvent(QMouseEvent *event)
{
    if ( event->modifiers() & Qt::ControlModifier ) {
        Ray ray = camera_->getRay(event->x(), event->y()) ;

        picker_.pickBody(ray);

        picking_ = true ;

    } else {
        SceneViewer::mousePressEvent(event) ;
    }


}

void SimulationGui::mouseReleaseEvent(QMouseEvent *event)
{
    if ( event->modifiers() & Qt::ControlModifier ) {
        picker_.removePickingConstraint();
        picking_ = false ;
    } else {
        SceneViewer::mouseReleaseEvent(event) ;
    }

}

void SimulationGui::mouseMoveEvent(QMouseEvent *event)
{
    int x = event->x() ;
    int y = event->y() ;

    if ( picking_ )  {
        Ray ray = camera_->getRay(x, y) ;
        picker_.movePickedBody(ray) ;

        update() ;
    }
    else {
        SceneViewer::mouseMoveEvent(event) ;

    }

}

