#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>

#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>
#include <xviz/scene/node_helpers.hpp>
#include <xviz/robot/robot_scene.hpp>
#include <xviz/gui/viewer.hpp>

#include <iostream>
#include <thread>

#include <xsim/world.hpp>

#include "bullet_gui.hpp"

#include <xsim/multi_body.hpp>
#include <xsim/soft_body.hpp>
#include <xsim/soft_body_shape.hpp>

#include <QApplication>
#include <QMainWindow>

#include "ur5_ik_solver.hpp"

using namespace xviz ;
using namespace xsim ;

using namespace std ;
using namespace Eigen ;

PhysicsWorld physics ;

MultiBodyPtr robot_mb, table_mb ;
RigidBodyPtr cube_rb ;
SoftBodyPtr cloth ;

static const char *arm_joint_names[] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

static const char *gripper_roll = "wrist_3_joint" ;
static const char *mimic_joint_names[] = {
    "robotiq_85_right_knuckle_joint",
                        "robotiq_85_left_inner_knuckle_joint",
                        "robotiq_85_right_inner_knuckle_joint",
                        "robotiq_85_left_finger_tip_joint",
                        "robotiq_85_right_finger_tip_joint"
};

static const char *gripper_main_control_joint_name = "robotiq_85_left_knuckle_joint";


void ik(MultiBody &body, Isometry3f &ee, float roll) {


    double jseed[6], j[6] ;

    for( uint i=0 ; i<6 ; i++ ) {
        double pos = body.getJointPosition(arm_joint_names[i]);
        jseed[i] = pos ;
    }

    UR5IKSolver solver ;
    if ( solver.solve(jseed, ee, roll, j) ) {

        for( uint i=0 ; i<6 ; i++ ) {
            Joint *ctrl = body.findJoint(arm_joint_names[i]) ;
            ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(0.9).setTargetPosition(j[i]));
        }
    }
}

SoftBodyPtr makeCloth() {
    Isometry3f tr = Isometry3f::Identity();
    tr.translate(Vector3f(-0.2, 0.3, 0.7)) ;

    SoftBodyShapePtr sbs = SoftBodyShape::fromMeshModel("/home/malasiot/local/bullet3/data/cloth_z_up.obj") ;

    MaterialPtr material(new PhongMaterial({0, 1, 1, 1}));
    material->setSide(Material::Side::Both) ;

    SoftBodyPtr sb = physics.addSoftBody(SoftBodyBuilder()
                        .setName("cloth")
                        .setShape(sbs)
                        .setMargin(0.05)
                        .setScale(0.3)
                        .makeVisualShape(material)
                        .setWorldTransform(tr)
                        .setMass(0.2));

    return sb ;
}

class GUI: public SimulationGui, CollisionFeedback {
public:
    GUI(xsim::PhysicsWorld &physics):
        SimulationGui(physics) {

        initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

//        physics.setCollisionFeedback(this);

        Quaternionf rot{0, -1, 0, 1};
        rot.normalize() ;

        Isometry3f pose ;
        pose.setIdentity() ;
        pose.linear() = rot.matrix() ;
        pose.translation() = Vector3f{ 0.25, 0.25, 0.2} ;

        ik(*robot_mb, pose, M_PI/2) ;

        openGripper() ;

    }

    void openGripper() {
        Joint *ctrl = robot_mb->findJoint(gripper_main_control_joint_name) ;

        MotorControl params(POSITION_CONTROL) ;
        params.setMaxVelocity(1.5);
        params.setMaxForce(0.1) ;
        params.setTargetPosition(0.001) ;
        ctrl->setMotorControl(params);

        for( Joint *mimic: ctrl->getMimicJoints()) {
            params.setTargetPosition(mimic->getMimicMultiplier() * 0.001) ;
            mimic->setMotorControl(params);
        }

    }

    void closeGripper() {
        Joint *ctrl = robot_mb->findJoint(gripper_main_control_joint_name) ;

        MotorControl params(POSITION_CONTROL) ;
        params.setMaxVelocity(1.5);
        params.setMaxForce(0.1) ;
        params.setTargetPosition(0.8) ;
        ctrl->setMotorControl(params);

        for( Joint *mimic: ctrl->getMimicJoints()) {
            params.setTargetPosition(mimic->getMimicMultiplier() * 0.8) ;
            mimic->setMotorControl(params);
        }

    }

    void onUpdate(float delta) override {
         SimulationGui::onUpdate(delta) ;

         vector<ContactResult> results ;
         physics.contactPairTest(cube_rb, table_mb->getLink("baseLink"), 0.01, results) ;
         cout << results.size() << endl ;
    }

    void keyPressEvent(QKeyEvent *event) override {
        Joint *joint = robot_mb->findJoint("robotiq_85_left_knuckle_joint") ;
        if ( event->key() == Qt::Key_Q ) {
            openGripper();

        } else if ( event->key() == Qt::Key_W ) {
            closeGripper() ;
        } else SimulationGui::keyPressEvent(event) ;

        update() ;

    }

    void processContact(ContactResult &r) override {
        if ( r.a_ == nullptr || r.b_ == nullptr ) return ;
        if ( r.a_->getName() == "ground" || r.b_->getName() == "ground" ) return  ;
        cout << r.a_->getName() << ' ' << r.b_->getName() << endl ;
    }

};


void createScene() {

    physics.createSoftMultiBodyDynamicsWorld();
    physics.setGravity({0, 0, -10});

    // load URDFs
    string path = "/home/malasiot/source/xviz/data/physics/" ;
    URDFRobot robot = URDFRobot::load(path + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;
    URDFRobot table = URDFRobot::load(path + "models/table.urdf");

    // ur5 + gripper
    robot_mb = physics.addMultiBody(MultiBodyBuilder(robot)
                                .setFixedBase()
                                .setLinearDamping(0.f)
                                .setAngularDamping(0.f)
                                .setWorldTransform(Isometry3f(Translation3f{-0.1, -0.2, 0.67}))
                                ) ;

    robot_mb->setJointPosition("shoulder_lift_joint", -0.6);

    // table
    table_mb = physics.addMultiBody(MultiBodyBuilder(table).setName("table"));

    // cloth

    cloth = makeCloth() ;

    // a cube to grasp

    cube_rb = physics.addRigidBody(RigidBodyBuilder()
                         .setMass(0.1)
                         .setCollisionShape(make_shared<BoxCollisionShape>(Vector3f(0.02, 0.02, 0.02)))
                         .makeVisualShape({1.0, 0.1, 0.6, 1})
                         .setName("cube")
                         .setWorldTransform(Isometry3f(Translation3f{0, 0, 0.75})));


}

int main(int argc, char **argv)
{
    createScene() ;

    QApplication app(argc, argv);

    SceneViewer::initDefaultGLContext() ;
   // ResourceLoader::instance().setLocalPath("/home/malasiot/source/xviz/data/physics/models/");

    QMainWindow window ;
    window.setCentralWidget(new GUI(physics)) ;
    window.resize(512, 512) ;
    window.show() ;

    return app.exec();
}
