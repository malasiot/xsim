#include <xsim/collision_space.hpp>
#include <iostream>

using namespace xsim ;
using namespace Eigen ;
using namespace std ;
using namespace xviz ;

vector<pair<string, string>> disabled_pairs = {

    {"base_link" ,"shoulder_link" },
    {"base_link" , "upper_arm_link"},
      { "ee_link" ,"robotiq_85_base_link" },
      { "ee_link" ,"robotiq_85_left_finger_link" },
      { "ee_link" ,"robotiq_85_left_finger_tip_link" },
      { "ee_link" ,"robotiq_85_left_inner_knuckle_link" },
      { "ee_link" ,"robotiq_85_left_knuckle_link" },
      { "ee_link" ,"robotiq_85_right_finger_link" },
      { "ee_link" ,"robotiq_85_right_finger_tip_link" },
      { "ee_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "ee_link" ,"robotiq_85_right_knuckle_link" },
      { "ee_link" ,"robotiq_coupler" },
      { "ee_link" ,"wrist_1_link" },
      { "ee_link" ,"wrist_2_link" },
      { "ee_link" ,"wrist_3_link" },
      { "forearm_link" ,"upper_arm_link" },
      { "forearm_link" ,"wrist_1_link" },
      { "forearm_link" ,"wrist_2_link" },
      { "forearm_link" ,"wrist_3_link" },
      { "robotiq_85_base_link" ,"robotiq_85_left_finger_link" },
      { "robotiq_85_base_link" ,"robotiq_85_left_finger_tip_link" },
      { "robotiq_85_base_link" ,"robotiq_85_left_inner_knuckle_link" },
      { "robotiq_85_base_link" ,"robotiq_85_left_knuckle_link" },
      { "robotiq_85_base_link" ,"robotiq_85_right_finger_link" },
      { "robotiq_85_base_link" ,"robotiq_85_right_finger_tip_link" },
      { "robotiq_85_base_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_base_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_base_link" ,"robotiq_coupler" },
      { "robotiq_85_base_link" ,"wrist_1_link" },
      { "robotiq_85_base_link" ,"wrist_2_link" },
      { "robotiq_85_base_link" ,"wrist_3_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_left_finger_tip_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_left_inner_knuckle_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_left_knuckle_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_right_finger_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_right_finger_tip_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_left_finger_link" ,"robotiq_coupler" },
      { "robotiq_85_left_finger_link" ,"wrist_1_link" },
      { "robotiq_85_left_finger_link" ,"wrist_2_link" },
      { "robotiq_85_left_finger_link" ,"wrist_3_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_85_left_inner_knuckle_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_85_left_knuckle_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_85_right_finger_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_85_right_finger_tip_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_left_finger_tip_link" ,"robotiq_coupler" },
      { "robotiq_85_left_finger_tip_link" ,"wrist_1_link" },
      { "robotiq_85_left_finger_tip_link" ,"wrist_2_link" },
      { "robotiq_85_left_finger_tip_link" ,"wrist_3_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"robotiq_85_left_knuckle_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"robotiq_85_right_finger_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"robotiq_85_right_finger_tip_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"robotiq_coupler" },
      { "robotiq_85_left_inner_knuckle_link" ,"wrist_1_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"wrist_2_link" },
      { "robotiq_85_left_inner_knuckle_link" ,"wrist_3_link" },
      { "robotiq_85_left_knuckle_link" ,"robotiq_85_right_finger_link" },
      { "robotiq_85_left_knuckle_link" ,"robotiq_85_right_finger_tip_link" },
      { "robotiq_85_left_knuckle_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_left_knuckle_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_left_knuckle_link" ,"robotiq_coupler" },
      { "robotiq_85_left_knuckle_link" ,"wrist_1_link" },
      { "robotiq_85_left_knuckle_link" ,"wrist_2_link" },
      { "robotiq_85_left_knuckle_link" ,"wrist_3_link" },
      { "robotiq_85_right_finger_link" ,"robotiq_85_right_finger_tip_link" },
      { "robotiq_85_right_finger_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_right_finger_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_right_finger_link" ,"robotiq_coupler" },
      { "robotiq_85_right_finger_link" ,"wrist_1_link" },
      { "robotiq_85_right_finger_link" ,"wrist_2_link" },
      { "robotiq_85_right_finger_link" ,"wrist_3_link" },
      { "robotiq_85_right_finger_tip_link" ,"robotiq_85_right_inner_knuckle_link" },
      { "robotiq_85_right_finger_tip_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_right_finger_tip_link" ,"robotiq_coupler" },
      { "robotiq_85_right_finger_tip_link" ,"wrist_1_link" },
      { "robotiq_85_right_finger_tip_link" ,"wrist_2_link" },
      { "robotiq_85_right_finger_tip_link" ,"wrist_3_link" },
      { "robotiq_85_right_inner_knuckle_link" ,"robotiq_85_right_knuckle_link" },
      { "robotiq_85_right_inner_knuckle_link" ,"robotiq_coupler" },
      { "robotiq_85_right_inner_knuckle_link" ,"wrist_1_link" },
      { "robotiq_85_right_inner_knuckle_link" ,"wrist_2_link" },
      { "robotiq_85_right_inner_knuckle_link" ,"wrist_3_link" },
      { "robotiq_85_right_knuckle_link" ,"robotiq_coupler" },
      { "robotiq_85_right_knuckle_link" ,"wrist_1_link" },
      { "robotiq_85_right_knuckle_link" ,"wrist_2_link" },
      { "robotiq_85_right_knuckle_link" ,"wrist_3_link" },
      { "robotiq_coupler" ,"wrist_1_link" },
      { "robotiq_coupler" ,"wrist_2_link" },
      { "robotiq_coupler" ,"wrist_3_link" },
      { "shoulder_link" ,"upper_arm_link" },
      { "wrist_1_link" ,"wrist_2_link" },
      { "wrist_1_link" ,"wrist_3_link" },
      { "wrist_2_link" ,"wrist_3_link" }
};

int main(int argc, char *argv[]) {
    CollisionSpace space ;

    for( const auto &cp : disabled_pairs ) {
        space.disableCollision(cp.first, cp.second) ;
    }

    CollisionShapePtr box(new BoxCollisionShape{{0.15, 0.15, 0.15}}) ;

    space.addCollisionObject("box1", box, Isometry3f(Translation3f{{0, 0, 0.2}})) ;
    space.addCollisionObject("box2", box, Isometry3f(Translation3f{{0, 0, 0.6}})) ;

    // load URDFs
    string path = "/home/malasiot/source/xsim/data/" ;
    URDFRobot robot = URDFRobot::load(path + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;
    robot.setJointPosition("elbow_joint", 0.5) ;
    robot.setJointPosition("shoulder_lift_joint", -.54) ;
   // robot.setJointPosition("shoulder_pan_joint", 0) ;
   // robot.setJointPosition("wrist_1_joint", -1.57) ;
   // robot.setJointPosition("wrist_2_joint", -1.57) ;
   // robot.setJointPosition("wrist_3_joint", 0) ;


  //  robot.setWorldTransform(Isometry3f(Translation3f{-0.1, -0.2, 0.67}));

    space.addRobot(robot) ;
    cout << space.hasCollision() << endl ;
}
