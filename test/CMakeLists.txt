include_directories(
    3rdparty/
    ${BULLET_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
 )

add_executable(test_bullet test_bullet.cpp bullet_gui.cpp bullet_gui.hpp body.cpp body.hpp)
target_link_libraries(test_bullet xsim ${BULLET_LIBRARIES})

add_executable(test_rope test_rope.cpp bullet_gui.cpp bullet_gui.hpp)
set_target_properties(test_rope PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_rope xsim ${BULLET_LIBRARIES})

add_executable(test_gripper test_gripper.cpp bullet_gui.cpp bullet_gui.hpp)
set_target_properties(test_gripper PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_gripper xsim ${BULLET_LIBRARIES})

#add_executable(test_cartpole test_cartpole.cpp bullet_gui.cpp bullet_gui.hpp)
#set_target_properties(test_cartpole PROPERTIES AUTOMOC TRUE)
#target_link_libraries(test_cartpole xsim ${BULLET_LIBRARIES})

add_executable(test_bullet_robot test_robot.cpp bullet_gui.cpp bullet_gui.hpp)
set_target_properties(test_bullet_robot PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_bullet_robot xsim ${BULLET_LIBRARIES})

add_executable(test_two_joints test_two_joints.cpp bullet_gui.cpp bullet_gui.hpp)
set_target_properties(test_two_joints PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_two_joints xsim ${BULLET_LIBRARIES})

add_executable(test_cloth test_cloth.cpp bullet_gui.cpp bullet_gui.hpp)
set_target_properties(test_cloth PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_cloth xsim ${BULLET_LIBRARIES})

add_executable(test_vehicle test_vehicle.cpp bullet_gui.cpp bullet_gui.hpp)
set_target_properties(test_vehicle PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_vehicle xsim ${BULLET_LIBRARIES})


add_executable(test_manipulator
    test_manipulator.cpp
    mainwindow.hpp
    mainwindow.cpp
    bullet_gui.cpp
    bullet_gui.hpp
    ur5_ik_solver.cpp
    ur5_ik_solver.hpp
    world.cpp
    world.hpp

    planner.cpp
    planner.hpp
    planning_interface.cpp
    planning_interface.hpp
    gui.cpp
    gui.hpp
    robot.cpp
    robot.hpp
)
set_target_properties(test_manipulator PROPERTIES AUTOMOC TRUE)
target_link_libraries(test_manipulator xsim ${BULLET_LIBRARIES} )

add_executable(test_collisions test_collisions.cpp)
target_link_libraries(test_collisions xsim ${BULLET_LIBRARIES})

add_executable(test_planner test_planner.cpp
    test_planner.cpp
    ur5_ik_solver.cpp
    ur5_ik_solver.hpp
   )
target_link_libraries(test_planner xsim ${BULLET_LIBRARIES} )
