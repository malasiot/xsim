find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)
include_directories(${BULLET_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(push_env
    test_manipulator.cpp
    mainwindow.hpp
    mainwindow.cpp
    bullet_gui.cpp
    bullet_gui.hpp
    ur5_ik_solver.cpp
    ur5_ik_solver.hpp
    environment.cpp
    environment.hpp
    dqn.cpp
    dqn.hpp
    experience_replay.hpp
    experience_replay.cpp
    trainer.cpp
    trainer.hpp

    gui.cpp
    gui.hpp
    world.cpp
    world.hpp
    robot.cpp
    robot.hpp
)
set_target_properties(push_env PROPERTIES AUTOMOC TRUE)
target_link_libraries(push_env xsim cvx ${BULLET_LIBRARIES} ${OpenCV_LIBRARIES} ${TORCH_LIBRARIES})
