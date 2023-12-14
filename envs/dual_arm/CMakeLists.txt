find_package(OpenCV REQUIRED)
include_directories(${BULLET_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(Qt5 COMPONENTS Core Gui Network Widgets REQUIRED)

add_executable(dual_arm
    run_env.cpp
    mainwindow.hpp
    mainwindow.cpp
    bullet_gui.cpp
    bullet_gui.hpp
    kuka_iiwa_ik_solver.cpp
#    environment.cpp
#    environment.hpp
#    dqn.cpp
#    dqn.hpp
#    experience_replay.hpp
#    experience_replay.cpp
#    agent.cpp
#    agent.hpp
#    trainer.cpp
#    trainer.hpp
    world.cpp
    world.hpp
    gui.cpp
    gui.hpp
    player.cpp
    player.hpp

)
set_target_properties(dual_arm PROPERTIES AUTOMOC TRUE)
target_link_libraries(dual_arm xsim cvx ${BULLET_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Network Qt5::Widgets)

add_executable(reachability reachability.cpp kuka_iiwa_ik_solver.cpp)
target_link_libraries(reachability xsim cvx ${BULLET_LIBRARIES})
