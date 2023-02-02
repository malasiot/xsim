find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)
include_directories(${BULLET_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(Qt5 COMPONENTS Core Gui Network Widgets REQUIRED)

add_executable(push_env
    run_env.cpp
    mainwindow.hpp
    mainwindow.cpp
    bullet_gui.cpp
    bullet_gui.hpp
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
set_target_properties(push_env PROPERTIES AUTOMOC TRUE)
target_link_libraries(push_env xsim cvx ${BULLET_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Network Qt5::Widgets)
