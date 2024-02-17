
include_directories(
    3rdparty/
    ${BULLET_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
)

set(LIB_SOURCES
    world.cpp
    rigid_body.cpp
    constraints.cpp
    collision.cpp
    multi_body.cpp
    sensor.cpp
    soft_body.cpp
    multi_body_builder.cpp
    soft_body_shape.cpp
    collision_space.cpp
    kinematic.cpp
    urdf_loader.cpp
    urdf_robot.cpp
    robot_scene.cpp
    sdf_parser.cpp
    sdf_world.cpp

    3rdparty/pugi/pugixml.cpp

    ${LIBRARY_INCLUDE_DIR}/world.hpp
    ${LIBRARY_INCLUDE_DIR}/rigid_body.hpp
    ${LIBRARY_INCLUDE_DIR}/collision.hpp
    ${LIBRARY_INCLUDE_DIR}/convert.hpp
    ${LIBRARY_INCLUDE_DIR}/constraints.hpp
    ${LIBRARY_INCLUDE_DIR}/multi_body.hpp
    ${LIBRARY_INCLUDE_DIR}/sensor.hpp
    ${LIBRARY_INCLUDE_DIR}/soft_body.hpp
    ${LIBRARY_INCLUDE_DIR}/multi_body_builder.hpp
    ${LIBRARY_INCLUDE_DIR}/collision_space.hpp
    ${LIBRARY_INCLUDE_DIR}/kinematic.hpp
    ${LIBRARY_INCLUDE_DIR}/urdf_robot.hpp
    ${LIBRARY_INCLUDE_DIR}/urdf_loader.hpp
    ${LIBRARY_INCLUDE_DIR}/robot_scene.hpp

)

if ( OMPL_FOUND )
list(APPEND LIB_SOURCES
    ompl_planner.cpp
    joint_state_planner.cpp
    task_space_planner.cpp
    task_space.cpp
    goal_region.cpp
    ${LIBRARY_INCLUDE_DIR}/ompl_planner.hpp
    ${LIBRARY_INCLUDE_DIR}/joint_state_planner.hpp
    ${LIBRARY_INCLUDE_DIR}/task_space_planner.hpp
    ${LIBRARY_INCLUDE_DIR}/task_space.hpp
    ${LIBRARY_INCLUDE_DIR}/goal_region.hpp
)
endif (OMPL_FOUND)

add_library(xsim SHARED ${LIB_SOURCES})

target_link_libraries(xsim  xviz cvx ${BULLET_LIBRARIES} ${OMPL_LIBRARIES})

# Install library
install(TARGETS xsim
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)


# Install headers
install(DIRECTORY "${LIBRARY_INCLUDE_DIR}" DESTINATION "${INSTALL_INCLUDE_DIR}" )
