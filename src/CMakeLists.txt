
include_directories(
    3rdparty/
    ${BULLET_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
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

       ${LIBRARY_INCLUDE_DIR}/world.hpp
       ${LIBRARY_INCLUDE_DIR}/rigid_body.hpp
       ${LIBRARY_INCLUDE_DIR}/collision.hpp
       ${LIBRARY_INCLUDE_DIR}/convert.hpp
       ${LIBRARY_INCLUDE_DIR}/constraints.hpp
       ${LIBRARY_INCLUDE_DIR}/multi_body.hpp
       ${LIBRARY_INCLUDE_DIR}/sensor.hpp
       ${LIBRARY_INCLUDE_DIR}/soft_body.hpp
       ${LIBRARY_INCLUDE_DIR}/multi_body_builder.hpp
)

add_library(xsim SHARED ${LIB_SOURCES})

target_link_libraries(xsim  ${BULLET_LIBRARIES} xviz)

# Install library
install(TARGETS xsim
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)


# Install headers
install(DIRECTORY "${LIBRARY_INCLUDE_DIR}" DESTINATION "${INSTALL_INCLUDE_DIR}" )
