# This "exports" all targets which have been put into the export set

install(EXPORT ${PROJECT_EXPORT} DESTINATION ${INSTALL_CMAKE_DIR} FILE ${PROJECT_NAME}Targets.cmake)

# Create the <package>Config.cmake.in
configure_file(${CMAKE_SOURCE_DIR}/CMake/Config.cmake.in
  "${PROJECT_CMAKE_FILES}/${LIB_PREFIX}Config.cmake" @ONLY)

# Create the <package>ConfigVersion.cmake.in
configure_file(${CMAKE_SOURCE_DIR}/CMake/ConfigVersion.cmake.in
  "${PROJECT_CMAKE_FILES}/${LIB_PREFIX}ConfigVersion.cmake" @ONLY)

# Install <package>Config.cmake and <package>ConfigVersion.cmake files
install(FILES
  "${PROJECT_CMAKE_FILES}/${LIB_PREFIX}Config.cmake"
  "${PROJECT_CMAKE_FILES}/${LIB_PREFIX}ConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Uninstall targets
configure_file("${CMAKE_SOURCE_DIR}/CMake/Uninstall.cmake.in"
  "${PROJECT_CMAKE_FILES}/Uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${PROJECT_CMAKE_FILES}/Uninstall.cmake)

