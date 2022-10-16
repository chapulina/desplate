cmake_minimum_required(VERSION 3.8)
project(desplate_empy)

find_package(ament_cmake REQUIRED)

##########################
# Generate template at compile time

list(APPEND FORMATS sdf urdf)
foreach(FORMAT sdf urdf)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${FORMAT})
  add_custom_command(
    OUTPUT generate_template_${FORMAT}
    COMMAND empy3
      ${CMAKE_CURRENT_SOURCE_DIR}/${FORMAT}/vehicle.${FORMAT}.em >
      ${CMAKE_CURRENT_BINARY_DIR}/${FORMAT}/vehicle.${FORMAT}
  )
  add_custom_target(generate_template_target_${FORMAT} ALL
    DEPENDS generate_template_${FORMAT}
  )
endforeach()

##########################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install split for demonstration purposes, this could be a single install call

# Demo launch files
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)
# Templates to be generated at launch time
install(
  DIRECTORY
    sdf
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)
# Descriptions generated at compile time
install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/sdf
    ${CMAKE_CURRENT_BINARY_DIR}/urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
