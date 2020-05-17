set(controller_SRC
  MyFirstController.cpp
)

set(controller_HDR
  MyFirstController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMyFirstController_EXPORTS")

find_package(RBDyn REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(print_robot print_robot.cpp)
target_include_directories(print_robot PRIVATE ${EIGEN3_INCLUDE_DIR} ${RBDyn_INCLUDE_DIRS})
target_link_libraries(print_robot ${RBDyn_LIBRARIES} mc_rtc::mc_control)
