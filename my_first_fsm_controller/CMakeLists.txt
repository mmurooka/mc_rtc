cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME MyFirstFsmController)
set(PROJECT_DESCRIPTION "MyFirstFsmController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/MyFirstFsmController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/MyFirstFsmController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/MyFirstFsmController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

