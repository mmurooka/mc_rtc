add_library(mpack STATIC mpack.c mpack.h)
set_property(TARGET mpack PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(mpack PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
