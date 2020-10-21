cmake_minimum_required(VERSION 3.0.0)
project(space_filling_forest_star VERSION 0.1.0)

include(CTest)
enable_testing()

aux_source_directory(src SRC)
include_directories(src)

add_executable(space_filling_forest_star ${SRC})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
