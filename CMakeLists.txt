cmake_minimum_required(VERSION 3.0.0)
project(space_filling_forest_star VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-O3)

add_subdirectory(lib/flann)
find_package(flann)
include_directories(lib/rapid-2.01)
include_directories(lib/ann/ann/include)
include_directories(lib/rapidxml)

aux_source_directory(src SRC)
include_directories(src)

add_executable(space_filling_forest_star ${SRC})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
