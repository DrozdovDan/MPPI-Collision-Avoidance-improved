cmake_minimum_required(VERSION 3.20)
project(mppi-ca-cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Routing can be built without fetching OR-Tools or the MPPI Python stack.
option(BUILD_MPPI_CONTROLLER "Build the original MPPI controller and bindings" ON)
include(CTest)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
add_library(polygon_geometry STATIC src/polygon_geometry.cpp)
set_target_properties(polygon_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(polygon_geometry PUBLIC ${PROJECT_SOURCE_DIR}/include)
add_library(corridor_planner STATIC src/corridor_planner.cpp src/corridor_map_io.cpp src/corridor_lanes.cpp)
target_include_directories(corridor_planner PUBLIC ${PROJECT_SOURCE_DIR}/include PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(corridor_planner PUBLIC polygon_geometry Threads::Threads)
target_compile_definitions(corridor_planner PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
add_executable(corridor_plan tools/corridor_plan.cpp)
target_include_directories(corridor_plan PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_definitions(corridor_plan PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
target_link_libraries(corridor_plan PRIVATE corridor_planner)
if(BUILD_TESTING)
    add_executable(corridor_planner_test tests/corridor_planner_test.cpp)
    target_link_libraries(corridor_planner_test PRIVATE corridor_planner)
    add_test(NAME corridor_planner COMMAND corridor_planner_test)
    add_executable(corridor_lanes_test tests/corridor_lanes_test.cpp)
    target_link_libraries(corridor_lanes_test PRIVATE corridor_planner)
    add_test(NAME corridor_lanes COMMAND corridor_lanes_test)
    add_executable(corridor_radius_test tests/corridor_radius_test.cpp)
    target_link_libraries(corridor_radius_test PRIVATE corridor_planner)
    add_test(NAME corridor_radius COMMAND corridor_radius_test)
endif()
if(NOT BUILD_MPPI_CONTROLLER)
    return()
endif()
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

add_compile_options(-lcblas)
add_compile_options(-lopenblas)

find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

include(FetchContent)
FetchContent_Declare(
  or-tools
  GIT_REPOSITORY https://github.com/google/or-tools.git
  GIT_TAG        main
)


set(BUILD_SHARED_LIBS OFF) # disables dynamic(shared) library building
set(BUILD_DEPS ON)

# After the following call, the CMake targets defined by OR-Tools
# will be defined and available to the rest of the build
FetchContent_MakeAvailable(or-tools)

#find_package(ortools CONFIG REQUIRED)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})


set(XTENSOR_USE_XSIMD ON)

add_subdirectory(external/xsimd)
add_subdirectory(external/xtl)
add_subdirectory(external/xtensor)
add_subdirectory(external/xtensor-blas)
add_subdirectory(external/xtensor-python)

add_subdirectory(external/pybind11)


add_library(controller_library SHARED
        src/controller.cpp
        src/dynamics.cpp
        src/geometry.cpp
        src/orca_lines.cpp)


target_include_directories(controller_library
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        )

target_compile_definitions(controller_library PUBLIC -DUSE_CBLAS)

# -D_SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS -D_SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING -D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS -DUSE_BOP -DUSE_GLOP -DUSE_CBC -DUSE_CLP -DUSE_SCIP 

target_link_libraries(controller_library polygon_geometry ${LAPACK_LIBRARIES})

target_link_libraries(controller_library xtensor)
target_link_libraries(controller_library xtensor-blas)
target_link_libraries(controller_library xsimd)
target_link_libraries(controller_library ortools::ortools)


pybind11_add_module(mppi_ca_cpp python_bind/converters.hpp python_bind/converters.cpp python_bind/mppi_ca_cpp.cpp NO_EXTRAS)
target_link_libraries(mppi_ca_cpp PRIVATE xtensor-python Python::NumPy)
target_link_libraries(mppi_ca_cpp PRIVATE controller_library ortools::ortools)

set_target_properties(mppi_ca_cpp PROPERTIES PREFIX "")
set_target_properties(mppi_ca_cpp PROPERTIES OUTPUT_NAME "mppi_ca_cpp")
set_target_properties(mppi_ca_cpp PROPERTIES SUFFIX ".so")

add_custom_target(mppi_all)
add_dependencies(mppi_all
        controller_library
        mppi_ca_cpp)
