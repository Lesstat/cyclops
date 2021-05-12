cmake_minimum_required(VERSION 3.5)
project(cycle-routing)
add_compile_options(-Wall -Wextra -Wpedantic --std=c++17 -Wno-register -fpermissive)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package (Threads)
find_package(glpk REQUIRED) 
find_package(CGAL 4.11 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(GMP REQUIRED)

find_package(Boost 1.58 REQUIRED COMPONENTS serialization program_options filesystem iostreams)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

message(STATUS "json lib '${JSONCPP_LIBRARIES}'")
link_libraries(${JSONCPP_LIBRARIES})

add_library(threadpool INTERFACE)
target_include_directories(threadpool INTERFACE vendor/ThreadPool)


# Actual Implementation of project in static library
file(GLOB lib_src src/cr_lib/*.cpp )
add_library(cr_lib STATIC ${lib_src})
target_include_directories(cr_lib PUBLIC src/cr_lib)
target_include_directories(cr_lib PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(cr_lib PRIVATE ${GLPK_LIBRARIES})
target_link_libraries(cr_lib PRIVATE ${Boost_LIBRARIES})
target_link_libraries(cr_lib PRIVATE ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(cr_lib PRIVATE CGAL::CGAL)
target_link_libraries(cr_lib PRIVATE ${GMP_LIBRARIES})
target_link_libraries(cr_lib PUBLIC threadpool)

add_subdirectory(vendor/Simple-Web-Server)

# Executable of project links against lib
add_executable(cyclops src/main.cpp)
target_link_libraries(cyclops cr_lib)
target_link_libraries(cyclops simple-web-server)
target_include_directories(cyclops PRIVATE ${JSONCPP_INCLUDE_DIRS})

# Definition of Testing library catch
add_library(catch INTERFACE)
target_include_directories(catch INTERFACE vendor/catch)


add_executable(cr_expr src/experiments.cpp)
target_link_libraries(cr_expr cr_lib)


# Testing executable links against catch and cr_lib
file(GLOB test_src test/*.cpp)
add_executable(cr_test ${test_src})
target_link_libraries(cr_test cr_lib catch)

# Enable testing through ctest
add_test(NAME cr_test COMMAND cr_test)
enable_testing()

