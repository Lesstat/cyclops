cmake_minimum_required(VERSION 3.9)
project(cycle-routing)
add_compile_options(-Wall -Wextra -Wpedantic -g)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cotire)

find_package(glpk REQUIRED)

find_package(Boost 1.65 REQUIRED COMPONENTS serialization program_options filesystem)

# Actual Implementation of project in static library
file(GLOB lib_src src/cr_lib/*.cpp )
add_library(cr_lib STATIC ${lib_src})
target_include_directories(cr_lib PUBLIC src/cr_lib)
target_compile_features(cr_lib PUBLIC cxx_std_17)
target_link_libraries(cr_lib ${GLPK_LIBRARIES})
target_link_libraries(cr_lib ${Boost_LIBRARIES})

add_subdirectory(vendor/Simple-Web-Server)

# Executable of project links against lib
add_executable(cr src/main.cpp)
target_link_libraries(cr cr_lib)
target_link_libraries(cr simple-web-server)

# Definition of Testing library catch
add_library(catch INTERFACE)
target_include_directories(catch INTERFACE vendor/catch)

# Testing executable links against catch and cr_lib
file(GLOB test_src test/*.cpp)
add_executable(cr_test ${test_src})
target_link_libraries(cr_test cr_lib catch)

# Enable testing through ctest
add_test(NAME cr_test COMMAND cr_test)
enable_testing()

cotire(cr_lib cr)
