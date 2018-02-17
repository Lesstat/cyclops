cmake_minimum_required(VERSION 3.5)
project(cycle-routing)
add_compile_options(-Wall -Wextra -Wpedantic -Wregister -std=c++17 -g)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cotire)

#find_package(glpk REQUIRED)

find_package(Boost 1.58 REQUIRED COMPONENTS serialization program_options filesystem)
find_package (Threads)



# Actual Implementation of project in static library
file(GLOB lib_src src/cr_lib/*.cpp )
add_library(cr_lib STATIC ${lib_src})
target_include_directories(cr_lib PUBLIC src/cr_lib /usr/include/coin )
#target_link_libraries(cr_lib ${GLPK_LIBRARIES})
target_link_libraries(cr_lib -lClp)
target_link_libraries(cr_lib -lCoinUtils)
target_link_libraries(cr_lib ${Boost_LIBRARIES})
target_link_libraries(cr_lib -lm)
target_link_libraries(cr_lib ${CMAKE_THREAD_LIBS_INIT})

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
