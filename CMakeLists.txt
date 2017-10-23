
cmake_minimum_required(VERSION 3.9)
project(cycle-routing)

add_compile_options(-Wall -Wextra -Wpedantic)
file(GLOB cpp_files src/*.cpp)

add_executable(cr ${cpp_files})
target_compile_features(cr PUBLIC cxx_std_17)
