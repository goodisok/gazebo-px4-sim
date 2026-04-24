cmake_minimum_required(VERSION 3.16)
project(QuadraticDrag)

find_package(gz-sim8 REQUIRED)

add_library(QuadraticDrag SHARED QuadraticDrag.cc)
target_link_libraries(QuadraticDrag PUBLIC gz-sim8::gz-sim8)
