cmake_minimum_required(VERSION 3.26)

project(PNAV_2D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# create executable
add_executable(main 
            main.cpp
            pnav.cpp)


