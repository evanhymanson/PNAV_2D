cmake_minimum_required(VERSION 3.26)

project(PNAV_2D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch Eigen
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(Eigen)

# create executable
add_executable(main
            main.cpp
            pnav.cpp
            logger.cpp
            filter.cpp
            sensor_noise.cpp)

# Link Eigen to executable
target_link_libraries(main PRIVATE Eigen3::Eigen)


