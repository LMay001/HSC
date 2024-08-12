cmake_minimum_required(VERSION 3.5)
project(HSC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization filters kdtree)
find_package(Eigen3)


include_directories(
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include # path of header files
    /usr/local/include  # path of yaml-cpp
)


add_executable(hsc
    src/main.cpp
    src/HSC.cpp
)

target_link_libraries(hsc
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    /usr/local/lib/libyaml-cpp.so
)
