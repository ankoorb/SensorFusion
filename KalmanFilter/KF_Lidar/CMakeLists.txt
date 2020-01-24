cmake_minimum_required(VERSION 3.10)
project(KF_Lidar)

set(CMAKE_CXX_STANDARD 11)

add_executable(KF_Lidar main.cpp KalmanFilter.cpp Tracking.cpp)