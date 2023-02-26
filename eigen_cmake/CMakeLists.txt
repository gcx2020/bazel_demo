cmake_minimum_required( VERSION 2.8 )
project( useEigen )
 
set( CMAKE_BUILD_TYPE "Debug" )
# set( CMAKE_CXX_FLAGS "-O3" )
 
# include_directories( "/usr/local/include/eigen3" )
include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/")
add_executable( eigenMatrix eigenMatrix.cpp )
