cmake_minimum_required(VERSION 3.9)
project(AStarOnGrid)

set(CMAKE_CXX_STANDARD 11)

add_executable(AStarOnGrid main.cpp ExplorationAgenda.cpp ExplorationAgenda.h)