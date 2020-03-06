cmake_minimum_required(VERSION 2.8.3)

set(behavior_tree_sources
        serialize.cpp
        )

add_library(behavior_tree ${behavior_tree_sources})