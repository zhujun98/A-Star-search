cmake_minimum_required(VERSION 3.8)
project(Bachelor)

add_executable(test_map_search
        test_map_search.cpp
        ../map.cpp
        ../search.h
        )

target_compile_features(test_map_search
        PUBLIC cxx_auto_type
        PRIVATE cxx_auto_type)