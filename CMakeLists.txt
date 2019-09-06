cmake_minimum_required(VERSION 3.10.0)
project(meshoui VERSION 0.2.0)

find_package(Threads)

add_subdirectory(3rdparty)

add_library(meshouiray lightmap.h lightmap.cpp)
target_include_directories(meshouiray PUBLIC .)
if(NOT MSVC)
   target_link_libraries(meshouiray PUBLIC assimp linalg Threads::Threads)
else()
   target_link_libraries(meshouiray PUBLIC assimp linalg)
endif()

add_executable(lightmapgen main.cpp)
if(NOT MSVC)
    target_link_libraries(lightmapgen PRIVATE meshouiray cxxopts stb stdc++fs)
else()
    target_link_libraries(lightmapgen PRIVATE meshouiray cxxopts stb)
endif()

add_custom_command(TARGET meshouiray POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                       ${CMAKE_CURRENT_SOURCE_DIR}/teapot.dae
                       $<TARGET_FILE_DIR:meshouiray>/teapot.dae
                   )
