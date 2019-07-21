cmake_minimum_required(VERSION 3.10.0)
project(meshoui VERSION 0.2.0)

find_package(Threads)

add_subdirectory(3rdparty)

add_executable(meshouiray
    main.cpp
    carray.h
    assets.h assets.cpp
    intersect.h intersect.cpp)

if(NOT MSVC)
    target_link_libraries(meshouiray assimp linalg stb stdc++fs Threads::Threads)
else()
    target_link_libraries(meshouiray assimp linalg stb)
endif()

add_custom_command(TARGET meshouiray POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                       ${CMAKE_CURRENT_SOURCE_DIR}/teapot.dae
                       $<TARGET_FILE_DIR:meshouiray>/teapot.dae
                   )
