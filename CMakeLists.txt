cmake_minimum_required(VERSION 3.10.0)
project(meshoui VERSION 0.2.0)

add_subdirectory(3rdparty)

add_executable(meshouiray
    main.cpp)

if(NOT MSVC)
    target_link_libraries(meshouiray linalg stb stdc++fs)
else()
    target_link_libraries(meshouiray linalg stb)
endif()
