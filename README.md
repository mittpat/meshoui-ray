# meshouivk - Simple Collada parser and viewer

![sample viewer_output](https://raw.githubusercontent.com/mittpat/meshoui/master/screenshot.png)

Compiling
-------
* git submodule update --init --recursive
* mkdir BUILD & cd BUILD
* cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ..
* cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 14 2015 Win64" ..

Notes
-------

The Collada support is partial. Only phong material is supported.
Each header + implementation is independant from the others and can be used standalone.

Master targets Vulkan 1.1.70.
Compiled and tested on
* g++  (Ubuntu 7.3.0-27ubuntu1~18.04)
* MSVC (Windows 10 x64, MSVC++ 14.0)

License
-------

meshoui is licensed by Patrick Pelletier under the MIT License, see LICENSE for more information.
