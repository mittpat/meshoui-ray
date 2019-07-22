# meshouiray

![sample viewer_output](https://raw.githubusercontent.com/mittpat/meshoui-ray/master/screenshot.png)

Efficient baking of self-shadowing to texture on CPU.

Compiling
-------
* git submodule update --init --recursive
* mkdir BUILD & cd BUILD
* cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ..
* cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 14 2015 Win64" ..

Notes
-------

Compiled and tested on
* g++  (Ubuntu 7.3.0-27ubuntu1~18.04)
* MSVC (Windows 10 x64, MSVC++ 14.0)

License
-------

meshoui is licensed by Patrick Pelletier under the MIT License, see LICENSE for more information.
