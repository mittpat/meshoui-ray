# meshouiray

![sample viewer_output](https://raw.githubusercontent.com/mittpat/meshoui-ray/master/screenshot.png)

Efficient baking of self-shadowing to texture on CPU.
Works by raycasting from uv coordinates on the surface of the model to a light source.

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

Perf
-------

* For 27384 vertices
* To bake a 1024x1024 lightmap to memory (save to file #if'ed out)

```
build-meshoui-ray-Imported_Kit-Release$ sudo perf stat ./meshouiray 

 Performance counter stats for './meshouiray':

       1316.651131      task-clock (msec)         #    5.716 CPUs utilized          
               963      context-switches          #    0.731 K/sec                  
               364      cpu-migrations            #    0.276 K/sec                  
             4,980      page-faults               #    0.004 M/sec                  
     4,806,578,691      cycles                    #    3.651 GHz                    
     2,809,423,525      stalled-cycles-frontend   #   58.45% frontend cycles idle   
     6,673,128,040      instructions              #    1.39  insn per cycle         
                                                  #    0.42  stalled cycles per insn
       783,555,506      branches                  #  595.112 M/sec                  
        14,920,557      branch-misses             #    1.90% of all branches        

       0.230363837 seconds time elapsed
```

License
-------

meshoui is licensed by Patrick Pelletier under the MIT License, see LICENSE for more information.
