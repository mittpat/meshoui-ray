#include "lightmap.h"

#include <string>

#include <cxxopts.hpp>

#include <experimental/filesystem>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#define MO_SAVE_TO_FILE

namespace std { namespace filesystem = experimental::filesystem; }
using namespace linalg;
using namespace linalg::aliases;

int main(int argc, char** argv)
{
    std::string filename;
    try
    {
        cxxopts::Options options(argv[0], "Generate light maps from geometry and uv within Collada files.");
        options
          .positional_help("[optional args]")
          .show_positional_help();
        options.add_options()
          ("f,file", "File name (.dae)", cxxopts::value<std::string>()->default_value("teapot.dae"))
          ("help", "Print help")
          ;
        cxxopts::ParseResult result = options.parse(argc, argv);
        if (result.count("help"))
        {
            std::cout << options.help({"", "Group"}) << std::endl;
            exit(0);
        }
        filename = result["file"].as<std::string>();
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    MoDirectionalLight directionalLightSources[1] = {
        {normalize(float3(1,1,1)),
         0.6f,
         0.009f} // the sun's angular radius
    };

    MoPointLight pointLightSources[1] = {
        {float3(2,2,2),
         2.f,
         0.1f,
         0.f, 1.f, 0.f}
    };

    if (!filename.empty() && std::filesystem::exists(filename))
    {
        std::cout << "generating light map for " << filename << std::endl;

        Assimp::Importer importer;
        const aiScene * scene = importer.ReadFile(filename, aiProcess_Debone | aiProcessPreset_TargetRealtime_Fast);

        for (std::uint32_t meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
        {
            MoTriangleList triangleList;
            moCreateTriangleList(scene->mMeshes[meshIdx], &triangleList);

            MoLightmapCreateInfo info = {};
            info.nullColor = {127,127,127,255};
            info.width = 512;
            info.height = 512;
            info.ambiantLightingSampleCount = 256;
            info.ambiantLightingPower = 0.4f;
            info.ambiantOcclusionDistance = 1.f;
            info.directionalLightingSampleCount = 32;
            info.pDirectionalLightSources = directionalLightSources;
            info.directionalLightSourceCount = 1;
            info.pointLightingSampleCount = 32;
            info.pPointLightSources = pointLightSources;
            info.pointLightSourceCount = 0;
            std::vector<MoTextureSample> output(info.width * info.height, {0,0,0,0});
            moGenerateLightMap(triangleList, output.data(), &info, &std::cout);
            moDestroyTriangleList(triangleList);

    #ifdef MO_SAVE_TO_FILE
            // self shadowing test
            std::string outputFilename = std::string("test_uv_") + std::to_string(meshIdx) + ".png";
            stbi_write_png(outputFilename.c_str(), info.width, info.height, 4, output.data(), 4 * info.width);
            std::cout << "saved output as " << outputFilename << std::endl;
    #endif
        }
    }

    return 0;
}

/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2018 Patrick Pelletier
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/
