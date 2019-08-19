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
    std::string outputFilenameFmt;

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

    MoLightmapCreateInfo info = {};
    info.directionalLightingSampleCount = 1;
    info.pDirectionalLightSources = directionalLightSources;
    info.directionalLightSourceCount = 1;
    info.pointLightingSampleCount = 32;
    info.pPointLightSources = pointLightSources;
    info.pointLightSourceCount = 0;

    try
    {
        cxxopts::Options options(argv[0], "Generate light maps from geometry and uv within Collada files.");
        options
          .positional_help("[optional args]")
          .show_positional_help();
        options.add_options()
          ("c,coherent", "Use coherent rays", cxxopts::value<bool>())
          ("D,amb_dist", "Ambient distance clamp", cxxopts::value<float>()->default_value("1.f"))
          ("P,amb_pow", "Ambient power [0-1]", cxxopts::value<float>()->default_value("1.f"))
          ("S,amb_samp", "Ambient sample count", cxxopts::value<std::uint32_t>()->default_value("64"))
          ("s,size", "Output size in pixels", cxxopts::value<std::vector<std::uint32_t>>()->default_value("256,256"))
          ("f,file", "Input dae file name", cxxopts::value<std::string>()->default_value("teapot.dae"))
          ("n,null", "Null color as 4-byte rgba", cxxopts::value<std::vector<std::uint8_t>>()->default_value("127,127,127,255"))
          ("o,output", "Output png file name", cxxopts::value<std::string>()->default_value("%s_%d_lightmap.png"))
          ("help", "Print help")
          ;
        cxxopts::ParseResult result = options.parse(argc, argv);
        if (result.count("help"))
        {
            std::cout << options.help({"", "Group"}) << std::endl;
            exit(0);
        }
        filename = result["file"].as<std::string>();
        outputFilenameFmt = result["output"].as<std::string>();

        info.despeckle = result["coherent"].as<bool>();
        info.size = uint2(result["size"].as<std::vector<std::uint32_t>>().data());
        info.nullColor = byte4(result["null"].as<std::vector<std::uint8_t>>().data());
        info.ambientLightingSampleCount = result["amb_samp"].as<std::uint32_t>();
        info.ambientLightingPower = result["amb_pow"].as<float>();
        info.ambientOcclusionDistance = result["amb_dist"].as<float>();
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    if (!filename.empty() && std::filesystem::exists(filename))
    {
        std::cout << "generating light map for " << filename << std::endl;

        std::vector<MoTextureSample> output(info.size.x * info.size.y);

        Assimp::Importer importer;
        const aiScene * scene = importer.ReadFile(filename, aiProcess_Debone | aiProcessPreset_TargetRealtime_Fast);

        for (std::uint32_t meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
        {
            output = {};

            MoTriangleList triangleList;
            moCreateTriangleList(scene->mMeshes[meshIdx], &triangleList);
            moGenerateLightMap(triangleList, output.data(), &info, &std::cout);
            moDestroyTriangleList(triangleList);

    #ifdef MO_SAVE_TO_FILE
            // self shadowing test
            char outputFilename[256];
            std::snprintf(outputFilename, 256, outputFilenameFmt.c_str(), std::filesystem::path(filename).stem().c_str(), meshIdx);
            stbi_write_png(outputFilename, info.size.x, info.size.y, 4, output.data(), 4 * info.size.x);
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
