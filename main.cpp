#include "lightmap.h"

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

int main(int, char**)
{
    float4 lightSources[1] = {{float4(normalize(float3(1,1,1)),100)}};

    std::string filename = "teapot.dae";
    if (!filename.empty() && std::filesystem::exists(filename))
    {
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
            info.enableAmbiantLightingSurfaceDiffusion = 1;
            info.ambiantLightingSampleCount = 512;
            info.ambiantLightingContribution = 1.f;
            info.ambiantOcclusionDistance = 1.f;
            info.pDirectionalLightSources = lightSources;
            info.directionalLightSourceCount = 1;
            std::vector<MoTextureSample> output(info.width * info.height, {0,0,0,0});
            moGenerateLightMap(triangleList, output.data(), &info);
            moDestroyTriangleList(triangleList);

    #ifdef MO_SAVE_TO_FILE
            // self shadowing test
            stbi_write_png((std::string("test_uv_") + std::to_string(meshIdx) + ".png").c_str(), info.width, info.height, 4, output.data(), 4 * info.width);
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
