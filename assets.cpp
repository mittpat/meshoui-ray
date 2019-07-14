#include "assets.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <experimental/filesystem>

namespace std { namespace filesystem = experimental::filesystem; }
using namespace linalg;
using namespace linalg::aliases;

void parseNodes(const aiScene * scene, const std::vector<MoTriangleList> & meshes, aiNode * node, std::vector<MoNode> & nodes)
{
    nodes.push_back({node->mName.C_Str(), transpose(float4x4((float*)&node->mTransformation)), {}, {}});
    for (std::uint32_t i = 0; i < node->mNumMeshes; ++i)
    {
        nodes.back().children.push_back({scene->mMeshes[node->mMeshes[i]]->mName.C_Str(),
                                         identity,
                                         meshes[node->mMeshes[i]],
                                         {}});
    }

    for (std::uint32_t i = 0; i < node->mNumChildren; ++i)
    {
        parseNodes(scene, meshes, node->mChildren[i], nodes.back().children);
    }
}

void MoLoad(const std::string &filename, std::vector<MoNode> &nodes)
{
    if (!filename.empty() && std::filesystem::exists(filename))
    {
        Assimp::Importer importer;
        const aiScene * scene = importer.ReadFile(filename, aiProcess_Debone | aiProcessPreset_TargetRealtime_Fast);

        std::vector<MoTriangleList> meshes(scene->mNumMeshes);
        for (std::uint32_t meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
        {
            std::vector<MoTriangle> &triangles = meshes[meshIdx].triangles;

            const auto* mesh = scene->mMeshes[meshIdx];
            for (std::uint32_t faceIdx = 0; faceIdx < mesh->mNumFaces; ++faceIdx)
            {
                const auto* face = &mesh->mFaces[faceIdx];
                switch (face->mNumIndices)
                {
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                {
                    triangles.push_back({
                         float3((float*)&mesh->mVertices[face->mIndices[0]]),
                         float3((float*)&mesh->mVertices[face->mIndices[1]]),
                         float3((float*)&mesh->mVertices[face->mIndices[2]])
                                        });
                    break;
                }
                default:
                    break;
                }
            }

            meshes[meshIdx].bvh = MoBVH(triangles);
        }

        nodes.push_back({std::filesystem::canonical(filename).c_str(), identity, {}, {}});
        parseNodes(scene, meshes, scene->mRootNode, nodes.back().children);
    }
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
