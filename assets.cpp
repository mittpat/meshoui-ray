#include "assets.h"
#include "carray.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <experimental/filesystem>

namespace std { namespace filesystem = experimental::filesystem; }
using namespace linalg;
using namespace linalg::aliases;

void parseNodes(const aiScene * ai_scene, aiNode * ai_node, MoNode node, MoMeshList meshes)
{
    carray_resize(&node->pChildren, &node->childCount, ai_node->mNumMeshes + ai_node->mNumChildren);
    for (std::uint32_t i = 0; i < ai_node->mNumMeshes; ++i)
    {
        MoNode child = const_cast<MoNode&>(node->pChildren[i]) = new MoNode_T();
        *child = {};
        child->name = ai_scene->mMeshes[ai_node->mMeshes[i]]->mName.C_Str();
        child->model = identity;
        child->mesh = meshes->pTriangleLists[ai_node->mMeshes[i]];
    }
    for (std::uint32_t i = 0; i < ai_node->mNumChildren; ++i)
    {
        MoNode child = const_cast<MoNode&>(node->pChildren[ai_node->mNumMeshes + i]) = new MoNode_T();
        *child = {};
        child->name = ai_node->mChildren[i]->mName.C_Str();
        child->model = transpose(float4x4((float*)&ai_node->mChildren[i]->mTransformation));
        parseNodes(ai_scene, ai_node->mChildren[i], child, meshes);
    }
}

bool MoLoadAsset(const std::string& filename, MoNode* pRootNode, MoMeshList* pMeshList)
{
    if (!filename.empty() && std::filesystem::exists(filename))
    {
        Assimp::Importer importer;
        const aiScene * scene = importer.ReadFile(filename, aiProcess_Debone | aiProcessPreset_TargetRealtime_Fast);

        MoMeshList meshList = *pMeshList = new MoMeshList_T();
        *meshList = {};
        carray_resize(&meshList->pTriangleLists, &meshList->triangleListCount, scene->mNumMeshes);
        for (std::uint32_t meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx)
        {
            MoTriangleList triangleList = const_cast<MoTriangleList&>(meshList->pTriangleLists[meshIdx]) = new MoTriangleList_T();
            *triangleList = {};

            const auto* mesh = scene->mMeshes[meshIdx];
            carray_resize(&triangleList->pTriangles, &triangleList->triangleCount, mesh->mNumFaces);
            for (std::uint32_t faceIdx = 0; faceIdx < mesh->mNumFaces; ++faceIdx)
            {
                const auto* face = &mesh->mFaces[faceIdx];
                switch (face->mNumIndices)
                {
                case 3:
                {
                    MoTriangle& triangle = const_cast<MoTriangle&>(triangleList->pTriangles[faceIdx]);
                    triangle.v0 = float3((float*)&mesh->mVertices[face->mIndices[0]]);
                    triangle.v1 = float3((float*)&mesh->mVertices[face->mIndices[1]]);
                    triangle.v2 = float3((float*)&mesh->mVertices[face->mIndices[2]]);
                    if (mesh->HasTextureCoords(0))
                    {
                        triangle.uv0 = float2((float*)&mesh->mTextureCoords[0][face->mIndices[0]]);
                        triangle.uv1 = float2((float*)&mesh->mTextureCoords[0][face->mIndices[1]]);
                        triangle.uv2 = float2((float*)&mesh->mTextureCoords[0][face->mIndices[2]]);
                    }
                    triangle.n0 = float3((float*)&mesh->mNormals[face->mIndices[0]]);
                    triangle.n1 = float3((float*)&mesh->mNormals[face->mIndices[1]]);
                    triangle.n2 = float3((float*)&mesh->mNormals[face->mIndices[2]]);
                    break;
                }
                default:
                    printf("MoLoadAsset(): File %s, Mesh %s, Face %d has %d vertices.\n", filename.c_str(), mesh->mName.C_Str(), faceIdx, face->mNumIndices);
                    break;
                }
            }

            MoCreateBVHAlgorithm algo;
            // 3d space
            algo.getBoundingBox = [](const MoTriangle& object) -> MoBBox { return object.getBoundingBox(); };
            algo.getCentroid = [](const MoTriangle& object) -> float3 { return object.getCentroid(); };
            moCreateBVH(triangleList->pTriangles, triangleList->triangleCount, &triangleList->bvh, &algo);

            // uv space
            algo.getBoundingBox = [](const MoTriangle& object) -> MoBBox { return object.getUVBoundingBox(); };
            algo.getCentroid = [](const MoTriangle& object) -> float3 { return object.getUVCentroid(); };
            moCreateBVH(triangleList->pTriangles, triangleList->triangleCount, &triangleList->bvhUV, &algo);
        }

        MoNode rootNode = *pRootNode = new MoNode_T();
        *rootNode = {};
        rootNode->name = filename + ":" + scene->mRootNode->mName.C_Str();
        rootNode->model = transpose(float4x4((float*)&scene->mRootNode->mTransformation));
        parseNodes(scene, scene->mRootNode, rootNode, meshList);

        return true;
    }

    return false;
}

void MoDestroyNode(MoNode node)
{
    for (std::uint32_t i = 0; i < node->childCount; ++i)
    {
        MoDestroyNode(node->pChildren[i]);
    }
    carray_free(node->pChildren, &node->childCount);
    delete node;
}

void MoUnloadAsset(MoNode rootNode, MoMeshList meshList)
{
    for (std::uint32_t i = 0; i < meshList->triangleListCount; ++i)
    {
        MoTriangleList triangleList = meshList->pTriangleLists[i];
        moDestroyBVH(triangleList->bvh);
        moDestroyBVH(triangleList->bvhUV);
        carray_free(triangleList->pTriangles, &triangleList->triangleCount);
        delete triangleList;
    }
    carray_free(meshList->pTriangleLists, &meshList->triangleListCount);
    delete meshList;

    MoDestroyNode(rootNode);
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
