#include "assets.h"
#include "carray.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <experimental/filesystem>

#include <deque>
#include <thread>

namespace std { namespace filesystem = experimental::filesystem; }
using namespace linalg;
using namespace linalg::aliases;

void MoCreateTriangleList(const aiMesh * ai_mesh, MoTriangleList *pTriangleList)
{
    MoTriangleList triangleList = *pTriangleList = new MoTriangleList_T();
    *triangleList = {};

    carray_resize(&triangleList->pTriangles, &triangleList->triangleCount, ai_mesh->mNumFaces);
    for (std::uint32_t faceIdx = 0; faceIdx < ai_mesh->mNumFaces; ++faceIdx)
    {
        const auto* face = &ai_mesh->mFaces[faceIdx];
        switch (face->mNumIndices)
        {
        case 3:
        {
            MoTriangle& triangle = const_cast<MoTriangle&>(triangleList->pTriangles[faceIdx]);
            triangle.v0 = float3((float*)&ai_mesh->mVertices[face->mIndices[0]]);
            triangle.v1 = float3((float*)&ai_mesh->mVertices[face->mIndices[1]]);
            triangle.v2 = float3((float*)&ai_mesh->mVertices[face->mIndices[2]]);
            if (ai_mesh->HasTextureCoords(0))
            {
                triangle.uv0 = float2((float*)&ai_mesh->mTextureCoords[0][face->mIndices[0]]);
                triangle.uv1 = float2((float*)&ai_mesh->mTextureCoords[0][face->mIndices[1]]);
                triangle.uv2 = float2((float*)&ai_mesh->mTextureCoords[0][face->mIndices[2]]);
            }
            triangle.n0 = float3((float*)&ai_mesh->mNormals[face->mIndices[0]]);
            triangle.n1 = float3((float*)&ai_mesh->mNormals[face->mIndices[1]]);
            triangle.n2 = float3((float*)&ai_mesh->mNormals[face->mIndices[2]]);
            break;
        }
        default:
            printf("parseAiMesh(): Mesh %s, Face %d has %d vertices.\n", ai_mesh->mName.C_Str(), faceIdx, face->mNumIndices);
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

void MoDestroyTriangleList(MoTriangleList triangleList)
{
    moDestroyBVH(triangleList->bvh);
    moDestroyBVH(triangleList->bvhUV);
    carray_free(triangleList->pTriangles, &triangleList->triangleCount);
    delete triangleList;
}

bool MoLoadAsset(const std::string& filename, MoMeshList* pMeshList)
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
            MoTriangleList* triangleList = const_cast<MoTriangleList*>(&meshList->pTriangleLists[meshIdx]);
            MoCreateTriangleList(scene->mMeshes[meshIdx], triangleList);
        }
        return true;
    }
    return false;
}

void MoUnloadAsset(MoMeshList meshList)
{
    for (std::uint32_t i = 0; i < meshList->triangleListCount; ++i)
    {
        MoDestroyTriangleList(meshList->pTriangleLists[i]);
    }
    carray_free(meshList->pTriangleLists, &meshList->triangleListCount);
    delete meshList;
}

void MoGenerateLightMap(const MoTriangleList mesh, MoTextureSample* pTextureSamples, std::uint32_t width, std::uint32_t height)
{
#define MO_UV_MULTISAMPLE_OFFSET 1.0f
#define MO_SURFACE_BIAS 0.01f
    MoIntersectBVHAlgorithm intersectAlgorithm;
    intersectAlgorithm.intersectObj = [](const MoRay& ray, const MoTriangle& object) -> float
    {
        float3 intersectionPoint;
        if (moRayTriangleIntersect(ray, object, intersectionPoint))
        {
            return 0.0;
        }
        return std::numeric_limits<float>::max();
    };
    intersectAlgorithm.intersectBBox = [](const MoRay& ray, const MoBBox& bbox, float* t_near, float* t_far) -> bool
    {
        return bbox.intersect(ray, t_near, t_far);
    };

    MoIntersectBVHAlgorithm uvIntersectAlgorithm;
    uvIntersectAlgorithm.intersectObj = [](const MoRay& ray, const MoTriangle& object) -> float
    {
        if (moTexcoordInTriangleUV(ray.origin.xy(), object))
        {
            return 0.0;
        }
        return std::numeric_limits<float>::max();
    };
    uvIntersectAlgorithm.intersectBBox = [](const MoRay& ray, const MoBBox& bbox, float* t_near, float* t_far) -> bool
    {
        return bbox.intersect(ray, t_near, t_far);
    };

    std::deque<std::thread> threads;
    for (int row = 0; row < height; ++row)
    {
        if (threads.size() > std::thread::hardware_concurrency())
        {
            threads.front().join();
            threads.pop_front();
        }
        threads.emplace_back(std::thread([&, row]()
        {
            for (int column = 0; column < width; ++column)
            {
                float2 uv((column + 0.5) / float(width), (row + 0.5) / float(height));
                MoTriangle intersection;

                std::uint32_t index = (height - row - 1) * width + column;

                float3 multisamples[] = {float3(uv, -1.0f),
                                         float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(width),  MO_UV_MULTISAMPLE_OFFSET / float(height)), -1.0f),
                                         float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(width), -MO_UV_MULTISAMPLE_OFFSET / float(height)), -1.0f),
                                         float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(width), -MO_UV_MULTISAMPLE_OFFSET / float(height)), -1.0f),
                                         float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(width),  MO_UV_MULTISAMPLE_OFFSET / float(height)), -1.0f)
                                        };
                for (float3 singleSample : multisamples)
                {
                    if (moIntersectBVH(mesh->bvhUV, MoRay(singleSample, {0,0,1}), intersection, &uvIntersectAlgorithm))
                    {
                        float3 barycentricCoordinates = intersection.uvBarycentric(uv);
                        float3 world = intersection.v0 * barycentricCoordinates[0]
                                + intersection.v1 * barycentricCoordinates[1]
                                + intersection.v2 * barycentricCoordinates[2];

                        float3 normal = intersection.n0 * barycentricCoordinates[0]
                                + intersection.n1 * barycentricCoordinates[1]
                                + intersection.n2 * barycentricCoordinates[2];

                        float3 lightDirection = normalize(float3(-100,40,40) - world);

                        float diffuseFactor = dot(normal, lightDirection);
                        if (diffuseFactor > 0.0)
                        {
                            if (moIntersectBVH(mesh->bvh, MoRay(world + normal * MO_SURFACE_BIAS, lightDirection),
                                               intersection, &intersectAlgorithm))
                            {
                                pTextureSamples[index] = { 0,0,0,255 };
                            }
                            else
                            {
                                pTextureSamples[index] = {
                                    std::uint8_t(std::min(255.f, diffuseFactor * 255)),
                                    std::uint8_t(std::min(255.f, diffuseFactor * 255)),
                                    std::uint8_t(std::min(255.f, diffuseFactor * 255)),
                                    255 };
                            }
                        }
                        else
                        {
                            pTextureSamples[index] = { 0,0,0,255 };
                        }
                        break;
                    }
                }
                if (pTextureSamples[index].a == 0)
                {
                    pTextureSamples[index] = { 127,127,127,255 };
                }
            }
        }));
    }
    for (auto & thread : threads)
    {
        thread.join();
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
