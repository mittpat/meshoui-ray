#include "assets.h"

#include <experimental/filesystem>

#include <linalg.h>

#include <deque>
#include <functional>
#include <thread>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace std { namespace filesystem = experimental::filesystem; }
using namespace linalg;
using namespace linalg::aliases;

int main(int, char**)
{
    MoIntersectBVHAlgorithm intersectAlgorithm;
    intersectAlgorithm.intersectObj = [](const MoRay& ray, const MoTriangle& object) -> float
    {
        float3 intersectionPoint;
        if (moRayTriangleIntersect(ray, object, intersectionPoint))
        {
#define MO_RAY_ANY
#ifdef MO_RAY_ANY
            return 0.0;
#else
            return length(intersectionPoint - ray.origin);
#endif
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
#ifdef MO_RAY_ANY
            return 0.0;
#else
            return length(intersectionPoint - ray.origin);
#endif
        }
        return std::numeric_limits<float>::max();
    };
    uvIntersectAlgorithm.intersectBBox = [](const MoRay& ray, const MoBBox& bbox, float* t_near, float* t_far) -> bool
    {
        return bbox.intersect(ray, t_near, t_far);
    };


    MoNode root;
    MoMeshList meshes;

    MoLoadAsset("teapot.dae", &root, &meshes);

    int total = 0;
    std::function<void(MoNode, const float4x4 &)> draw = [&](MoNode node, const float4x4 & model)
    {
        if (node->mesh != nullptr)
        {
            //model

            struct Sample
            {
                uint8_t r, g, b, a;
            };

            int2 resolution(1024,1024);
            std::vector<Sample> outputUV(resolution[0] * resolution[1], {0,0,0,0});
            std::vector<Sample> output3D(resolution[0] * resolution[1], {0,0,0,0});

            double fov = 75.0 * 3.14159 / 360;
            double scale = std::tan(fov * 0.5);
            double imageAspectRatio = resolution[0] / double(resolution[1]);
            float4x4 cameraWorldTransform = identity;
            float3 eye(-10,0,0);// = cameraWorldTransform.w.xyz();

#define THREADED_RAY
#ifdef THREADED_RAY
            std::deque<std::thread> threads;
#endif
            for (std::uint32_t row = 0, height = resolution[1]; row < height; ++row)
            {
#ifdef THREADED_RAY
                if (threads.size() > std::thread::hardware_concurrency())
                {
                    threads.front().join();
                    threads.pop_front();
                }
                threads.emplace_back(std::thread([&, row]()
                {
#endif
                    for (std::uint32_t column = 0, width = resolution[0]; column < width; ++column)
                    {
                        float2 uv(column / float(resolution[0]), row / float(resolution[1]));
                        MoTriangle intersection;

                        std::uint32_t index = (resolution[1] - row - 1) * resolution[0] + column;
                        if (moIntersectBVH(node->mesh->bvhUV, MoRay(float3(uv, -1.0f), {0,0,1}), intersection, &uvIntersectAlgorithm))
                        {
                            float3 world = intersection.interpolate(uv);

                            auto dir = float3(-100,40,40)-world;
                            dir = normalize(dir);
                            if (moIntersectBVH(node->mesh->bvh, MoRay(world+dir*0.0001f,
                                                                      dir), intersection, &intersectAlgorithm))
                            {
                                outputUV[index] = { 0,0,0,255 };
                            }
                            else
                            {
                                outputUV[index] = { 255,255,255,255 };
                            }
                        }
                        else
                        {
                            outputUV[index] = { 255,255,255,255 };
                        }


                        index = row * resolution[0] + column;
                        double x = (2 * (column + 0.5) / double(resolution[0]) - 1) * imageAspectRatio * scale;
                        double y = (1 - 2 * (row + 0.5) / double(resolution[1])) * scale;
                        float3 sampleDirection = mul(cameraWorldTransform, float4(1, x, y, 0)).xyz();
                        sampleDirection = normalize(sampleDirection);
                        if (moIntersectBVH(node->mesh->bvh, MoRay(eye, sampleDirection), intersection, &intersectAlgorithm))
                        {
                            output3D[index] = { 0,0,0,255 };
                        }
                        else
                        {
                            output3D[index] = { 255,255,255,255 };
                        }
                    }
#ifdef THREADED_RAY
                }));
#endif
            }
#ifdef THREADED_RAY
            for (auto & thread : threads)
            {
                thread.join();
            }
#endif
            // self shadowing test
            stbi_write_png((std::string("test_uv_") + std::to_string(total) + ".png").c_str(), resolution[0], resolution[1], 4, outputUV.data(), 4 * resolution[0]);
            // simple raycast test
            stbi_write_png((std::string("test_3d_") + std::to_string(total++) + ".png").c_str(), resolution[0], resolution[1], 4, output3D.data(), 4 * resolution[0]);
        }
        for (std::uint32_t i = 0; i < node->childCount; ++i)
        {
            MoNode child = node->pChildren[i];
            draw(child, mul(model, child->model));
        }
    };
    draw(root, root->model);

    MoUnloadAsset(root, meshes);

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
