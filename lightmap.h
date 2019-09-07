#pragma once

#include <linalg.h>
#include <iosfwd>

class MoRay
{
public:
    MoRay() {}
    MoRay(const linalg::aliases::float3& o, const linalg::aliases::float3& d)
        : origin(o)
        , direction(d)
        , oneOverDirection(1 / direction.x, 1 / direction.y, 1 / direction.z) {}

    alignas(16) linalg::aliases::float3 origin;
    alignas(16) linalg::aliases::float3 direction;
    alignas(16) linalg::aliases::float3 oneOverDirection;
};

class MoBBox
{
public:
    MoBBox() {}
    MoBBox(const linalg::aliases::float3& min, const linalg::aliases::float3& max);
    MoBBox(const linalg::aliases::float3& point);

    std::uint32_t longestSide() const;
    bool intersect(const MoRay& ray, float& t_near, float& t_far) const;
    void expandToInclude(const linalg::aliases::float3& point);
    void expandToInclude(const MoBBox& box);

    alignas(16) linalg::aliases::float3 min;
    alignas(16) linalg::aliases::float3 max;
};

struct MoTriangle
{
    MoBBox getBoundingBox() const
    {
        MoBBox bb(v0);
        bb.expandToInclude(v1);
        bb.expandToInclude(v2);
        return bb;
    }
    linalg::aliases::float3 getCentroid() const
    {
        return (v0 + v1 + v2) / 3.0f;
    }
    MoBBox getUVBoundingBox() const
    {
        MoBBox bb(linalg::aliases::float3(uv0, 0.f));
        bb.expandToInclude(linalg::aliases::float3(uv1, 0.f));
        bb.expandToInclude(linalg::aliases::float3(uv2, 0.f));
        return bb;
    }
    linalg::aliases::float3 getUVCentroid() const
    {
        return linalg::aliases::float3((uv0 + uv1 + uv2) / 3.0f, 0.f);
    }
    linalg::aliases::float3 getUVBarycentric(const linalg::aliases::float2& uv) const;
    void getSurface(const linalg::aliases::float3& barycentricCoordinates, linalg::aliases::float3& point, linalg::aliases::float3& normal) const;

    alignas(16) linalg::aliases::float3 v0;
    alignas(16) linalg::aliases::float3 v1;
    alignas(16) linalg::aliases::float3 v2;
    alignas( 8) linalg::aliases::float2 uv0;
    alignas( 8) linalg::aliases::float2 uv1;
    alignas( 8) linalg::aliases::float2 uv2;
    alignas(16) linalg::aliases::float3 n0;
    alignas(16) linalg::aliases::float3 n1;
    alignas(16) linalg::aliases::float3 n2;
};

bool moRayTriangleIntersect(const MoRay& ray, const MoTriangle& triangle, float &t, float &u, float &v);

bool moTexcoordInTriangleUV(linalg::aliases::float2 tex, const MoTriangle& triangle);

struct MoBVHSplitNode
{
    MoBBox        boundingBox;
    std::uint32_t start;
    std::uint32_t offset;
};

typedef struct MoTriangleList_T* MoTriangleList;
typedef struct MoBVH_T
{
    std::uint32_t         splitNodeCount;
    const MoBVHSplitNode* pSplitNodes;
    const std::uint32_t*  pIndices;
    MoTriangleList        triangleList;
}* MoBVH;

struct MoCreateBVHAlgorithm
{
    MoBBox                  (*getBoundingBox)(const MoTriangle&);
    linalg::aliases::float3 (*getCentroid)(const MoTriangle&);
};

void moCreateBVH(MoTriangleList triangleList, MoBVH* pBVH, MoCreateBVHAlgorithm* pAlgorithm);

void moDestroyBVH(MoBVH bvh);

struct MoIntersectBVHAlgorithm
{
    // 0.0 intersects and returns immediately
    // > 0 intersects and continues to attempt to find nearer intersection
    // std::numeric_limits<float>::max() doesn't intersect
    float (*intersectObj)(const MoRay& ray, const MoTriangle&, float&, float&);
    // bbox, near, far -> bool
    bool (*intersectBBox)(const MoRay& ray, const MoBBox&, float&, float&);
};

void moSetIntersectBVHAlgorithm_TriangleMesh(MoIntersectBVHAlgorithm* pAlgorithm);
void moSetIntersectBVHAlgorithm_Texcoords(MoIntersectBVHAlgorithm* pAlgorithm);

struct MoIntersectResult
{
    const MoTriangle* pTriangle;
    linalg::aliases::float3 barycentric;
    float distance;
};

bool moIntersectBVH(MoBVH bvh, const MoRay& ray, MoIntersectResult& intersection, MoIntersectBVHAlgorithm* pAlgorithm);

struct aiMesh;

typedef struct MoTriangleList_T
{
    const MoTriangle* pTriangles;
    std::uint32_t     triangleCount;
    MoBVH             bvh;
    MoBVH             bvhUV;
}* MoTriangleList;

void moCreateTriangleList(const aiMesh* ai_mesh, MoTriangleList *pTriangleList);
void moDestroyTriangleList(MoTriangleList triangleList);

struct MoDirectionalLight
{
    linalg::aliases::float3 direction;
    float                   power;
    float                   angularSize;
};

struct MoPointLight
{
    linalg::aliases::float3 position;
    float                   power;
    float                   size;
    float                   constantAttenuation;
    float                   linearAttenuation;
    float                   quadraticAttenuation;
};

struct MoLightmapCreateInfo {
    linalg::aliases::byte4   nullColor;
    linalg::aliases::uint2   size;
    std::uint32_t            flipY;
    std::uint32_t            despeckle;
    std::uint32_t            jobs;
    // ambient lighting
    std::uint32_t            ambientLightingSampleCount;
    float                    ambientLightingPower;
    float                    ambientOcclusionDistance;
    // directional lighting
    std::uint32_t            directionalLightingSampleCount;
    MoDirectionalLight*      pDirectionalLightSources;
    std::uint32_t            directionalLightSourceCount;
    // point lighting
    std::uint32_t            pointLightingSampleCount;
    MoPointLight*            pPointLightSources;
    std::uint32_t            pointLightSourceCount;
};

typedef linalg::aliases::byte4 MoTextureSample; //rgba
void moGenerateLightMap(const MoTriangleList mesh, MoTextureSample* pTextureSamples, const MoLightmapCreateInfo* pCreateInfo, std::ostream* pLog = nullptr);
void moGenerateNormalMap(const MoTriangleList mesh, MoTextureSample* pTextureSamples, const MoLightmapCreateInfo* pCreateInfo, std::ostream* pLog = nullptr);

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
