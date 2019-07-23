#include "intersect.h"
#include "carray.h"

#include <algorithm>
#include <limits>

namespace
{
    enum : std::uint32_t
    {
        Node_Untouched    = 0xffffffff,
        Node_TouchedTwice = 0xfffffffd,
        Node_Root         = 0xfffffffc,
    };
}

using namespace linalg;
using namespace linalg::aliases;

MoBBox::MoBBox(const float3& _min, const float3& _max)
    : min(_min)
    , max(_max)
{
    extent = max - min;
}

MoBBox::MoBBox(const float3& point)
    : min(point)
    , max(point)
{
    extent = max - min;
}

std::uint32_t MoBBox::longestSide() const
{
    std::uint32_t dimension = 0;
    if (extent.y > extent.x)
    {
        dimension = 1;
        if (extent.z > extent.y)
        {
            dimension = 2;
        }
    }
    else if (extent.z > extent.x)
    {
        dimension = 2;
    }
    return dimension;
}

void MoBBox::expandToInclude(const float3& point)
{
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
    extent = max - min;
}

void MoBBox::expandToInclude(const MoBBox& box)
{
    min.x = std::min(min.x, box.min.x);
    min.y = std::min(min.y, box.min.y);
    min.z = std::min(min.z, box.min.z);
    max.x = std::max(max.x, box.max.x);
    max.y = std::max(max.y, box.max.y);
    max.z = std::max(max.z, box.max.z);
    extent = max - min;
}

// adapted from Tavian Barnes' "Fast, Branchless Ray/Bounding Box Intersections"
bool MoBBox::intersect(const MoRay& ray, float* t_near, float* t_far) const
{
    float tx1 = (min.x - ray.origin.x) * ray.oneOverDirection.x;
    float tx2 = (max.x - ray.origin.x) * ray.oneOverDirection.x;

    float tmin = std::min(tx1, tx2);
    float tmax = std::max(tx1, tx2);

    float ty1 = (min.y - ray.origin.y) * ray.oneOverDirection.y;
    float ty2 = (max.y - ray.origin.y) * ray.oneOverDirection.y;

    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    float tz1 = (min.z - ray.origin.z) * ray.oneOverDirection.z;
    float tz2 = (max.z - ray.origin.z) * ray.oneOverDirection.z;

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    if (t_near)
        *t_near = tmin;

    if (t_far)
        *t_far = tmax;

    return tmax >= tmin;
}

float3 MoTriangle::uvBarycentric(const float2 &uv) const
{
    float x[4] = {uv.x, uv0.x, uv1.x, uv2.x};
    float y[4] = {uv.y, uv0.y, uv1.y, uv2.y};

    float d = (y[2] - y[3]) * (x[1] - x[3]) + (x[3] - x[2]) * (y[1] - y[3]);
    float l1 = ((y[2] - y[3]) * (x[0] - x[3]) + (x[3] - x[2]) * (y[0] - y[3]))
            / d;
    float l2 = ((y[3] - y[1]) * (x[0] - x[3]) + (x[1] - x[3]) * (y[0] - y[3]))
            / d;
    float l3 = 1 - l1 - l2;

#if 0
    float2 test = l1 * uv0 + l2 * uv1 + l3 * uv2;
#endif

    return float3(l1, l2, l3);
}

// Adapted from the Möller–Trumbore intersection algorithm
bool moRayTriangleIntersect(const MoRay &ray, const MoTriangle &triangle, float3 &intersectionPoint)
{
    const float EPSILON = 0.0000001f;
    const float3 & vertex0 = triangle.v0;
    const float3 & vertex1 = triangle.v1;
    const float3 & vertex2 = triangle.v2;
    const float3 edge1 = vertex1 - vertex0;
    const float3 edge2 = vertex2 - vertex0;
    const float3 h = linalg::cross(ray.direction, edge2);
    const float a = linalg::dot(edge1, h);
    if (a > -EPSILON && a < EPSILON)
    {
        // This ray is parallel to this triangle.
        return false;
    }

    const float f = 1.0/a;
    const float3 s = ray.origin - vertex0;
    const float u = f * linalg::dot(s, h);
    if (u < 0.0 || u > 1.0)
    {
        return false;
    }

    const float3 q = linalg::cross(s, edge1);
    const float v = f * linalg::dot(ray.direction, q);
    if (v < 0.0 || u + v > 1.0)
    {
        return false;
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    const float t = f * linalg::dot(edge2, q);
    if (t > EPSILON)
    {
        // ray intersection
        intersectionPoint = ray.origin + ray.direction * t;
        return true;
    }

    // This means that there is a line intersection but not a ray intersection.
    return false;
}

bool moTexcoordInTriangleUV(float2 tex, const MoTriangle& triangle)
{
    float s = triangle.uv0.y * triangle.uv2.x - triangle.uv0.x * triangle.uv2.y + (triangle.uv2.y - triangle.uv0.y) * tex.x + (triangle.uv0.x - triangle.uv2.x) * tex.y;
    float t = triangle.uv0.x * triangle.uv1.y - triangle.uv0.y * triangle.uv1.x + (triangle.uv0.y - triangle.uv1.y) * tex.x + (triangle.uv1.x - triangle.uv0.x) * tex.y;

    if ((s < 0) != (t < 0))
        return false;

    float area = -triangle.uv1.y * triangle.uv2.x + triangle.uv0.y * (triangle.uv2.x - triangle.uv1.x) + triangle.uv0.x * (triangle.uv1.y - triangle.uv2.y) + triangle.uv1.x * triangle.uv2.y;

    return area < 0 ?
            (s <= 0 && s + t >= area) :
            (s >= 0 && s + t <= area);
}

void moCreateBVH(const MoTriangle *pObjects, uint32_t objectCount, MoBVH* pBVH, MoCreateBVHAlgorithm *pAlgorithm)
{
    MoBVH bvh = *pBVH = new MoBVH_T();
    *bvh = {};
    carray_resize(&bvh->pObjects, &bvh->objectCount, objectCount);
    carray_copy(pObjects, bvh->pObjects, bvh->objectCount);
    bvh->splitNodeCount = 0;
    std::uint32_t stackPtr = 0;

    struct Entry
    {
        std::uint32_t parent;
        std::uint32_t start;
        std::uint32_t end;
    };
    Entry* entries = {};
    std::uint32_t entryCount = 0;
    carray_resize(&entries, &entryCount, 1);
    entries[stackPtr].start = 0;
    entries[stackPtr].end = bvh->objectCount;
    entries[stackPtr].parent = Node_Root;
    stackPtr++;

    std::uint32_t splitNodeCount = 0;

    MoBVHSplitNode splitNode;
    while (stackPtr > 0)
    {
        Entry &entry = entries[--stackPtr];
        std::uint32_t start = entry.start;
        std::uint32_t end = entry.end;
        std::uint32_t count = end - start;

        splitNodeCount++;
        splitNode.start = start;
        splitNode.count = count;
        splitNode.offset = Node_Untouched;

        MoBBox boundingBox = pAlgorithm->getBoundingBox(bvh->pObjects[start]);
        MoBBox boundingBoxCentroids = pAlgorithm->getCentroid(bvh->pObjects[start]);
        for (std::uint32_t i = start + 1; i < end; ++i)
        {
            boundingBox.expandToInclude(pAlgorithm->getBoundingBox(bvh->pObjects[i]));
            boundingBoxCentroids.expandToInclude(pAlgorithm->getCentroid(bvh->pObjects[i]));
        }
        splitNode.boundingBox = boundingBox;

        // we're at the leaf
        if (count <= 1)
        {
            splitNode.offset = 0;
        }

        carray_push_back(&bvh->pSplitNodes, &bvh->splitNodeCount, splitNode);
        if (entry.parent != Node_Root)
        {
            MoBVHSplitNode & splitNode = const_cast<MoBVHSplitNode*>(bvh->pSplitNodes)[entry.parent];
            splitNode.offset--;
            if (splitNode.offset == Node_TouchedTwice)
            {
                splitNode.offset = splitNodeCount - 1 - entry.parent;
            }
        }

        if (splitNode.offset == 0)
        {
            continue;
        }

        // find longest bbox side
        std::uint32_t splitDimension = boundingBoxCentroids.longestSide();
        float splitLength = .5f * (boundingBoxCentroids.min[splitDimension] + boundingBoxCentroids.max[splitDimension]);

        std::uint32_t mid = start;
        for (std::uint32_t i = start; i < end; ++i)
        {
            if (pAlgorithm->getCentroid(bvh->pObjects[i])[splitDimension] < splitLength)
            {
                MoTriangle* lObjects = const_cast<MoTriangle*>(bvh->pObjects);
                std::swap(lObjects[i], lObjects[mid]);
                ++mid;
            }
        }

        if (mid == start || mid == end)
        {
            mid = start + (end-start) / 2;
        }

        // left
        if (entryCount <= stackPtr)
            carray_push_back(&entries, &entryCount, {});
        entries[stackPtr].start = mid;
        entries[stackPtr].end = end;
        entries[stackPtr].parent = splitNodeCount - 1;
        stackPtr++;

        // right
        if (entryCount <= stackPtr)
            carray_push_back(&entries, &entryCount, {});
        entries[stackPtr].start = start;
        entries[stackPtr].end = mid;
        entries[stackPtr].parent = splitNodeCount - 1;
        stackPtr++;
    }

    carray_resize(&bvh->pSplitNodes, &bvh->splitNodeCount, splitNodeCount);
    carray_free(entries, &entryCount);
}

void moDestroyBVH(MoBVH bvh)
{
    carray_free(bvh->pObjects, &bvh->objectCount);
    carray_free(bvh->pSplitNodes, &bvh->splitNodeCount);
    delete bvh;
}

bool moIntersectBVH(MoBVH bvh, const MoRay& ray, MoTriangle& intersection, MoIntersectBVHAlgorithm* pAlgorithm)
{
    float intersectionDistance = std::numeric_limits<float>::max();
    float bbhits[4];
    std::uint32_t closer, other;

    // Working set
    struct Traversal
    {
        std::uint32_t index;
        float distance;
    };
    std::vector<Traversal> traversal(1);
    std::int32_t stackPtr = 0;

    traversal[stackPtr].index = 0;
    traversal[stackPtr].distance = std::numeric_limits<float>::lowest();

    while (stackPtr >= 0)
    {
        std::uint32_t index = traversal[stackPtr].index;
        float near = traversal[stackPtr].distance;
        stackPtr--;
        const MoBVHSplitNode& node = bvh->pSplitNodes[index];

        if (near > intersectionDistance)
        {
            continue;
        }

        if (node.offset == 0)
        {
            for (std::uint32_t i = 0; i < node.count; ++i)
            {
                const MoTriangle& obj = bvh->pObjects[node.start + i];
                float currentDistance = pAlgorithm->intersectObj(ray, obj);
                if (currentDistance <= 0.0)
                {
                    intersection = obj;
                    return true;
                }
                if (currentDistance < intersectionDistance)
                {
                    intersection = obj;
                    intersectionDistance = currentDistance;
                }
            }
        }
        else
        {
            bool hitLeft = pAlgorithm->intersectBBox(ray, bvh->pSplitNodes[index + 1].boundingBox, &bbhits[0], &bbhits[1]);
            bool hitRight = pAlgorithm->intersectBBox(ray, bvh->pSplitNodes[index + node.offset].boundingBox, &bbhits[2], &bbhits[3]);

            if (hitLeft && hitRight)
            {
                closer = index + 1;
                other = index + node.offset;

                if (bbhits[2] < bbhits[0])
                {
                    std::swap(bbhits[0], bbhits[2]);
                    std::swap(bbhits[1], bbhits[3]);
                    std::swap(closer, other);
                }

                ++stackPtr;
                if (traversal.size() <= stackPtr)
                    traversal.push_back({});
                traversal[stackPtr] = Traversal{other, bbhits[2]};
                ++stackPtr;
                if (traversal.size() <= stackPtr)
                    traversal.push_back({});
                traversal[stackPtr] = Traversal{closer, bbhits[0]};
            }
            else if (hitLeft)
            {
                ++stackPtr;
                if (traversal.size() <= stackPtr)
                    traversal.push_back({});
                traversal[stackPtr] = Traversal{index + 1, bbhits[0]};
            }
            else if (hitRight)
            {
                ++stackPtr;
                if (traversal.size() <= stackPtr)
                    traversal.push_back({});
                traversal[stackPtr] = Traversal{index + node.offset, bbhits[2]};
            }
        }
    }

    return intersectionDistance < std::numeric_limits<float>::max();
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
