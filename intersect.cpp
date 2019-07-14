#include "intersect.h"

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

    // Adapted from the Möller–Trumbore intersection algorithm
    bool rayTriangleIntersect(const linalg::aliases::float3 & origin,
                              const linalg::aliases::float3 & direction,
                              const MoTriangle & triangle,
                              linalg::aliases::float3 & intersectionPoint)
    {
        const float EPSILON = 0.0000001f;
        const linalg::aliases::float3 & vertex0 = triangle.v0;
        const linalg::aliases::float3 & vertex1 = triangle.v1;
        const linalg::aliases::float3 & vertex2 = triangle.v2;
        const linalg::aliases::float3 edge1 = vertex1 - vertex0;
        const linalg::aliases::float3 edge2 = vertex2 - vertex0;
        const linalg::aliases::float3 h = linalg::cross(direction, edge2);
        const float a = linalg::dot(edge1, h);
        if (a > -EPSILON && a < EPSILON)
        {
            // This ray is parallel to this triangle.
            return false;
        }

        const float f = 1.0/a;
        const linalg::aliases::float3 s = origin - vertex0;
        const float u = f * linalg::dot(s, h);
        if (u < 0.0 || u > 1.0)
        {
            return false;
        }

        const linalg::aliases::float3 q = linalg::cross(s, edge1);
        const float v = f * linalg::dot(direction, q);
        if (v < 0.0 || u + v > 1.0)
        {
            return false;
        }

        // At this stage we can compute t to find out where the intersection point is on the line.
        const float t = f * linalg::dot(edge2, q);
        if (t > EPSILON)
        {
            // ray intersection
            intersectionPoint = origin + direction * t;
            return true;
        }

        // This means that there is a line intersection but not a ray intersection.
        return false;
    }
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
bool MoBBox::intersect(const float3& origin, const float3& oneOverDirection, float* t_near, float* t_far) const
{
    float tx1 = (min.x - origin.x) * oneOverDirection.x;
    float tx2 = (max.x - origin.x) * oneOverDirection.x;

    float tmin = std::min(tx1, tx2);
    float tmax = std::max(tx1, tx2);

    float ty1 = (min.y - origin.y) * oneOverDirection.y;
    float ty2 = (max.y - origin.y) * oneOverDirection.y;

    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    float tz1 = (min.z - origin.z) * oneOverDirection.z;
    float tz2 = (max.z - origin.z) * oneOverDirection.z;

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    if (t_near)
        *t_near = tmin;

    if (t_far)
        *t_far = tmax;

    return tmax >= tmin;
}

// Adapted from Brandon Pelfrey's "A Simple, Optimized Bounding Volume Hierarchy for Ray/Object Intersection Testing"
MoBVH::MoBVH(const std::vector<MoTriangle>& _triangles, std::uint32_t _leafSize)
    : nodeCount(0)
    , leafCount(0)
    , leafSize(_leafSize)
    , triangles(_triangles)
    , nodes()
{
    std::uint32_t stackPtr = 0;

    struct Entry
    {
        std::uint32_t parent;
        std::uint32_t start;
        std::uint32_t end;
    };
    std::vector<Entry> entries(1);
    entries[stackPtr].start = 0;
    entries[stackPtr].end = triangles.size();
    entries[stackPtr].parent = Node_Root;
    stackPtr++;

    Split node;
    nodes.reserve(triangles.size() * 2);

    while (stackPtr > 0)
    {
        Entry &entry = entries[--stackPtr];
        std::uint32_t start = entry.start;
        std::uint32_t end = entry.end;
        std::uint32_t count = end - start;

        nodeCount++;
        node.start = start;
        node.count = count;
        node.offset = Node_Untouched;

        MoBBox boundingBox(triangles[start].getBoundingBox());
        MoBBox boundingBoxCentroids(triangles[start].getCentroid());
        for (std::uint32_t i = start + 1; i < end; ++i)
        {
            boundingBox.expandToInclude(triangles[i].getBoundingBox());
            boundingBoxCentroids.expandToInclude(triangles[i].getCentroid());
        }
        node.boundingBox = boundingBox;

        if (count <= leafSize)
        {
            node.offset = 0;
            leafCount++;
        }

        nodes.push_back(node);
        if (entry.parent != Node_Root)
        {
            nodes[entry.parent].offset--;
            if (nodes[entry.parent].offset == Node_TouchedTwice)
            {
                nodes[entry.parent].offset = nodeCount - 1 - entry.parent;
            }
        }

        if (node.offset == 0)
        {
            continue;
        }

        // find longest bbox side
        std::uint32_t splitDimension = 0;
        if (boundingBoxCentroids.extent.y > boundingBoxCentroids.extent.x)
        {
            splitDimension = 1;
            if (boundingBoxCentroids.extent.z > boundingBoxCentroids.extent.y)
            {
                splitDimension = 2;
            }
        }
        else if (boundingBoxCentroids.extent.z > boundingBoxCentroids.extent.x)
        {
            splitDimension = 2;
        }

        float splitLength = .5f * (boundingBoxCentroids.min[splitDimension] + boundingBoxCentroids.max[splitDimension]);

        std::uint32_t mid = start;
        for (std::uint32_t i = start; i < end; ++i)
        {
            if (triangles[i].getCentroid()[splitDimension] < splitLength)
            {
                std::swap(triangles[i], triangles[mid]);
                ++mid;
            }
        }

        if (mid == start || mid == end)
        {
            mid = start + (end-start) / 2;
        }

        // left
        if (entries.size() <= stackPtr)
            entries.push_back({});
        entries[stackPtr].start = mid;
        entries[stackPtr].end = end;
        entries[stackPtr].parent = nodeCount - 1;
        stackPtr++;

        // right
        if (entries.size() <= stackPtr)
            entries.push_back({});
        entries[stackPtr].start = start;
        entries[stackPtr].end = mid;
        entries[stackPtr].parent = nodeCount - 1;
        stackPtr++;
    }

    nodes.resize(nodeCount);
}

bool MoBVH::getIntersection(const float3& origin, const float3& direction,
                                              MoIntersection* intersection, bool anyHit) const
{
    float3 oneOverDirection(1 / direction.x, 1 / direction.y, 1 / direction.z);

    intersection->distance = std::numeric_limits<float>::max();
    intersection->object = nullptr;
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
        const Split& node = nodes[index];

        if (near > intersection->distance)
        {
            continue;
        }

        if (node.offset == 0)
        {
            for (std::uint32_t i = 0; i < node.count; ++i)
            {
                MoIntersection current;

                const MoTriangle& obj = triangles[node.start + i];
                if (rayTriangleIntersect(origin, direction, obj, current.point))
                {
                    current.object = &obj;
                    current.distance = length(current.point - origin);
                    if (anyHit)
                    {
                        return true;
                    }
                    if (current.distance < intersection->distance)
                    {
                        *intersection = current;
                    }
                }
            }
        }
        else
        {
            bool hitLeft = nodes[index + 1].boundingBox.intersect(origin, oneOverDirection, &bbhits[0], &bbhits[1]);
            bool hitRight = nodes[index + node.offset].boundingBox.intersect(origin, oneOverDirection, &bbhits[2], &bbhits[3]);

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

    return intersection->object != nullptr;
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
