#pragma once

#include <linalg.h>

#include <vector>

class MoBBox
{
public:
    MoBBox() {}
    MoBBox(const linalg::aliases::float3& min, const linalg::aliases::float3& max);
    MoBBox(const linalg::aliases::float3& point);

    bool intersect(const linalg::aliases::float3& origin, const linalg::aliases::float3& oneOverDirection,
                   float* t_near = nullptr, float* t_far = nullptr) const;
    void expandToInclude(const linalg::aliases::float3& point);
    void expandToInclude(const MoBBox& box);

    linalg::aliases::float3 min;
    linalg::aliases::float3 max;
    linalg::aliases::float3 extent;
};

struct MoTriangle
{
    linalg::aliases::float3 v0, v1, v2;
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
};

struct MoIntersection
{
    const MoTriangle* object;
    linalg::aliases::float3 point;
    float distance;
};

class MoBVH
{
public:
    MoBVH() {}
    MoBVH(MoTriangle* pTriangles, uint32_t triangleCount);
    bool getIntersection(const linalg::aliases::float3& origin, const linalg::aliases::float3& direction,
                         MoIntersection* intersection, bool anyHit = false) const;

    std::uint32_t nodeCount;
    std::uint32_t leafCount;
    MoTriangle* triangles;
    uint32_t triangleCount;

    struct Split
    {
        MoBBox boundingBox;
        std::uint32_t start;
        std::uint32_t count;
        std::uint32_t offset;
    };
    std::vector<Split> nodes;
};

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