#include "lightmap.h"

#include <assimp/mesh.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

using namespace linalg;
using namespace linalg::aliases;

std::uint32_t carray_nextPowerOfTwo(std::uint32_t v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

template<typename T>
void carray_resize(const T** array, std::uint32_t* currentSize, std::uint32_t newSize)
{
    T** local = const_cast<T**>(array);

    std::uint32_t previous = carray_nextPowerOfTwo(*currentSize);
    std::uint32_t next = carray_nextPowerOfTwo(newSize);
    if (previous != next)
    {
        *local = reinterpret_cast<T*>(realloc(*local, next * sizeof(T)));
    }
    *currentSize = newSize;
}

template<typename T>
inline void carray_resize(T** array, std::uint32_t* currentSize, std::uint32_t newSize)
{
    carray_resize(const_cast<const T**>(array), currentSize, newSize);
}

template<typename T>
void carray_push_back(const T** array, std::uint32_t* size, T value)
{
    T** local = const_cast<T**>(array);

    std::uint32_t previous = carray_nextPowerOfTwo(*size);
    std::uint32_t next = carray_nextPowerOfTwo(++(*size));
    if (previous != next)
    {
        *local = reinterpret_cast<T*>(realloc(*local, next * sizeof(T)));
    }
    (*local)[*size-1] = value;
}

template<typename T>
inline void carray_push_back(T** array, std::uint32_t* size, T value)
{
    carray_push_back(const_cast<const T**>(array), size, value);
}

template<typename T>
void carray_copy(const T* source, const T* destination, std::uint32_t count)
{
    T* lsource = const_cast<T*>(source);
    T* ldestination = const_cast<T*>(destination);

    memcpy(ldestination, lsource, count * sizeof(T));
}

template<typename T>
inline void carray_copy(T* source, T* destination, std::uint32_t count)
{
    carray_push_back(const_cast<const T*>(source), const_cast<const T*>(destination), count);
}

template<typename T>
inline void carray_free(const T* array, std::uint32_t* size)
{
    T* local = const_cast<T*>(array);

    free(local);
    *size = 0;
}

MoBBox::MoBBox(const float3& _min, const float3& _max)
    : min(_min)
    , max(_max)
{
}

MoBBox::MoBBox(const float3& point)
    : min(point)
    , max(point)
{
}

std::uint32_t MoBBox::longestSide() const
{
    float3 extent = max - min;

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
}

void MoBBox::expandToInclude(const MoBBox& box)
{
    min.x = std::min(min.x, box.min.x);
    min.y = std::min(min.y, box.min.y);
    min.z = std::min(min.z, box.min.z);
    max.x = std::max(max.x, box.max.x);
    max.y = std::max(max.y, box.max.y);
    max.z = std::max(max.z, box.max.z);
}

// adapted from Tavian Barnes' "Fast, Branchless Ray/Bounding Box Intersections"
bool MoBBox::intersect(const MoRay& ray, float &t_near, float &t_far) const
{
    float tx1 = (min.x - ray.origin.x) * ray.oneOverDirection.x;
    float tx2 = (max.x - ray.origin.x) * ray.oneOverDirection.x;

    t_near = std::min(tx1, tx2);
    t_far = std::max(tx1, tx2);

    float ty1 = (min.y - ray.origin.y) * ray.oneOverDirection.y;
    float ty2 = (max.y - ray.origin.y) * ray.oneOverDirection.y;

    t_near = std::max(t_near, std::min(ty1, ty2));
    t_far = std::min(t_far, std::max(ty1, ty2));

    float tz1 = (min.z - ray.origin.z) * ray.oneOverDirection.z;
    float tz2 = (max.z - ray.origin.z) * ray.oneOverDirection.z;

    t_near = std::max(t_near, std::min(tz1, tz2));
    t_far = std::min(t_far, std::max(tz1, tz2));

    return t_far >= t_near;
}

float3 MoTriangle::getUVBarycentric(const float2 &uv) const
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

void MoTriangle::getSurface(const float3& barycentricCoordinates, float3 &point, float3 &normal) const
{
    point = v0 * barycentricCoordinates[0]
          + v1 * barycentricCoordinates[1]
          + v2 * barycentricCoordinates[2];

    normal = n0 * barycentricCoordinates[0]
           + n1 * barycentricCoordinates[1]
           + n2 * barycentricCoordinates[2];
    normal = normalize(normal);
}

// Adapted from the Möller–Trumbore intersection algorithm
bool moRayTriangleIntersect(const MoRay &ray, const MoTriangle &triangle,
                            float &t, float &u, float &v)
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
    u = f * linalg::dot(s, h);
    if (u < 0.0 || u > 1.0)
    {
        return false;
    }

    const float3 q = linalg::cross(s, edge1);
    v = f * linalg::dot(ray.direction, q);
    if (v < 0.0 || u + v > 1.0)
    {
        return false;
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    t = f * linalg::dot(edge2, q);
    if (t > EPSILON)
    {
        // ray intersection
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

void moCreateBVH(MoTriangleList triangleList, MoBVH* pBVH, MoCreateBVHAlgorithm *pAlgorithm)
{
    enum : std::uint32_t
    {
        Node_Untouched    = 0xffffffff,
        Node_TouchedTwice = 0xfffffffd,
        Node_Root         = 0xfffffffc,
    };

    MoBVH bvh = *pBVH = new MoBVH_T();
    *bvh = {};
    bvh->triangleList = triangleList;    

    std::uint32_t currentSize = 0;
    const std::uint32_t* indices = nullptr;
    carray_resize(&indices, &currentSize, triangleList->triangleCount);
    for (std::uint32_t i = 0; i < currentSize; ++i)
        const_cast<std::uint32_t*>(indices)[i] = i;

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
    entries[stackPtr].end = currentSize;
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
        splitNode.offset = Node_Untouched;

        MoBBox boundingBox = pAlgorithm->getBoundingBox(triangleList->pTriangles[indices[start]]);
        MoBBox boundingBoxCentroids = pAlgorithm->getCentroid(triangleList->pTriangles[indices[start]]);
        for (std::uint32_t i = start + 1; i < end; ++i)
        {
            boundingBox.expandToInclude(pAlgorithm->getBoundingBox(triangleList->pTriangles[indices[i]]));
            boundingBoxCentroids.expandToInclude(pAlgorithm->getCentroid(triangleList->pTriangles[indices[i]]));
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
            if (pAlgorithm->getCentroid(triangleList->pTriangles[indices[i]])[splitDimension] < splitLength)
            {
                std::uint32_t* lObjects = const_cast<std::uint32_t*>(indices);
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

    for (std::uint32_t i = 0; i < splitNodeCount; ++i)
    {
        MoBVHSplitNode& splitNode = const_cast<MoBVHSplitNode&>(bvh->pSplitNodes[i]);
        splitNode.start = splitNode.offset == 0 ? indices[splitNode.start] : 0;
    }

    carray_resize(&bvh->pSplitNodes, &bvh->splitNodeCount, splitNodeCount);
    carray_free(entries, &entryCount);    
    carray_free(indices, &currentSize);
}

void moDestroyBVH(MoBVH bvh)
{
    carray_free(bvh->pSplitNodes, &bvh->splitNodeCount);
    delete bvh;
}

void moSetIntersectBVHAlgorithm_TriangleMesh(MoIntersectBVHAlgorithm *pAlgorithm)
{
    pAlgorithm->intersectObj = [](const MoRay& ray, const MoTriangle& object, float& u, float& v) -> float
    {
        float t;
        if (moRayTriangleIntersect(ray, object, t, u, v))
        {
            return t;
        }
        return std::numeric_limits<float>::max();
    };
    pAlgorithm->intersectBBox = [](const MoRay& ray, const MoBBox& bbox, float& t_near, float& t_far) -> bool
    {
        return bbox.intersect(ray, t_near, t_far);
    };
}

void moSetIntersectBVHAlgorithm_Texcoords(MoIntersectBVHAlgorithm *pAlgorithm)
{
    pAlgorithm->intersectObj = [](const MoRay& ray, const MoTriangle& object, float&, float&) -> float
    {
        if (moTexcoordInTriangleUV(ray.origin.xy(), object))
        {
            return 0.0;
        }
        return std::numeric_limits<float>::max();
    };
    pAlgorithm->intersectBBox = [](const MoRay& ray, const MoBBox& bbox, float& t_near, float& t_far) -> bool
    {
        return bbox.intersect(ray, t_near, t_far);
    };
}

bool moIntersectBVH(MoBVH bvh, const MoRay& ray, MoIntersectResult& intersection, MoIntersectBVHAlgorithm* pAlgorithm)
{
    intersection.distance = std::numeric_limits<float>::max();
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

        if (near > intersection.distance)
        {
            continue;
        }

        if (node.offset == 0)
        {
            float u, v;
            const MoTriangle& triangle = bvh->triangleList->pTriangles[node.start];
            float currentDistance = pAlgorithm->intersectObj(ray, triangle, u, v);
            if (currentDistance <= 0.0)
            {
                intersection.pTriangle = &triangle;
                return true;
            }
            if (currentDistance < intersection.distance)
            {
                intersection.pTriangle = &triangle;
                intersection.barycentric = {1.f - u - v, u, v};
                intersection.distance = currentDistance;
            }
        }
        else
        {
            bool hitLeft = pAlgorithm->intersectBBox(ray, bvh->pSplitNodes[index + 1].boundingBox, bbhits[0], bbhits[1]);
            bool hitRight = pAlgorithm->intersectBBox(ray, bvh->pSplitNodes[index + node.offset].boundingBox, bbhits[2], bbhits[3]);

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

    return intersection.distance < std::numeric_limits<float>::max();
}

void moCreateTriangleList(const aiMesh * ai_mesh, MoTriangleList *pTriangleList)
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
    moCreateBVH(triangleList, &triangleList->bvh, &algo);

    // uv space
    algo.getBoundingBox = [](const MoTriangle& object) -> MoBBox { return object.getUVBoundingBox(); };
    algo.getCentroid = [](const MoTriangle& object) -> float3 { return object.getUVCentroid(); };
    moCreateBVH(triangleList, &triangleList->bvhUV, &algo);
}

void moDestroyTriangleList(MoTriangleList triangleList)
{
    moDestroyBVH(triangleList->bvh);
    moDestroyBVH(triangleList->bvhUV);
    carray_free(triangleList->pTriangles, &triangleList->triangleCount);
    delete triangleList;
}

float3 moNextSphericalSample(std::mt19937 * generator, bool direction = false)
{
    std::uniform_real_distribution<float> genX(-1.f, 1.f);
    std::uniform_real_distribution<float> genY(-1.f, 1.f);
    std::uniform_real_distribution<float> genZ(-1.f, 1.f);
    float3 vect;
    do
    {
        vect = float3(genX(*generator), genY(*generator), genZ(*generator));
    }
    while (length2(vect) > 1.f);
    if (direction)
    {
        vect = normalize(vect);
    }
    return vect;
}

#define MO_SURFACE_BIAS 0.01f
float3 moGatherAmbient(MoBVH bvh, MoIntersectBVHAlgorithm* pAlgorithm,
                       const float3& surfacePoint, const float3& surfaceNormal,
                       float lightSourcePower, float lightSourceDistance,
                       std::mt19937* generator, std::uint32_t sampleCount)
{
    if (sampleCount == 0)
    {
        return {0.f,0.f,0.f};
    }

    float value = 0.f;
    for (std::uint32_t i = 0; i < sampleCount; ++i)
    {
        float3 rayCast = moNextSphericalSample(generator, true);
        float diffuseFactor = dot(surfaceNormal, rayCast);
        if (diffuseFactor > 0.f)
        {
            MoIntersectResult intersection = {};
            if (moIntersectBVH(bvh, MoRay(surfacePoint + surfaceNormal * MO_SURFACE_BIAS, rayCast), intersection, pAlgorithm)
                    && intersection.distance < lightSourceDistance)
            {
                // light is occluded
            }
            else
            {
                value += diffuseFactor * lightSourcePower;
            }
        }
    }
    return float3{value, value, value} / sampleCount;
}

float3 moGatherLight(MoBVH bvh, MoIntersectBVHAlgorithm* pAlgorithm,
                     const float3& surfacePoint, const float3& surfaceNormal,
                     const float3& lightSource, float lightSourcePower, float lightSourceSize, float lightSourceDistance,
                     float constantAttenuation, float linearAttenuation, float quadraticAttenuation,
                     std::mt19937 * generator, std::uint32_t sampleCount)
{
    if (sampleCount == 0)
    {
        return {0.f,0.f,0.f};
    }

    float3 mainRay = lightSource - surfacePoint;

    if (sampleCount >= 8)
    {
        // edge-test for AA
        bool doMulisampling = false;
        static const float3 aasamples[7] = {{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}, {-1,0,0}, {0,-1,0}, {0,0,-1}};
        for (const float3& sample : aasamples)
        {
            float3 rayCast = normalize(mainRay + sample * lightSourceSize * 1.5f);
            float diffuseFactor = dot(surfaceNormal, rayCast);
            if (diffuseFactor > 0.f)
            {
                MoIntersectResult intersection = {};
                if (moIntersectBVH(bvh, MoRay(surfacePoint + surfaceNormal * MO_SURFACE_BIAS, rayCast), intersection, pAlgorithm))
                {
                    float3 impactPoint, impactNormal;
                    intersection.pTriangle->getSurface(intersection.barycentric, impactPoint, impactNormal);
                    float hit = dot(-rayCast, impactNormal);
                    if (hit > -0.9f && hit < 0.9f)
                    {
                        doMulisampling = true;
                        break;
                    }
                }
            }
        }
        if (!doMulisampling)
        {
            sampleCount = 1;
        }
    }

    float value = 0.f;
    for (std::uint32_t i = 0; i < sampleCount; ++i)
    {
        float3 delta = mainRay;
        if (i > 0) delta += moNextSphericalSample(generator, false) * lightSourceSize;
        float3 rayCast = normalize(delta);
        float diffuseFactor = dot(surfaceNormal, rayCast);
        if (diffuseFactor > 0.f)
        {
            MoIntersectResult intersection = {};
            if (moIntersectBVH(bvh, MoRay(surfacePoint + surfaceNormal * MO_SURFACE_BIAS, rayCast), intersection, pAlgorithm)
                    && intersection.distance < lightSourceDistance)
            {
                // light is occluded
            }
            else
            {
                value += diffuseFactor * lightSourcePower / (constantAttenuation + linearAttenuation * length(delta) + quadraticAttenuation * length2(delta));
            }
        }
    }
    return float3{value, value, value} / sampleCount;
}

void moParallelFor(std::int32_t from, std::int32_t to, std::function<void(std::int32_t)> f, std::uint32_t jobs = 0)
{
    if (jobs != 1)
    {
        if (jobs == 0) jobs = std::thread::hardware_concurrency();
        std::deque<std::thread> threads;
        for (std::int32_t at = from; at < to; ++at)
        {
            if (threads.size() > jobs)
            {
                threads.front().join();
                threads.pop_front();
            }
            threads.emplace_back(std::thread([f, at](){ f(at); }));
        }
        for (auto& thread : threads)
        {
            thread.join();
        }
    }
    else
    {
        for (std::int32_t at = from; at < to; ++at)
        {
            f(at);
        }
    }
}

#define MO_UV_MULTISAMPLE_OFFSET 1.0f

void moGenerateLightMap(const MoTriangleList mesh, MoTextureSample* pTextureSamples, const MoLightmapCreateInfo* pCreateInfo, std::ostream *pLog)
{
    std::mt19937 generator;

    std::mutex logMutex;
    std::uint32_t rowsCompleted = 0;

    MoIntersectBVHAlgorithm intersectAlgorithm;
    moSetIntersectBVHAlgorithm_TriangleMesh(&intersectAlgorithm);
    MoIntersectBVHAlgorithm uvIntersectAlgorithm;
    moSetIntersectBVHAlgorithm_Texcoords(&uvIntersectAlgorithm);

    moParallelFor(0, pCreateInfo->size.y, [&](std::int32_t row) {
        for (std::uint32_t column = 0; column < pCreateInfo->size.x; ++column)
        {
            std::mt19937 coherentGenerator;
            std::mt19937* localGenerator = &generator;
            if (pCreateInfo->despeckle == 1) localGenerator = &coherentGenerator;

            float2 uv((column + 0.5f) / float(pCreateInfo->size.x),
                      (row + 0.5f) / float(pCreateInfo->size.y));
            MoIntersectResult intersectionUV = {};

            std::uint32_t index = column;
            if (pCreateInfo->flipY == 1) index += row * pCreateInfo->size.x;
            else index += (pCreateInfo->size.y - row - 1) * pCreateInfo->size.x;

            float3 multiTexels[] = {float3(uv, -1.0f),
                                     float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x),  MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f),
                                     float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x), -MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f),
                                     float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x), -MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f),
                                     float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x),  MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f)
                                    };
            for (float3 singleTexel : multiTexels)
            {
                if (moIntersectBVH(mesh->bvhUV, MoRay(singleTexel, {0,0,1}), intersectionUV, &uvIntersectAlgorithm))
                {
                    const MoTriangle& triangle = *intersectionUV.pTriangle;

                    float3 surfacePoint, surfaceNormal;
                    triangle.getSurface(triangle.getUVBarycentric(uv), surfacePoint, surfaceNormal);

                    // ambient
                    float3 value = moGatherAmbient(mesh->bvh, &intersectAlgorithm, surfacePoint, surfaceNormal,
                        pCreateInfo->ambientLightingPower * 2.0f/*top hemisphere*/ * 2.0f/*white point*/, pCreateInfo->ambientOcclusionDistance,
                        localGenerator, pCreateInfo->ambientLightingSampleCount);

                    // directional
                    for (std::uint32_t j = 0; j < pCreateInfo->directionalLightSourceCount; ++j)
                    {
                        float relativeSamplingRadius = std::sin(pCreateInfo->pDirectionalLightSources[j].angularSize);
                        value += moGatherLight(mesh->bvh, &intersectAlgorithm, surfacePoint, surfaceNormal,
                            surfacePoint + pCreateInfo->pDirectionalLightSources[j].direction, pCreateInfo->pDirectionalLightSources[j].power,
                            relativeSamplingRadius, std::numeric_limits<float>::max(), 1.f, 0.f, 0.f,
                            localGenerator, pCreateInfo->directionalLightingSampleCount);
                    }

                    // point
                    for (std::uint32_t j = 0; j < pCreateInfo->pointLightSourceCount; ++j)
                    {
                        value += moGatherLight(mesh->bvh, &intersectAlgorithm, surfacePoint, surfaceNormal,
                            pCreateInfo->pPointLightSources[j].position, pCreateInfo->pPointLightSources[j].power,
                            pCreateInfo->pPointLightSources[j].size, length(pCreateInfo->pPointLightSources[j].position - surfacePoint),
                            pCreateInfo->pPointLightSources[j].constantAttenuation, pCreateInfo->pPointLightSources[j].linearAttenuation, pCreateInfo->pPointLightSources[j].quadraticAttenuation,
                            localGenerator, pCreateInfo->pointLightingSampleCount);
                    }

                    pTextureSamples[index].x = std::uint8_t(std::min(255.f, value.x * 255.999f));
                    pTextureSamples[index].y = std::uint8_t(std::min(255.f, value.y * 255.999f));
                    pTextureSamples[index].z = std::uint8_t(std::min(255.f, value.z * 255.999f));
                    pTextureSamples[index].w = pCreateInfo->nullColor.w;

                    break;
                }
            }
            if (pTextureSamples[index].w == 0)
            {
                pTextureSamples[index] = pCreateInfo->nullColor;
            }
        }

        if (pLog)
        {
            std::lock_guard<std::mutex> lock(logMutex);
            ++rowsCompleted;
            if (rowsCompleted <= 1) *pLog << "\r" << rowsCompleted << " line generated" << std::flush;
            else *pLog << "\r" << rowsCompleted << " lines generated" << std::flush;
        }
    }, pCreateInfo->jobs);

    if (pLog)
    {
        *pLog << std::endl << "generating done" << std::endl;
    }
}

void moGenerateNormalMap(const MoTriangleList mesh, MoTextureSample* pTextureSamples, const MoLightmapCreateInfo* pCreateInfo, std::ostream *pLog)
{
    std::mt19937 generator;

    std::mutex logMutex;
    std::uint32_t rowsCompleted = 0;

    MoIntersectBVHAlgorithm intersectAlgorithm;
    moSetIntersectBVHAlgorithm_TriangleMesh(&intersectAlgorithm);
    MoIntersectBVHAlgorithm uvIntersectAlgorithm;
    moSetIntersectBVHAlgorithm_Texcoords(&uvIntersectAlgorithm);

    moParallelFor(0, pCreateInfo->size.y, [&](std::int32_t row) {
        for (std::uint32_t column = 0; column < pCreateInfo->size.x; ++column)
        {
            std::mt19937 coherentGenerator;
            std::mt19937* localGenerator = &generator;
            if (pCreateInfo->despeckle == 1) localGenerator = &coherentGenerator;

            float2 uv((column + 0.5f) / float(pCreateInfo->size.x),
                      (row + 0.5f) / float(pCreateInfo->size.y));
            MoIntersectResult intersectionUV = {};

            std::uint32_t index = column;
            if (pCreateInfo->flipY == 1) index += row * pCreateInfo->size.x;
            else index += (pCreateInfo->size.y - row - 1) * pCreateInfo->size.x;

            float3 multiTexels[] = {float3(uv, -1.0f),
                                     float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x),  MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f),
                                     float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x), -MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f),
                                     float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x), -MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f),
                                     float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.x),  MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->size.y)), -1.0f)
                                    };
            for (float3 singleTexel : multiTexels)
            {
                if (moIntersectBVH(mesh->bvhUV, MoRay(singleTexel, {0,0,1}), intersectionUV, &uvIntersectAlgorithm))
                {
                    const MoTriangle& triangle = *intersectionUV.pTriangle;

                    float3 surfacePoint, surfaceNormal;
                    triangle.getSurface(triangle.getUVBarycentric(uv), surfacePoint, surfaceNormal);

                    float3 value = surfaceNormal / 2 + float3(0.5f,0.5f,0.5f);

                    pTextureSamples[index].x = std::uint8_t(std::min(255.f, value.x * 255.999f));
                    pTextureSamples[index].y = std::uint8_t(std::min(255.f, value.y * 255.999f));
                    pTextureSamples[index].z = std::uint8_t(std::min(255.f, value.z * 255.999f));
                    pTextureSamples[index].w = pCreateInfo->nullColor.w;

                    break;
                }
            }
            if (pTextureSamples[index].w == 0)
            {
                pTextureSamples[index] = pCreateInfo->nullColor;
            }
        }

        if (pLog)
        {
            std::lock_guard<std::mutex> lock(logMutex);
            ++rowsCompleted;
            if (rowsCompleted <= 1) *pLog << "\r" << rowsCompleted << " line generated" << std::flush;
            else *pLog << "\r" << rowsCompleted << " lines generated" << std::flush;
        }
    }, pCreateInfo->jobs);

    if (pLog)
    {
        *pLog << std::endl << "generating done" << std::endl;
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
