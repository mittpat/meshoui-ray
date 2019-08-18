#include "lightmap.h"

#include <assimp/mesh.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
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
    enum : std::uint32_t
    {
        Node_Untouched    = 0xffffffff,
        Node_TouchedTwice = 0xfffffffd,
        Node_Root         = 0xfffffffc,
    };

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
            for (std::uint32_t i = 0; i < node.count; ++i)
            {
                intersection.pTriangle = &bvh->pObjects[node.start + i];
                float currentDistance = pAlgorithm->intersectObj(ray, *intersection.pTriangle);
                if (currentDistance <= 0.0)
                {
                    return true;
                }
                if (currentDistance < intersection.distance)
                {
                    intersection.distance = currentDistance;
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
    moCreateBVH(triangleList->pTriangles, triangleList->triangleCount, &triangleList->bvh, &algo);

    // uv space
    algo.getBoundingBox = [](const MoTriangle& object) -> MoBBox { return object.getUVBoundingBox(); };
    algo.getCentroid = [](const MoTriangle& object) -> float3 { return object.getUVCentroid(); };
    moCreateBVH(triangleList->pTriangles, triangleList->triangleCount, &triangleList->bvhUV, &algo);
}

void moDestroyTriangleList(MoTriangleList triangleList)
{
    moDestroyBVH(triangleList->bvh);
    moDestroyBVH(triangleList->bvhUV);
    carray_free(triangleList->pTriangles, &triangleList->triangleCount);
    delete triangleList;
}

float moGather(MoBVH bvh, MoIntersectBVHAlgorithm* pAlgorithm, const float3& surfacePoint, const float3& surfaceNormal,
               const float3& lightSource, float lightSourcePower, float lightSourceSize, float lightSourceDistance,
               float constantAttenuation, float linearAttenuation, float quadraticAttenuation,
               float3* sphericalVectorList, std::uint32_t sampleCount)
{
#define MO_SURFACE_BIAS 0.01f

    float value = 0.f;
    for (std::uint32_t i = 0; i < sampleCount; ++i)
    {
        float3 delta = (lightSource + sphericalVectorList[i] * lightSourceSize) - surfacePoint;
        float3 rayCast = normalize(delta);
        float diffuseFactor = dot(surfaceNormal, rayCast);
        if (diffuseFactor > 0.0)
        {
            float power = diffuseFactor * lightSourcePower / (constantAttenuation + linearAttenuation * length(delta) + quadraticAttenuation * length2(delta));
            MoIntersectResult intersection = {};
            if (moIntersectBVH(bvh, MoRay(surfacePoint + surfaceNormal * MO_SURFACE_BIAS, rayCast), intersection, pAlgorithm))
            {
                if (intersection.distance < lightSourceDistance)
                {
                    // light is occluded
                    power *= 0.f;
                }
            }
            power /= sampleCount;
            value += power;
        }
    }
    return value;
}

void moGenerateLightMap(const MoTriangleList mesh, MoTextureSample* pTextureSamples, const MoLightmapCreateInfo* pCreateInfo, std::ostream *pLog)
{
    std::mutex logMutex;
    std::uint32_t rowsCompleted = 0;

#define MO_UV_MULTISAMPLE_OFFSET 1.0f

    static std::random_device rd{};
    static std::mt19937 gen{rd()};
    static float3* sSphericalDirectionVectorList = nullptr;
    static float3* sSphericalPositionVectorList = nullptr;
    static std::uint32_t sVectorListSize = 0;

    std::uint32_t sampleCount = std::max({pCreateInfo->ambiantLightingSampleCount, pCreateInfo->directionalLightingSampleCount, pCreateInfo->pointLightingSampleCount});
    if (sVectorListSize < sampleCount)
    {
        std::uniform_real_distribution<float> genX(-1.f, 1.f);
        std::uniform_real_distribution<float> genY(-1.f, 1.f);
        std::uniform_real_distribution<float> genZ(-1.f, 1.f);

        std::uint32_t temp1 = sVectorListSize;
        std::uint32_t temp2 = temp1;
        carray_resize(&sSphericalDirectionVectorList, &temp1, sampleCount);
        carray_resize(&sSphericalPositionVectorList, &temp2, sampleCount);
        for (std::uint32_t at = sVectorListSize; at < sampleCount;)
        {
            float3 vect(genX(gen), genY(gen), genZ(gen));
            if (length2(vect) < 1.f)
            {
                sSphericalPositionVectorList[at] = vect;
                sSphericalDirectionVectorList[at++] = normalize(vect);
            }
        }
        sVectorListSize = sampleCount;
    }

    MoIntersectBVHAlgorithm intersectAlgorithm;
    intersectAlgorithm.intersectObj = [](const MoRay& ray, const MoTriangle& object) -> float
    {
        float3 intersectionPoint;
        if (moRayTriangleIntersect(ray, object, intersectionPoint))
        {
            return length(intersectionPoint - ray.origin);
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
    for (std::uint32_t row = 0; row < pCreateInfo->height; ++row)
    {
        if (threads.size() > std::thread::hardware_concurrency())
        {
            threads.front().join();
            threads.pop_front();
        }
        threads.emplace_back(std::thread([&, row]()
        {
            for (std::uint32_t column = 0; column < pCreateInfo->width; ++column)
            {
                float2 uv((column + 0.5f) / float(pCreateInfo->width), (row + 0.5f) / float(pCreateInfo->height));
                MoIntersectResult intersectionUV = {};

                std::uint32_t index = column;
                if (pCreateInfo->flipY == 1)
                {
                    index += row * pCreateInfo->width;
                }
                else
                {
                    index += (pCreateInfo->height - row - 1) * pCreateInfo->width;
                }

                float3 multiTexels[] = {float3(uv, -1.0f),
                                         float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->width),  MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->height)), -1.0f),
                                         float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->width), -MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->height)), -1.0f),
                                         float3(uv + float2( MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->width), -MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->height)), -1.0f),
                                         float3(uv + float2(-MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->width),  MO_UV_MULTISAMPLE_OFFSET / float(pCreateInfo->height)), -1.0f)
                                        };
                for (float3 singleTexel : multiTexels)
                {
                    if (moIntersectBVH(mesh->bvhUV, MoRay(singleTexel, {0,0,1}), intersectionUV, &uvIntersectAlgorithm))
                    {
                        const MoTriangle& triangle = *intersectionUV.pTriangle;

                        float3 barycentricCoordinates = triangle.uvBarycentric(uv);
                        float3 surfacePoint = triangle.v0 * barycentricCoordinates[0]
                                + triangle.v1 * barycentricCoordinates[1]
                                + triangle.v2 * barycentricCoordinates[2];

                        float3 surfaceNormal = triangle.n0 * barycentricCoordinates[0]
                                + triangle.n1 * barycentricCoordinates[1]
                                + triangle.n2 * barycentricCoordinates[2];
                        surfaceNormal = normalize(surfaceNormal);

                        // ambiant
                        float value = moGather(mesh->bvh, &intersectAlgorithm, surfacePoint, surfaceNormal,
                            surfacePoint, pCreateInfo->ambiantLightingPower * 2.f/*top hemisphere*/ * 2.f/*white point*/, 1.f, pCreateInfo->ambiantOcclusionDistance,
                                               1.f, 0.f, 0.f,
                            sSphericalDirectionVectorList, pCreateInfo->ambiantLightingSampleCount);

                        // directional
                        for (std::uint32_t j = 0; j < pCreateInfo->directionalLightSourceCount; ++j)
                        {
                            float relativeSamplingRadius = std::sin(pCreateInfo->pDirectionalLightSources[j].angularSize);
                            value += moGather(mesh->bvh, &intersectAlgorithm, surfacePoint, surfaceNormal,
                                surfacePoint + pCreateInfo->pDirectionalLightSources[j].direction, pCreateInfo->pDirectionalLightSources[j].power, relativeSamplingRadius, std::numeric_limits<float>::max(),
                                              1.f, 0.f, 0.f,
                                sSphericalPositionVectorList, pCreateInfo->directionalLightingSampleCount);
                        }

                        // point
                        for (std::uint32_t j = 0; j < pCreateInfo->pointLightSourceCount; ++j)
                        {
                            value += moGather(mesh->bvh, &intersectAlgorithm, surfacePoint, surfaceNormal,
                                pCreateInfo->pPointLightSources[j].position, pCreateInfo->pPointLightSources[j].power, pCreateInfo->pPointLightSources[j].size, length(pCreateInfo->pPointLightSources[j].position - surfacePoint),
                                              pCreateInfo->pPointLightSources[j].constantAttenuation, pCreateInfo->pPointLightSources[j].linearAttenuation, pCreateInfo->pPointLightSources[j].quadraticAttenuation,
                                sSphericalPositionVectorList, pCreateInfo->pointLightingSampleCount);
                        }

                        pTextureSamples[index].x = pTextureSamples[index].y = pTextureSamples[index].z = std::uint8_t(std::min(255.f, value * 255));
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
        }));
    }
    for (auto & thread : threads)
    {
        thread.join();
    }

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
