#pragma once

#include "ShaderTypes.h"

#include <simd/simd.h>
#include <vector>

namespace Triangulation::Helpers {

double Cross2D(const simd_float3& p0, const simd_float3& p1, const simd_float3& p2);
double EdgeLength(const Vertex& vertexA, const Vertex& vertexB);

double TrianglePerimeter(const std::vector<Vertex>& vertices, uint32_t indexA, uint32_t indexB, uint32_t indexC);
double TriangleArea(const std::vector<Vertex>& vertices, uint32_t indexA, uint32_t indexB, uint32_t indexC);
double PolygonSignedArea(const std::vector<Vertex>& vertices);

bool IsCounterClockwise(const std::vector<Vertex>& vertices, uint32_t indexA, uint32_t indexB, uint32_t indexC);
std::vector<uint32_t> BuildCCWOrder(const std::vector<Vertex>& vertices);

bool PointInTriangle(const simd_float3& p,
                     const simd_float3& a,
                     const simd_float3& b,
                     const simd_float3& c);

bool PointInsidePolygon(const simd_float3& p,
                        const std::vector<Vertex>& vertices);

bool SegmentsIntersect(const simd_float3& p1,
                       const simd_float3& p2,
                       const simd_float3& q1,
                       const simd_float3& q2);

struct DiagonalTable {
    std::vector<std::vector<bool>> isDiagonal; // [n][n]
    bool polygonIsCCW = true;
};

bool IsAdjacent(int vertexA, int vertexB, size_t vertexCount);

DiagonalTable BuildDiagonalTable(const std::vector<Vertex>& poly);

bool IsTriangleInsidePolygon(const std::vector<Vertex>& poly,
                             int i,
                             int j,
                             int k,
                             const DiagonalTable& diag);
} // namespace TriangulationHelpers
