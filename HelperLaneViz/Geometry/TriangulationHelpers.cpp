//
//  TriangulationHelpers.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 12. 14..
//

#include "TriangulationHelpers.h"

double Triangulation::Helpers::Cross2D(const simd_float3& p0, const simd_float3& p1, const simd_float3& p2) {
    simd_float3 vector01 = p1 - p0;
    simd_float3 vector02 = p2 - p0;

    return simd_cross(vector01, vector02).z;
}

double Triangulation::Helpers::EdgeLength(const Vertex& vertexA, const Vertex& vertexB) {
    return simd_distance_squared(vertexB.position, vertexA.position);
}

double Triangulation::Helpers::TrianglePerimeter(const std::vector<Vertex>& vertices,
                                                 uint32_t indexA, uint32_t indexB,
                                                 uint32_t indexC) {
    double edgeLengthAB = EdgeLength(vertices[indexA], vertices[indexB]);
    double edgeLengthBC = EdgeLength(vertices[indexB], vertices[indexC]);
    double edgeLengthCA = EdgeLength(vertices[indexC], vertices[indexA]);

    return edgeLengthAB + edgeLengthBC + edgeLengthCA;
}

double Triangulation::Helpers::TriangleArea(const std::vector<Vertex>& vertices,
                                            uint32_t indexA,
                                            uint32_t indexB,
                                            uint32_t indexC) {
    const simd_float3 ab = vertices[indexB].position - vertices[indexA].position;
    const simd_float3 ac = vertices[indexC].position - vertices[indexA].position;

    return abs(simd_cross(ab, ac).z) * 0.5;
}

double Triangulation::Helpers::PolygonSignedArea(const std::vector<Vertex>& vertices) {
    const size_t vertexCount = vertices.size();
    double signedAreaSum = 0.0;

    for (size_t i = 0; i < vertexCount; ++i) {
        const auto& current = vertices[i].position;
        const auto& next = vertices[(i + 1) % vertexCount].position;

        signedAreaSum += simd_cross(current, next).z;
    }

    return 0.5 * signedAreaSum; // positive = CCW, negative = CW
}

bool Triangulation::Helpers::IsCounterClockwise(const std::vector<Vertex>& vertices,
                                                uint32_t indexA,
                                                uint32_t indexB,
                                                uint32_t indexC) {
    const auto& posA = vertices[indexA].position;
    const auto& posB = vertices[indexB].position;
    const auto& posC = vertices[indexC].position;

    simd_float3 vectorAB = posB - posA;
    simd_float3 vectorAC = posC - posA;

    const double result = simd_cross(vectorAB, vectorAC).z;

    return result > 0.0;
}

// Build vertex ordering for CCW traversal (reverses if input is CW)
std::vector<uint32_t> Triangulation::Helpers::BuildCCWOrder(const std::vector<Vertex>& vertices) {
    const size_t vertexCount = vertices.size();
    std::vector<uint32_t> order(vertexCount);

    const bool isAlreadyCCW = PolygonSignedArea(vertices) >= 0.0;

    for (size_t i = 0; i < vertexCount; ++i) {
        order[i] = isAlreadyCCW
        ? static_cast<uint32_t>(i)
        : static_cast<uint32_t>(vertexCount - 1 - i);
    }

    return order;
}

bool Triangulation::Helpers::PointInTriangle(const simd_float3& p,
                                             const simd_float3& a,
                                             const simd_float3& b,
                                             const simd_float3& c) {
    const double d1 = Cross2D(p, a, b);
    const double d2 = Cross2D(p, b, c);
    const double d3 = Cross2D(p, c, a);

    const bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(hasNeg && hasPos);
}

bool Triangulation::Helpers::PointInsidePolygon(const simd_float3& p, const std::vector<Vertex>& vertices) {
    const size_t n = vertices.size();
    if (n < 3) {
        return false;
    }

    int crossings = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = vertices[i].position;
        const auto& b = vertices[(i + 1) % n].position;

        if ((a.y <= p.y && b.y > p.y) || (b.y <= p.y && a.y > p.y)) {
            float t = (p.y - a.y) / (b.y - a.y);
            if (p.x < a.x + t * (b.x - a.x)) {
                crossings++;
            }
        }
    }
    return (crossings % 2) == 1;
}

bool Triangulation::Helpers::SegmentsIntersect(const simd_float3& p1,
                                               const simd_float3& p2,
                                               const simd_float3& q1,
                                               const simd_float3& q2) {
    const double d1 = Cross2D(p1, p2, q1);
    const double d2 = Cross2D(p1, p2, q2);
    const double d3 = Cross2D(q1, q2, p1);
    const double d4 = Cross2D(q1, q2, p2);

    // Segments intersect if points are on opposite sides
    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) {
        if ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
            return true;
        }
    }
    return false;
}

bool Triangulation::Helpers::IsAdjacent(int vertexA, int vertexB, size_t vertexCount) {
    return (vertexA + 1) % vertexCount == vertexB || (vertexB + 1) % vertexCount == vertexA;
};

Triangulation::Helpers::DiagonalTable Triangulation::Helpers::BuildDiagonalTable(const std::vector<Vertex>& poly) {
    DiagonalTable out;
    const size_t n = poly.size();
    out.isDiagonal.assign(n, std::vector<bool>(n, false));
    out.polygonIsCCW = PolygonSignedArea(poly) > 0.0;

    auto isValidDiagonal = [&](int i, int j) {
        if (i == j || IsAdjacent(i, j, n)) {
            return false;
        }

        const simd_float3& a = poly[i].position;
        const simd_float3& b = poly[j].position;

        // Midpoint must be inside polygon
        simd_float3 mid = 0.5f * (a + b);
        if (!PointInsidePolygon(mid, poly)) {
            return false;
        }

        // Must not properly intersect any polygon edge (except shared endpoints)
        for (int v = 0; v < n; ++v) {
            const int vNext = (v + 1) % n;
            // skip edges incident to i or j
            if (v == i || vNext == i ||v == j || vNext == j) {
                continue;
            }

            const simd_float3& c = poly[v].position;
            const simd_float3& d = poly[vNext].position;

            if (SegmentsIntersect(a, b, c, d)) {
                return false;
            }
        }
        return true;
    };

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (isValidDiagonal(i, j)) {
                out.isDiagonal[i][j] = out.isDiagonal[j][i] = true;
            }
        }
    }

    return out;
}

// O(1) triangle validity check using the diagonal table.
bool Triangulation::Helpers::IsTriangleInsidePolygon(const std::vector<Vertex>& poly,
                                                     int i,
                                                     int j,
                                                     int k,
                                                     const DiagonalTable& diag) {
    const size_t n = poly.size();

    auto edgeAccepted = [&](int a, int b) {
        return IsAdjacent(a, b, n) || diag.isDiagonal[a][b];
    };

    return edgeAccepted(i, j) && edgeAccepted(j, k) && edgeAccepted(k, i);
}

bool Triangulation::Helpers::IsValidEar(const std::vector<Vertex>& vertices,
                                        const std::vector<uint32_t>& polygon,
                                        size_t prevIdx, size_t currIdx, size_t nextIdx,
                                        bool polygonIsCCW) {
    const size_t n = polygon.size();
    if (n < 3) {
        return false;
    }

    const auto& pPrev = vertices[polygon[prevIdx]].position;
    const auto& pCurr = vertices[polygon[currIdx]].position;
    const auto& pNext = vertices[polygon[nextIdx]].position;

    // Check winding: for CCW polygon, ear must be CCW (convex vertex)
    const double cross = Cross2D(pPrev, pCurr, pNext);
    if (polygonIsCCW) {
        if (cross <= 0) return false; // Reflex vertex, not an ear
    } else {
        if (cross >= 0) return false; // For CW polygon, ear must be CW
    }

    // Check that no other polygon vertices are inside this triangle
    for (size_t i = 0; i < n; ++i) {
        if (i == prevIdx || i == currIdx || i == nextIdx) continue;

        const auto& testPoint = vertices[polygon[i]].position;
        if (PointInTriangle(testPoint, pPrev, pCurr, pNext)) {
            return false;
        }
    }

    return true;
}
