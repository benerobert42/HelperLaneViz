//
//  TriangulationHoles.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 12. 14..
//

#include "TriangulationHoles.h"

#include "TriangulationHelpers.h"

// Handles one hole only yet, for ear clipping only
std::vector<Vertex> Triangulation::Helpers::BuildBridgedPolygonImpl(const std::vector<Vertex>& outerVertices,
                                                                    const std::vector<Vertex>& holeVertices) {
    if (outerVertices.size() < 3 || holeVertices.size() < 3) {
        return outerVertices;
    }

    std::vector<Vertex> outer = outerVertices;
    std::vector<Vertex> hole  = holeVertices;

    if (PolygonSignedArea(outer) < 0.0) {
        std::reverse(outer.begin(), outer.end());
    }
    if (PolygonSignedArea(hole) > 0.0) {
        std::reverse(hole.begin(), hole.end());
    }

    // Pick a reference vertex on the hole, rightmost here (max x, then smallest y)
    size_t hIdx = 0;
    float maxX = hole[0].position.x;
    float minY = hole[0].position.y;
    for (size_t i = 1; i < hole.size(); ++i) {
        const float x = hole[i].position.x;
        const float y = hole[i].position.y;
        if (x > maxX || (x == maxX && y < minY)) {
            maxX = x;
            minY = y;
            hIdx = i;
        }
    }
    const simd_float3 hPos = hole[hIdx].position;

    auto isBridgeValid = [&](const simd_float3& a, const simd_float3& b) -> bool {
        // Midpoint should lie inside outer and outside hole
        simd_float3 mid{(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, 1.0f};
        if (!PointInsidePolygon(mid, outer)) {
            return false;
        }
        if (PointInsidePolygon(mid, hole)) {
            return false;
        }

        // The bridge must not intersect any existing edges
        // at this point only outer and hole edge is given
        const size_t outerCount = outer.size();
        for (size_t i = 0; i < outerCount; ++i) {
            const auto& p1 = outer[i].position;
            const auto& p2 = outer[(i + 1) % outerCount].position;
            if (SegmentsIntersect(a, b, p1, p2)) {
                return false;
            }
        }

        const size_t holeCount = hole.size();
        for (size_t i = 0; i < holeCount; ++i) {
            const auto& p1 = hole[i].position;
            const auto& p2 = hole[(i + 1) % holeCount].position;
            if (SegmentsIntersect(a, b, p1, p2)) {
                return false;
            }
        }

        return true;
    };

    // Choose outer vertex to connect to hIdx
    size_t bestOuterIdx = size_t(-1);
    double bestDist2 = std::numeric_limits<double>::infinity();

    // Prefer vertices to the right of the hole vertex
    for (size_t oi = 0; oi < outer.size(); ++oi) {
        const simd_float3 oPos = outer[oi].position;

        if (oPos.x < hPos.x) {
            continue; // skip vertices strictly to the left
        }

        if (!isBridgeValid(hPos, oPos)) {
            continue;
        }

        const simd_float3 dPos = oPos - hPos;
        const double d2 = simd_length_squared(dPos);

        if (d2 < bestDist2) {
            bestDist2 = d2;
            bestOuterIdx = oi;
        }
    }

    // Fallback: if nothing to the right works, allow any visible outer vertex
    if (bestOuterIdx == size_t(-1)) {
        for (size_t oi = 0; oi < outer.size(); ++oi) {
            const simd_float3 oPos = outer[oi].position;

            if (!isBridgeValid(hPos, oPos)) {
                continue;
            }

            const simd_float3 dPos = oPos - hPos;
            const double d2 = simd_length_squared(dPos);

            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestOuterIdx = oi;
            }
        }
    }

    // If we still don't have a candidate, give up and just return the outer
    if (bestOuterIdx == size_t(-1)) {
        return outer;
    }

    // Build final bridged polygon vertex ordering
    //
    // Sequence (conceptual):
    //   outer[0 .. oIdx],
    //   hole[hIdx], hole[hIdx+1], ..., hole[hIdx-1],
    //   hole[hIdx],         // close hole
    //   outer[oIdx],        // return along same bridge
    //   outer[oIdx+1 .. end]
    //
    // This encodes two coincident bridge edges:
    //   outer[oIdx] -> hole[hIdx]
    //   hole[hIdx]  -> outer[oIdx]
    // allowing the final polygon to be simply connected.

    const size_t oIdx = bestOuterIdx;
    std::vector<Vertex> result;
    result.reserve(outer.size() + hole.size() + 2);

    // Chaining the vertices together
    for (size_t i = 0; i <= oIdx; ++i) {
        result.push_back(outer[i]);
    }
    result.push_back(hole[hIdx]);

    // Walk the hole once in its (CW) orientation, starting just after hIdx
    const size_t holeCount = hole.size();
    for (size_t step = 1; step < holeCount; ++step) {
        const size_t idx = (hIdx + holeCount - step) % holeCount;
        result.push_back(hole[idx]);
    }
    result.push_back(hole[hIdx]);
    result.push_back(outer[oIdx]);

    for (size_t i = oIdx + 1; i < outer.size(); ++i) {
        result.push_back(outer[i]);
    }

    return result;
}


bool Triangulation::Helpers::IsTriangleValidWithHoles(const std::vector<Vertex>& vertices,
                                                      int i,
                                                      int j,
                                                      int k,
                                                      const std::vector<Vertex>& outerVertices,
                                                      const std::vector<std::vector<Vertex>>& holes) {
    if (holes.empty()) {
        return true;
    }

    const auto& pA = vertices[i].position;
    const auto& pB = vertices[j].position;
    const auto& pC = vertices[k].position;

    for (const auto& hole : holes) {
        const size_t holeSize = hole.size();

        if (PointInsidePolygon(pA, hole) || PointInsidePolygon(pB, hole) || PointInsidePolygon(pC, hole)) {
            return false;
        }

        // Check if any triangle edge intersects any hole edge
        for (size_t h = 0; h < holeSize; ++h) {
            const auto& h1 = hole[h].position;
            const auto& h2 = hole[(h + 1) % holeSize].position;

            if (SegmentsIntersect(pA, pB, h1, h2) ||
                SegmentsIntersect(pB, pC, h1, h2) ||
                SegmentsIntersect(pC, pA, h1, h2)) {
                return false;
            }
        }
    }

    return true;
}
