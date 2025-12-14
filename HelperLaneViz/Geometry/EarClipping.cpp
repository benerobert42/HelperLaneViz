//
//  EarClipping.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 12. 14..
//

#include "Triangulation.h"
#include "TriangulationHelpers.h"

#include <numeric>

namespace {
using namespace Triangulation::Helpers;

bool IsValidEar(const std::vector<Vertex>& vertices,
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
}

std::vector<uint32_t> Triangulation::EarClippingTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const size_t n = vertices.size();

    if (n < 3) return indices;

    const bool isCCW = PolygonSignedArea(vertices) >= 0.0;

    // Working list of remaining vertex indices
    std::vector<uint32_t> polygon(n);
    std::iota(polygon.begin(), polygon.end(), 0);

    indices.reserve((n - 2) * 3);

    // Clip ears until only 3 vertices remain
    while (polygon.size() > 3) {
        const size_t size = polygon.size();
        bool earFound = false;

        for (size_t i = 0; i < size; ++i) {
            const size_t prev = (i + size - 1) % size;
            const size_t next = (i + 1) % size;

            if (IsValidEar(vertices, polygon, prev, i, next, isCCW)) {
                // Emit triangle
                indices.push_back(polygon[prev]);
                indices.push_back(polygon[i]);
                indices.push_back(polygon[next]);

                // Remove the ear vertex
                polygon.erase(polygon.begin() + static_cast<ptrdiff_t>(i));
                earFound = true;
                break;
            }
        }

        if (!earFound) {
            break;
        }
    }

    // Add final triangle
    if (polygon.size() == 3) {
        indices.push_back(polygon[0]);
        indices.push_back(polygon[1]);
        indices.push_back(polygon[2]);
    }

    return indices;
}
