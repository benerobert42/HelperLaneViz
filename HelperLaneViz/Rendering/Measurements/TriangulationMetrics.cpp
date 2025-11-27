//
//  TriangulationMetrics.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#include "TriangulationMetrics.h"
#include "Triangulation.h"
#include "TileMetrics.h"

namespace TriMetrics {

static std::vector<ttm::TrianglePx>
toTrianglesPx(const std::vector<Vertex>& vertices,
              const std::vector<uint32_t>& indices,
              simd_int2 framebuffer)
{
    std::vector<simd_float2> ndcPositions;
    ndcPositions.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        ndcPositions.push_back(simd_make_float2(vertex.position.x, vertex.position.y));
    }
    return ttm::buildPixelTrianglesFromNDC(ndcPositions, indices, framebuffer);
}

double computeCumulativeEdgeLength2D(const std::vector<Vertex>& vertices,
                                     const std::vector<uint32_t>& indices) {
    std::unordered_set<uint64_t> uniqueEdges;
    uniqueEdges.reserve(indices.size());

    auto makeEdgeKey = [](uint32_t vertexA, uint32_t vertexB) -> uint64_t {
        if (vertexA > vertexB) std::swap(vertexA, vertexB);
        return (uint64_t(vertexA) << 32) | uint64_t(vertexB);
    };

    const size_t triangleCount = indices.size() / 3;
    for (size_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
        const uint32_t index0 = indices[3 * triangleIndex + 0];
        const uint32_t index1 = indices[3 * triangleIndex + 1];
        const uint32_t index2 = indices[3 * triangleIndex + 2];
        
        uniqueEdges.insert(makeEdgeKey(index0, index1));
        uniqueEdges.insert(makeEdgeKey(index1, index2));
        uniqueEdges.insert(makeEdgeKey(index2, index0));
    }

    double totalLength = 0.0;
    for (uint64_t edgeKey : uniqueEdges) {
        const uint32_t indexA = static_cast<uint32_t>(edgeKey >> 32);
        const uint32_t indexB = static_cast<uint32_t>(edgeKey & 0xffffffffu);
        
        const auto& positionA = vertices[indexA].position;
        const auto& positionB = vertices[indexB].position;
        
        const double deltaX = static_cast<double>(positionB.x - positionA.x);
        const double deltaY = static_cast<double>(positionB.y - positionA.y);
        totalLength += std::sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    return totalLength;
}

void Print2DMeshMetrics(const std::vector<Vertex>& vertices,
                        const std::vector<uint32_t>& indices,
                        simd_int2 framebufferPx,
                        simd_int2 tileSizePx)
{
    const auto pixelTriangles = toTrianglesPx(vertices, indices, framebufferPx);
    const ttm::Metrics metrics = ttm::computeTileMetrics(pixelTriangles, framebufferPx, tileSizePx);
    const double cumulativeEdgeLength = computeCumulativeEdgeLength2D(vertices, indices);

    printf("[2D] TTO=%.0f  HTP_P95=%.1f  HTP_mean=%.1f  HTP_median=%.1f  "
           "SS_P95=%.1f  SS_mean=%.1f  SS_median=%.1f  BCI=%.3f  cumulatededgelength=%.6f\n",
           metrics.totalTilesTouched,
           metrics.trianglesPerTileP95, metrics.trianglesPerTileMean, metrics.trianglesPerTileMedian,
           metrics.tilesPerTriangleP95, metrics.tilesPerTriangleMean, metrics.tilesPerTriangleMedian,
           metrics.binningCostIndex,
           cumulativeEdgeLength);
}

} // namespace TriMetrics
