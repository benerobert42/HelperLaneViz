//
//  TriangulationMetrics.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#include "TriangulationMetrics.h"
#include "TileMetrics.h"

#include <cstdio>
#include <cmath>
#include <unordered_set>

namespace TriangulationMetrics {

namespace {

// MARK: NDC to pixel space helpers
std::vector<TileMetrics::TrianglePx> PixelTrianglesFromNDC(const std::vector<simd_float2>& ndcPositions,
                                              const std::vector<uint32_t>& indicesTri,
                                              simd_int2 fb) {
    auto ndcToPixel = [](simd_float2 ndc, simd_int2 fb) {
        return simd_int2((ndc * 0.5f + 0.5f) * fb);
    };

    std::vector<TileMetrics::TrianglePx> out;
    out.reserve(indicesTri.size() / 3);
    for (size_t i = 0; i + 2 < indicesTri.size(); i += 3) {
        const simd_int2 p0 = ndcToPixel(ndcPositions[indicesTri[i + 0]], fb);
        const simd_int2 p1 = ndcToPixel(ndcPositions[indicesTri[i + 1]], fb);
        const simd_int2 p2 = ndcToPixel(ndcPositions[indicesTri[i + 2]], fb);
        out.push_back({ p0, p1, p2 });
    }
    return out;
}

// Converts vertex positions to pixel-space triangles for tile overlap analysis.
std::vector<TileMetrics::TrianglePx> BuildPixelTriangles(const std::vector<Vertex>& vertices,
                                                         const std::vector<uint32_t>& indices,
                                                         simd_int2 framebufferSize) {
    std::vector<simd_float2> ndcPositions;
    ndcPositions.reserve(vertices.size());

    for (const auto& vertex : vertices) {
        ndcPositions.push_back(simd_make_float2(vertex.position.x, vertex.position.y));
    }

    return PixelTrianglesFromNDC(ndcPositions, indices, framebufferSize);
}

// Creates a canonical edge key where the smaller index comes first.
// This ensures edge (A,B) and edge (B,A) produce the same key.
uint64_t MakeCanonicalEdgeKey(uint32_t indexA, uint32_t indexB) {
    if (indexA > indexB) {
        std::swap(indexA, indexB);
    }
    return (static_cast<uint64_t>(indexA) << 32) | static_cast<uint64_t>(indexB);
}

// Computes edge-related metrics: unique edge count and total edge length.
void ComputeEdgeMetrics(const std::vector<Vertex>& vertices,
                        const std::vector<uint32_t>& indices,
                        size_t& outUniqueEdgeCount,
                        double& outTotalEdgeLength) {
    std::unordered_set<uint64_t> uniqueEdges;
    uniqueEdges.reserve(indices.size());
    
    const size_t triangleCount = indices.size() / 3;
    
    for (size_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
        const size_t baseIndex = triangleIndex * 3;
        const uint32_t vertexIndex0 = indices[baseIndex + 0];
        const uint32_t vertexIndex1 = indices[baseIndex + 1];
        const uint32_t vertexIndex2 = indices[baseIndex + 2];
        
        uniqueEdges.insert(MakeCanonicalEdgeKey(vertexIndex0, vertexIndex1));
        uniqueEdges.insert(MakeCanonicalEdgeKey(vertexIndex1, vertexIndex2));
        uniqueEdges.insert(MakeCanonicalEdgeKey(vertexIndex2, vertexIndex0));
    }
    
    double totalLength = 0.0;
    
    for (uint64_t edgeKey : uniqueEdges) {
        const uint32_t indexA = static_cast<uint32_t>(edgeKey >> 32);
        const uint32_t indexB = static_cast<uint32_t>(edgeKey & 0xFFFFFFFFu);
        
        const auto& positionA = vertices[indexA].position;
        const auto& positionB = vertices[indexB].position;
        
        const double deltaX = static_cast<double>(positionB.x) - static_cast<double>(positionA.x);
        const double deltaY = static_cast<double>(positionB.y) - static_cast<double>(positionA.y);
        
        totalLength += std::sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    outUniqueEdgeCount = uniqueEdges.size();
    outTotalEdgeLength = totalLength;
}

} // anonymous namespace


MeshMetrics ComputeMeshMetrics(const std::vector<Vertex>& verticesNDC,
                               const std::vector<uint32_t>& indices,
                               simd_int2 framebufferSizePx,
                               simd_int2 tileSizePx) {
    MeshMetrics result;
    
    if (verticesNDC.empty() || indices.size() < 3) {
        return result;
    }
    
    ComputeEdgeMetrics(verticesNDC, indices, result.uniqueEdgeCount, result.totalEdgeLength);

    // Compute tile-based metrics
    const auto pixelTriangles = BuildPixelTriangles(verticesNDC, indices, framebufferSizePx);
    const TileMetrics::Metrics tileMetrics = TileMetrics::ComputeTileMetrics(pixelTriangles, framebufferSizePx, tileSizePx);

    // Transfer metrics
    result.triangleCount = static_cast<size_t>(tileMetrics.triangleCount);
    result.totalTriangleArea = tileMetrics.sumAreaPx;
    
    result.totalTileOverlaps = tileMetrics.totalTilesTouched;
    result.nonEmptyTileCount = static_cast<size_t>(tileMetrics.nonEmptyTileCount);
    
    result.trianglesPerTile_Mean = tileMetrics.trianglesPerTileMean;
    result.trianglesPerTile_Median = tileMetrics.trianglesPerTileMedian;
    result.trianglesPerTile_P95 = tileMetrics.trianglesPerTileP95;
    
    result.tilesPerTriangle_Mean = tileMetrics.tilesPerTriangleMean;
    result.tilesPerTriangle_Median = tileMetrics.tilesPerTriangleMedian;
    result.tilesPerTriangle_P95 = tileMetrics.tilesPerTriangleP95;
    
    result.binningCostIndex = tileMetrics.binningCostIndex;
    
    // Note: overdrawSum and overdrawRatio should be computed via GPU
    // using Renderer::computeOverdrawMetricsWithOverdrawSum:overdrawRatio:
    // for accurate results. They are initialized to 0 here.
    
    return result;
}

} // namespace TriangulationMetrics
