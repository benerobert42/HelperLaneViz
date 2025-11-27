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

// =============================================================================
// MARK: - Helper Functions
// =============================================================================

namespace {

/// Converts vertex positions to pixel-space triangles for tile overlap analysis.
std::vector<ttm::TrianglePx> buildPixelTriangles(const std::vector<Vertex>& vertices,
                                                  const std::vector<uint32_t>& indices,
                                                  simd_int2 framebufferSize) {
    std::vector<simd_float2> ndcPositions;
    ndcPositions.reserve(vertices.size());
    
    for (const auto& vertex : vertices) {
        ndcPositions.push_back(simd_make_float2(vertex.position.x, vertex.position.y));
    }
    
    return ttm::buildPixelTrianglesFromNDC(ndcPositions, indices, framebufferSize);
}

/// Creates a canonical edge key where the smaller index comes first.
/// This ensures edge (A,B) and edge (B,A) produce the same key.
uint64_t makeCanonicalEdgeKey(uint32_t indexA, uint32_t indexB) {
    if (indexA > indexB) {
        std::swap(indexA, indexB);
    }
    return (static_cast<uint64_t>(indexA) << 32) | static_cast<uint64_t>(indexB);
}

/// Computes edge-related metrics: unique edge count and total edge length.
void computeEdgeMetrics(const std::vector<Vertex>& vertices,
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
        
        uniqueEdges.insert(makeCanonicalEdgeKey(vertexIndex0, vertexIndex1));
        uniqueEdges.insert(makeCanonicalEdgeKey(vertexIndex1, vertexIndex2));
        uniqueEdges.insert(makeCanonicalEdgeKey(vertexIndex2, vertexIndex0));
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

// =============================================================================
// MARK: - Public API
// =============================================================================

MeshMetrics computeMeshMetrics(const std::vector<Vertex>& vertices,
                               const std::vector<uint32_t>& indices,
                               simd_int2 framebufferSize,
                               simd_int2 tileSize) {
    MeshMetrics result;
    
    if (vertices.empty() || indices.size() < 3) {
        return result;
    }
    
    // Compute edge metrics
    computeEdgeMetrics(vertices, indices, result.uniqueEdgeCount, result.totalEdgeLength);
    
    // Compute tile-based metrics
    const auto pixelTriangles = buildPixelTriangles(vertices, indices, framebufferSize);
    const ttm::Metrics tileMetrics = ttm::computeTileMetrics(pixelTriangles, framebufferSize, tileSize);
    
    // Transfer triangle metrics
    result.triangleCount = static_cast<size_t>(tileMetrics.triangleCount);
    result.totalTriangleArea = tileMetrics.sumAreaPx;
    
    // Transfer tile overlap metrics
    result.totalTileOverlaps = tileMetrics.totalTilesTouched;
    result.nonEmptyTileCount = static_cast<size_t>(tileMetrics.nonEmptyTileCount);
    
    // Transfer distribution metrics
    result.trianglesPerTile_Mean = tileMetrics.trianglesPerTileMean;
    result.trianglesPerTile_Median = tileMetrics.trianglesPerTileMedian;
    result.trianglesPerTile_P95 = tileMetrics.trianglesPerTileP95;
    
    result.tilesPerTriangle_Mean = tileMetrics.tilesPerTriangleMean;
    result.tilesPerTriangle_Median = tileMetrics.tilesPerTriangleMedian;
    result.tilesPerTriangle_P95 = tileMetrics.tilesPerTriangleP95;
    
    result.binningCostIndex = tileMetrics.binningCostIndex;
    
    return result;
}

void printMeshMetrics(const MeshMetrics& metrics,
                      simd_int2 framebufferSize,
                      simd_int2 tileSize) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════════\n");
    printf("                      MESH TRIANGULATION METRICS                    \n");
    printf("═══════════════════════════════════════════════════════════════════\n");
    
    printf("\n┌─ Configuration ─────────────────────────────────────────────────┐\n");
    printf("│  Framebuffer:          %d × %d pixels\n", framebufferSize.x, framebufferSize.y);
    printf("│  Tile Size:            %d × %d pixels\n", tileSize.x, tileSize.y);
    printf("└──────────────────────────────────────────────────────────────────┘\n");
    
    printf("\n┌─ Mesh Summary ──────────────────────────────────────────────────┐\n");
    printf("│  Triangle Count:       %zu\n", metrics.triangleCount);
    printf("│  Unique Edge Count:    %zu\n", metrics.uniqueEdgeCount);
    printf("│  Total Edge Length:    %.4f (NDC units)\n", metrics.totalEdgeLength);
    printf("│  Total Triangle Area:  %.2f (pixels²)\n", metrics.totalTriangleArea);
    printf("└──────────────────────────────────────────────────────────────────┘\n");
    
    printf("\n┌─ Tile Coverage ──────────────────────────────────────────────────┐\n");
    printf("│  Total Tile Overlaps:  %.0f\n", metrics.totalTileOverlaps);
    printf("│  Non-Empty Tiles:      %zu\n", metrics.nonEmptyTileCount);
    printf("│  Binning Cost Index:   %.3f  (1.0 = ideal, higher = worse)\n", metrics.binningCostIndex);
    printf("└──────────────────────────────────────────────────────────────────┘\n");
    
    printf("\n┌─ Triangles Per Tile (binning pressure) ─────────────────────────┐\n");
    printf("│  Mean:                 %.2f triangles/tile\n", metrics.trianglesPerTile_Mean);
    printf("│  Median:               %.2f triangles/tile\n", metrics.trianglesPerTile_Median);
    printf("│  95th Percentile:      %.2f triangles/tile\n", metrics.trianglesPerTile_P95);
    printf("└──────────────────────────────────────────────────────────────────┘\n");
    
    printf("\n┌─ Tiles Per Triangle (triangle spread) ──────────────────────────┐\n");
    printf("│  Mean:                 %.2f tiles/triangle\n", metrics.tilesPerTriangle_Mean);
    printf("│  Median:               %.2f tiles/triangle\n", metrics.tilesPerTriangle_Median);
    printf("│  95th Percentile:      %.2f tiles/triangle\n", metrics.tilesPerTriangle_P95);
    printf("└──────────────────────────────────────────────────────────────────┘\n");
    
    printf("\n");
}

void computeAndPrintMeshMetrics(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices,
                                simd_int2 framebufferSize,
                                simd_int2 tileSize) {
    const MeshMetrics metrics = computeMeshMetrics(vertices, indices, framebufferSize, tileSize);
    printMeshMetrics(metrics, framebufferSize, tileSize);
}

} // namespace TriangulationMetrics
