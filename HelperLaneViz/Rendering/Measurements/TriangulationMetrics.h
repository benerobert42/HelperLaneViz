//
//  TriangulationMetrics.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#pragma once

#include <vector>
#include <simd/simd.h>
#include "ShaderTypes.h"

namespace TriangulationMetrics {

/// Summary of tile-based rendering metrics for a triangulated mesh.
struct MeshMetrics {
    // Edge metrics
    size_t uniqueEdgeCount = 0;
    double totalEdgeLength = 0.0;
    
    // Triangle metrics
    size_t triangleCount = 0;
    double totalTriangleArea = 0.0;
    
    // Tile overlap metrics (GPU binning efficiency)
    double totalTileOverlaps = 0.0;           ///< Sum of tiles touched by all triangles
    size_t nonEmptyTileCount = 0;             ///< Number of tiles containing at least one triangle
    
    // Triangles per tile distribution (measures overdraw/binning pressure)
    double trianglesPerTile_Mean = 0.0;
    double trianglesPerTile_Median = 0.0;
    double trianglesPerTile_P95 = 0.0;
    
    // Tiles per triangle distribution (measures triangle spread)
    double tilesPerTriangle_Mean = 0.0;
    double tilesPerTriangle_Median = 0.0;
    double tilesPerTriangle_P95 = 0.0;
    
    // Normalized binning cost: totalTileOverlaps / idealTileCoverage
    // Values close to 1.0 indicate efficient triangles; higher values indicate many small/thin triangles
    double binningCostIndex = 0.0;
};

/// Computes comprehensive metrics for a 2D triangulated mesh.
/// @param vertices      Mesh vertices with positions in NDC space [-1, 1]
/// @param indices       Triangle indices (3 per triangle)
/// @param framebufferSize   Framebuffer dimensions in pixels
/// @param tileSize      Tile dimensions in pixels (typically 32x32 for GPU tile-based renderers)
/// @return Computed metrics structure
MeshMetrics computeMeshMetrics(const std::vector<Vertex>& vertices,
                               const std::vector<uint32_t>& indices,
                               simd_int2 framebufferSize,
                               simd_int2 tileSize);

/// Prints mesh metrics to stdout in a readable format.
void printMeshMetrics(const MeshMetrics& metrics,
                      simd_int2 framebufferSize,
                      simd_int2 tileSize);

/// Convenience function: computes and prints metrics in one call.
void computeAndPrintMeshMetrics(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices,
                                simd_int2 framebufferSize,
                                simd_int2 tileSize);

} // namespace TriangulationMetrics
