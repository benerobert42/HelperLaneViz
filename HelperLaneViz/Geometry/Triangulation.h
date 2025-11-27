//
//  Triangulation.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#pragma once

#include "ShaderTypes.h"

#include <simd/simd.h>
#include <vector>
#include <cstdint>

namespace Triangulation {

/// Result structure containing triangle indices and optional metrics
struct Result {
    std::vector<uint32_t> indices;
    double totalEdgeLength = 0.0;
};

// =============================================================================
// MARK: - Convex Polygon Triangulation Methods
// =============================================================================

/// Minimum Weight Triangulation using dynamic programming.
/// Minimizes total internal edge length for convex polygons.
/// Time complexity: O(n³)
Result minimumWeightTriangulation(const std::vector<Vertex>& vertices);

/// Fan triangulation from the centroid.
/// Creates n triangles by connecting each edge to the polygon's center.
/// Note: Modifies vertices by appending the centroid vertex.
Result centroidFanTriangulation(std::vector<Vertex>& vertices);

/// Greedy triangulation selecting largest area triangles first.
/// Recursively subdivides polygon by choosing max-area triangles.
Result greedyMaxAreaTriangulation(const std::vector<Vertex>& vertices);

/// Strip triangulation alternating from both ends.
/// Creates a triangle strip pattern (efficient for rendering).
Result stripTriangulation(const std::vector<Vertex>& vertices);

/// Maximizes the minimum triangle area (max-min optimization).
/// Produces balanced triangles avoiding very small areas.
Result maxMinAreaTriangulation(const std::vector<Vertex>& vertices);

/// Minimizes the maximum triangle area (min-max optimization).
/// Produces balanced triangles avoiding very large areas.
Result minMaxAreaTriangulation(const std::vector<Vertex>& vertices);

// =============================================================================
// MARK: - General Polygon Triangulation
// =============================================================================

/// Constrained Delaunay Triangulation for arbitrary simple polygons.
/// Uses libigl's Triangle library wrapper.
std::vector<uint32_t> constrainedDelaunay(const std::vector<Vertex>& vertices);

} // namespace Triangulation
