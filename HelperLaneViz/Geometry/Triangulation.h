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

struct Result {
    std::vector<uint32_t> indices;
    double totalEdgeLength = 0.0;
};

// MARK: - Polygon Triangulation Methods

// Simple ear clipping triangulation - O(n²), works for any simple polygon.
Result earClippingTriangulation(const std::vector<Vertex>& vertices);

// Minimum Weight Triangulation using dynamic programming.
Result minimumWeightTriangulation(const std::vector<Vertex>& vertices);

// Fan triangulation from the centroid.
// Note: Modifies vertices by appending the centroid vertex.
Result centroidFanTriangulation(std::vector<Vertex>& vertices);

// Greedy triangulation selecting largest area triangles first (ear-clipping based).
Result greedyMaxAreaTriangulation(const std::vector<Vertex>& vertices);

// Strip triangulation alternating from both ends.
Result stripTriangulation(const std::vector<Vertex>& vertices);

// Maximizes the minimum triangle area (max-min optimization).
Result maxMinAreaTriangulation(const std::vector<Vertex>& vertices);

// Minimizes the maximum triangle area (min-max optimization).
Result minMaxAreaTriangulation(const std::vector<Vertex>& vertices);

// Constrained Delaunay Triangulation - handles any simple polygon including concave.
// Uses libigl's Triangle library wrapper.
std::vector<uint32_t> constrainedDelaunay(const std::vector<Vertex>& vertices);

// CDT with holes support - outer boundary + list of hole boundaries
// Each hole is a vector of vertices forming a closed loop (CW orientation for holes)
// Returns indices into the combined vertex array (outer first, then holes in order)
std::vector<uint32_t> constrainedDelaunayWithHoles(
    const std::vector<Vertex>& outerBoundary,
    const std::vector<std::vector<Vertex>>& holes);

} // namespace Triangulation
