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

// Calculate total edge length of a triangulation (sum of all triangle perimeters)
double calculateTotalEdgeLength(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices);

// MARK: - Polygon Triangulation Methods

// Simple ear clipping triangulation - O(n²), works for any simple polygon.
std::vector<uint32_t> earClippingTriangulation(const std::vector<Vertex>& vertices);

// Minimum Weight Triangulation using dynamic programming.
std::vector<uint32_t> minimumWeightTriangulation(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);

// Fan triangulation from the centroid.
// Note: Modifies vertices by appending the centroid vertex.
std::vector<uint32_t> centroidFanTriangulation(std::vector<Vertex>& vertices);

// Greedy triangulation selecting largest area triangles first (ear-clipping based).
std::vector<uint32_t> greedyMaxAreaTriangulation(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);

// Strip triangulation alternating from both ends.
std::vector<uint32_t> stripTriangulation(const std::vector<Vertex>& vertices);

// Maximizes the minimum triangle area (max-min optimization).
std::vector<uint32_t> maxMinAreaTriangulation(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);

// Minimizes the maximum triangle area (min-max optimization).
std::vector<uint32_t> minMaxAreaTriangulation(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);

// Constrained Delaunay Triangulation - handles any simple polygon including concave.
// Uses libigl's Triangle library wrapper.
std::vector<uint32_t> constrainedDelaunay(const std::vector<Vertex>& vertices);

} // namespace Triangulation
