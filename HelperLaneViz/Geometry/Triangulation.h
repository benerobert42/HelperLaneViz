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

// MARK: - Convex Polygon Triangulation Methods

// Minimum Weight Triangulation using dynamic programming.
Result minimumWeightTriangulation(const std::vector<Vertex>& vertices);

// Fan triangulation from the centroid.
// Creates n triangles by connecting each edge to the polygon's center.
// Note: Modifies vertices by appending the centroid vertex.
Result centroidFanTriangulation(std::vector<Vertex>& vertices);

// Greedy triangulation selecting largest area triangles first.
Result greedyMaxAreaTriangulation(const std::vector<Vertex>& vertices);

// Strip triangulation alternating from both ends.
Result stripTriangulation(const std::vector<Vertex>& vertices);

// Maximizes the minimum triangle area (max-min optimization).
Result maxMinAreaTriangulation(const std::vector<Vertex>& vertices);

// Minimizes the maximum triangle area (min-max optimization).
Result minMaxAreaTriangulation(const std::vector<Vertex>& vertices);

// MARK: - General Polygon Triangulation

// Constrained Delaunay Triangulation for arbitrary simple polygons.
// Uses libigl's Triangle library wrapper.
std::vector<uint32_t> constrainedDelaunay(const std::vector<Vertex>& vertices);

} // namespace Triangulation
