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

double calculateTotalEdgeLength(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices);

std::vector<Vertex> buildPolygonWithHoles(const std::vector<Vertex>& outer,
                                          const std::vector<std::vector<Vertex>>& holes);

// MARK: - Polygon Triangulation Methods

// Simple algorithms without specific optimisation target
std::vector<uint32_t> EarClippingTriangulation(const std::vector<Vertex>& vertices);
std::vector<uint32_t> CentroidFanTriangulation(std::vector<Vertex>& vertices);
std::vector<uint32_t> StripTriangulation(const std::vector<Vertex>& vertices);

std::vector<uint32_t> GreedyMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                 bool shouldHandleConcave = false,
                                                 bool handleHoles = false,
                                                 const std::vector<Vertex>& outerVertices = {},
                                                 const std::vector<std::vector<Vertex>>& holes = {});

std::vector<uint32_t> MinimumWeightTriangulation(const std::vector<Vertex>& vertices,
                                                 bool shouldHandleConcave = false,
                                                 bool handleHoles = false,
                                                 const std::vector<Vertex>& outerVertices = {},
                                                 const std::vector<std::vector<Vertex>>& holes = {});

// Maximizes the minimum triangle area (max-min optimization).
std::vector<uint32_t> MaxMinAreaTriangulation(const std::vector<Vertex>& vertices,
                                              bool shouldHandleConcave = false,
                                              bool handleHoles = false,
                                              const std::vector<Vertex>& outerVertices = {},
                                              const std::vector<std::vector<Vertex>>& holes = {});

// Minimizes the maximum triangle area (min-max optimization).
std::vector<uint32_t> MinMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                              bool shouldHandleConcave = false,
                                              bool handleHoles = false,
                                              const std::vector<Vertex>& outerVertices = {},
                                              const std::vector<std::vector<Vertex>>& holes = {});

// libigl's Triangle library wrapper based Constrained Delaunay Triangulation.
std::vector<uint32_t> ConstrainedDelaunayTriangulation(const std::vector<Vertex>& vertices);

} // namespace Triangulation
