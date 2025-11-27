//
//  TriangulationMetrics.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#pragma once

#include <vector>
#include <unordered_set>
#include <cmath>
#include <simd/simd.h>
#include "ShaderTypes.h"

namespace TriMetrics {

/// Computes the sum of all unique edge lengths in a 2D triangulated mesh.
double computeCumulativeEdgeLength2D(const std::vector<Vertex>& vertices,
                                     const std::vector<uint32_t>& indices);

/// Prints tile-based rendering metrics for a 2D mesh.
void Print2DMeshMetrics(const std::vector<Vertex>& vertices,
                        const std::vector<uint32_t>& indices,
                        simd_int2 framebufferPx,
                        simd_int2 tileSizePx);

} // namespace TriMetrics
