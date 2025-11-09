//
//  TriangulationMetrics.hpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#pragma once

#include <vector>
#include <simd/simd.h>

#include "ShaderTypes.h"

namespace ttm { struct TrianglePx; struct Metrics; }

namespace TriMetrics {
void print_edge_and_tile_metrics(const std::vector<Vertex>& V_mwt,
                                 const std::vector<uint32_t>& I_mwt,
                                 const std::vector<Vertex>& V_del,
                                 const std::vector<uint32_t>& I_del,
                                 simd_float4x4 VP,
                                 simd_int2 framebufferPx,
                                 simd_int2 tileSz);
}
