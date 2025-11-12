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

namespace TriMetrics {
void printEdgeAndTileMetrics(const std::vector<Vertex>& V_mwt,
                             const std::vector<uint32_t>& I_mwt,
                             const std::vector<Vertex>& V_del,
                             const std::vector<uint32_t>& I_del,
                             simd_float4x4 viewProjection,
                             simd_int2 framebufferPx,
                             simd_int2 tileSizePx);

void Print2DMeshMetrics(const std::vector<Vertex>& V,
                        const std::vector<uint32_t>& I,
                        simd_int2 framebufferPx,
                        simd_int2 tileSizePx);
}
