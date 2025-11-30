//
//  SVGLoader.hpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 11..
//

#pragma once

#include "ShaderTypes.h"

#include <vector>

namespace SVGLoader {

struct AABB2 {
    simd_float2 min;
    simd_float2 max;
};

// Tessellate SVG and optionally triangulate with ear-clipping.
// If triangulate=false, outIndices will be empty and only vertices are returned.
bool TessellateSvgToMesh(const std::string& filePath,
                         std::vector<Vertex>& outPositions,
                         std::vector<uint32_t>& outIndices,
                         float bezierMaxDeviationPx = 20.0f,
                         bool triangulate = true);
}
