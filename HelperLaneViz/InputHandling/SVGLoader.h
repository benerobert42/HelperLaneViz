//
//  SVGLoader.hpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 11..
//

#pragma once

#include "ShaderTypes.h"

#include <functional>
#include <vector>

namespace SVGLoader {

struct AABB2 {
    simd_float2 min;
    simd_float2 max;
};

// Triangulator: takes polygon vertices, returns triangle indices
using Triangulator = std::function<std::vector<uint32_t>(std::vector<Vertex>&)>;

// Tessellate SVG with custom triangulator applied to each path
bool TessellateSvgToMesh(const std::string& filePath,
                         std::vector<Vertex>& outPositions,
                         std::vector<uint32_t>& outIndices,
                         Triangulator triangulator,
                         float bezierMaxDeviationPx = 20.0f);

// Convenience: tessellate with built-in ear-clipping
bool TessellateSvgToMesh(const std::string& filePath,
                         std::vector<Vertex>& outPositions,
                         std::vector<uint32_t>& outIndices,
                         float bezierMaxDeviationPx = 20.0f);
}
