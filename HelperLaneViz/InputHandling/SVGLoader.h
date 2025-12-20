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

// A shape with optional holes
struct ShapeWithHoles {
    std::vector<Vertex> outerBoundary; // CCW outer boundary
    std::vector<std::vector<Vertex>> holes; // CW hole boundaries
};

// Triangulator: takes polygon vertices, returns triangle indices
using Triangulator = std::function<std::vector<uint32_t>(const std::vector<Vertex>&, bool)>;

// Tessellate SVG with custom triangulator
bool TessellateSvgToMesh(const std::string& filePath,
                         std::vector<Vertex>& outPositions,
                         std::vector<uint32_t>& outIndices,
                         Triangulator triangulator,
                         float bezierMaxDeviationPx = 20.0f);

// Parse SVG into shapes with holes
std::vector<ShapeWithHoles> ParseSvgToShapes(const std::string& filePath,
                                             float bezierMaxDeviationPx = 20.0f);
}
