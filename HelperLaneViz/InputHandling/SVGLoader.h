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
    std::vector<Vertex> outerBoundary;          // CCW outer boundary
    std::vector<std::vector<Vertex>> holes;     // CW hole boundaries
};

// Triangulator: takes polygon vertices, returns triangle indices
// Parameters: vertices (bridged polygon), shouldHandleConcave, handleHoles, outerVertices, holes
using Triangulator = std::function<std::vector<uint32_t>(const std::vector<Vertex>&, bool, bool, const std::vector<Vertex>&, const std::vector<std::vector<Vertex>>&)>;

// Triangulator with holes: takes outer boundary + holes, returns indices
using TriangulatorWithHoles = std::function<std::vector<uint32_t>(
    const std::vector<Vertex>&,
    const std::vector<std::vector<Vertex>>&)>;

// Tessellate SVG with custom triangulator
// For shapes with holes, uses CDT. For simple shapes, uses the provided triangulator.
bool TessellateSvgToMesh(const std::string& filePath,
                         std::vector<Vertex>& outPositions,
                         std::vector<uint32_t>& outIndices,
                         Triangulator triangulator,
                         float bezierMaxDeviationPx = 20.0f);

// Convenience: tessellate with built-in ear-clipping (simple shapes) / CDT (shapes with holes)
bool TessellateSvgToMesh(const std::string& filePath,
                         std::vector<Vertex>& outPositions,
                         std::vector<uint32_t>& outIndices,
                         float bezierMaxDeviationPx = 20.0f);

// Parse SVG into shapes with holes (for advanced usage)
std::vector<ShapeWithHoles> ParseSvgToShapes(const std::string& filePath,
                                              float bezierMaxDeviationPx = 20.0f);
}
