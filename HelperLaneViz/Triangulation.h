//
//  Triangulation.hpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#pragma once

#include "ShaderTypes.h"

#include <simd/simd.h>
#include <vector>

struct Triangulation {
    std::vector<uint32_t> indices;
    double totalCost = 0.0;
};

namespace TriangleFactory {
    std::vector<uint32_t> CreateConvexMWT(const std::vector<Vertex>& vertices);
    std::vector<uint32_t> CreateCentralTriangulation(std::vector<Vertex>& vertices);
    std::vector<uint32_t> CreateDelauneyTriangulation(std::vector<Vertex>& vertices);
    std::vector<uint32_t> CreateMaxAreaTriangulation(const std::vector<Vertex>& vertices);
}
