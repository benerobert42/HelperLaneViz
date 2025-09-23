//
//  Triangulation.hpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#pragma once

#include <simd/simd.h>
#include <vector>

struct Triangulation {
    std::vector<uint32_t> indices;
    double totalCost = 0.0;
};

namespace TriangleFactory {
    Triangulation minimumWeightTriangulationConvex(const std::vector<simd_float3>& v);
}
