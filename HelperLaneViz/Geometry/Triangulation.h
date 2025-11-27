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

#include <unordered_set>

namespace TriangleFactory {
    std::vector<uint32_t> CreateConvexMWT(const std::vector<Vertex>& vertices,
                                          double& cumulatedEdgeLength);
    std::vector<uint32_t> CreateCentralTriangulation(std::vector<Vertex>& vertices);

    std::vector<uint32_t> CreateMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                     double& cumulatedEdgeLength);
    std::vector<uint32_t> CreateStripTriangulation(const std::vector<Vertex>& vertices);

    std::vector<uint32_t> CreateConvexMinMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                              double& cumulatedEdgeLength);

    std::vector<uint32_t> CreateConvexMaxMinAreaTriangulation(const std::vector<Vertex>& vertices,
                                                              double& cumulatedEdgeLength);

    std::vector<uint32_t> TriangulatePolygon_CDT(const std::vector<Vertex>& vertices);
}
