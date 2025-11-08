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

struct EdgeMetrics {
    double unique_all = 0.0;
    double interior_sum = 0.0;
    double boundary_sum = 0.0;
    double nonmanifold_sum = 0.0;
    size_t interior_count = 0;
    size_t boundary_count = 0;
    size_t nonmanifold_count = 0;
    size_t unique_count = 0;
};

struct Triangulation {
    std::vector<uint32_t> indices;
    double totalCost = 0.0;
};

namespace TriangleFactory {
    std::vector<uint32_t> CreateConvexMWT(const std::vector<Vertex>& vertices,
                                          double& cumulatedEdgeLength);
    std::vector<uint32_t> CreateCentralTriangulation(std::vector<Vertex>& vertices);
    std::vector<uint32_t> CreateDelauneyTriangulation(std::vector<Vertex>& vertices);
    std::vector<uint32_t> CreateMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                     double& cumulatedEdgeLength);
    std::vector<uint32_t> CreateStripTriangulation(const std::vector<Vertex>& vertices);

    std::vector<uint32_t> CreateConvexMinMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                     double& cumulatedEdgeLength);

    std::vector<uint32_t> CreateConvexMaxMinAreaTriangulation(const std::vector<Vertex>& vertices,
                                                              double& cumulatedEdgeLength);

    std::vector<uint32_t>
    TriangulatePolygon_CDT(const std::vector<Vertex>& vertices);

    EdgeMetrics compute_edge_metrics(const std::vector<Vertex>& verts,
                                                    const std::vector<uint32_t>& triIndices);
}
