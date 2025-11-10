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

// Edge length measurements
struct EdgeKey {
    uint32_t a, b;
    EdgeKey(uint32_t i, uint32_t j) { if (i < j) { a=i; b=j; } else { a=j; b=i; } }
    bool operator==(const EdgeKey& o) const { return a==o.a && b==o.b; }
};
struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const noexcept {
        // 32-bit pair hash
        return (size_t)e.a * 1315423911u ^ (size_t)e.b;
    }
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

    std::unordered_set<EdgeKey, EdgeKeyHash> interior_edge_set(const std::vector<uint32_t>& triIndices);
    double interior_projected_length_px(const std::vector<Vertex>& verts,
                                        const std::vector<uint32_t>& triIndices,
                                        const simd_float4x4& VP,
                                        simd_int2 framebufferPx);
    double unique_length_3d(const std::vector<Vertex>& verts,
                            const std::vector<uint32_t>& triIndices);
    double interior_length_3d(const std::vector<Vertex>& verts,
                              const std::vector<uint32_t>& triIndices);
    EdgeMetrics compute_edge_metrics(const std::vector<Vertex>& verts,
                                     const std::vector<uint32_t>& triIndices);
}
