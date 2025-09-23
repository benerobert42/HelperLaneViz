//
//  Triangulation.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#include "Triangulation.h"

#include <cmath>
#include <limits>
#include <cstdint>
#include <functional>

// Assumes vertices are the boundary of a CONVEX polygon in CCW or CW order.
// For a general simple polygon: add a diagonal validity test in the inner loop.
Triangulation TriangleFactory::minimumWeightTriangulationConvex(const std::vector<simd_float3>& v) {
    const int vertexCount = (int)v.size();
    Triangulation out;
    if (vertexCount < 3) {
        return out;
    }

    // Ensure CCW for consistent winding (optional)
    auto twiceArea = [&](){
        double A = 0;
        for (int i=0; i<vertexCount; i++) {
            int j = (i + 1) % vertexCount;
            A += v[i].x*v[j].y - v[j].x*v[i].y;
        }
        return A;
    };
    bool ccw = twiceArea() > 0;

    // dp and split tables
    std::vector<double> dp(vertexCount * vertexCount, 0.0);
    std::vector<int> split(vertexCount * vertexCount, -1);
    auto DP = [&](int i,int j)->double& { return dp[i * vertexCount + j]; };
    auto SP = [&](int i,int j)->int&    { return split[i * vertexCount + j]; };

    auto weight = [&](int a, int b){ return simd_distance(v[a], v[b]); };

    // Initialize
    for (int i=0; i < vertexCount - 1; i++) {
        DP(i, i + 1) = 0.0;
    }

    for (int len = 2; len <= vertexCount - 1; ++len) {
        for (int i=0; i + len < vertexCount; ++i) {
            int j = i + len;
            double best = std::numeric_limits<double>::infinity();
            int bestk = -1;
            for (int k = i + 1; k < j; ++k) {
                // For non-convex polygons insert: if (!validTriangle(i,k,j)) continue;
                double c = DP(i, k) + DP(k, j) + weight(i, k) + weight(k, j);
                if (c < best) { best = c; bestk = k; }
            }
            DP(i, j) = best;
            SP(i, j) = bestk;
        }
    }

    out.totalCost = DP(0, vertexCount - 1);

    // Reconstruct triangles with consistent winding
    std::vector<uint32_t> idx;
    idx.reserve(3 * (vertexCount - 2));
    std::function<void(int,int)> emit = [&](int i,int j){
        int k = SP(i, j);
        if (k < 0) return;
        // Triangle (i,k,j)
        auto pushTri = [&](int a,int b,int c){
            if (ccw) { idx.push_back(a); idx.push_back(b); idx.push_back(c); }
            else     { idx.push_back(a); idx.push_back(c); idx.push_back(b); }
        };
        pushTri(i, k, j);
        emit(i, k);
        emit(k, j);
    };
    emit(0, vertexCount - 1);
    out.indices = std::move(idx);
    return out;
}
