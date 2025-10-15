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
#include <map>
#include <numeric>
#include <random>

// Assumes vertices are the boundary of a convex polygon
std::vector<uint32_t> TriangleFactory::CreateConvexMWT(const std::vector<Vertex>& vertices) {
    const int vertexCount = int(vertices.size());
    std::vector<uint32_t> out;
    float totalCost = 0;

    if (vertexCount < 3) {
        return out;
    }

    std::vector<double> dp(vertexCount * vertexCount, 0.0);
    std::vector<int> split(vertexCount * vertexCount, -1);
    auto DP = [&](int i,int j)->double& { return dp[i * vertexCount + j]; };
    auto SP = [&](int i,int j)->int&    { return split[i * vertexCount + j]; };

    auto weight = [&](int a, int b){ return simd_distance(vertices[a].position, vertices[b].position); };

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

    totalCost = DP(0, vertexCount - 1);

    // Reconstruct triangles with consistent winding
    std::vector<uint32_t> idx;
    idx.reserve(3 * (vertexCount - 2));
    std::function<void(int,int)> emit = [&](int i,int j){
        int k = SP(i, j);
        if (k < 0) return;
        // Triangle (i,k,j)
        auto pushTri = [&](int a,int b,int c){
            idx.push_back(a); idx.push_back(b); idx.push_back(c);
        };
        pushTri(i, k, j);
        emit(i, k);
        emit(k, j);
    };
    emit(0, vertexCount - 1);
    out = std::move(idx);
    return out;
}

std::vector<uint32_t> TriangleFactory::CreateCentralTriangulation(std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const size_t vertexCount = vertices.size();
    if (vertexCount < 3) {
        return indices;
    }

    // Center calculation by simple averaging
    simd_float2 center{0, 0};
    for (auto& vertex : vertices) {
        center += {vertex.position.x, vertex.position.y};
    }
    center /= vertexCount;

    Vertex vertex{.position = {center.x, center.y, 1}};
    vertices.push_back(vertex);
    const uint32_t c = (uint32_t)vertices.size() - 1;

    indices.reserve(vertexCount * 3);
    for (uint32_t i = 0; i < vertexCount; ++i) {
        uint32_t j = (i + 1) % (uint32_t)vertexCount;
        // For ccw input (i, j, c) keeps orientation
        indices.push_back(i);
        indices.push_back(j);
        indices.push_back(c);
    }
    return indices;
}

std::vector<uint32_t> TriangleFactory::CreateDelauneyTriangulation(std::vector<Vertex>& vertices) {
    auto circumcircleContainsStrict = [&](const simd_float2& a,
                                          const simd_float2& b,
                                          const simd_float2& c,
                                          const simd_float2& p) -> bool {
        float A = b.x - a.x;
        float B = b.y - a.y;
        float C = c.x - a.x;
        float D = c.y - a.y;
        float E = A*(a.x+b.x) + B*(a.y+b.y);
        float F = C*(a.x+c.x) + D*(a.y+c.y);
        float G = 2.f*(A*(c.y-b.y) - B*(c.x-b.x));

        if (std::fabs(G) < 1e-12f) {
            return false; // collinear
        }

        simd_float2 center{(D * E - B * F) / G, (A * F - C * E) / G};
        return simd_distance_squared(center, p) < simd_distance_squared(center, a); // equality not allowed
    };

    struct Triangle {
        uint32_t a;
        uint32_t b;
        uint32_t c;
    };

    struct Edge {
        uint32_t u;
        uint32_t v;

        Edge(uint32_t a, uint32_t b) {
            if (a < b) {
                u = a;
                v = b;
            } else {
                u = b;
                v = a;
            }
        } // canonicalize (min,max)

        bool operator<(const Edge& other) const {
            return (u < other.u) || (u == other.u && v < other.v);
        }
    };

    const uint32_t vertexCount = static_cast<uint32_t>(vertices.size());
    std::vector<uint32_t> out;
    if (vertexCount < 3) {
        return out;
    }

    std::vector<simd_float2> points;
    points.reserve(vertexCount+3);
    for (const auto& vertex: vertices) {
        points.push_back(simd_make_float2(vertex.position.x, vertex.position.y));
    }

    // (optional) randomized insertion order for robustness on circles
    std::vector<uint32_t> order(vertexCount);
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 rng{12345}; std::shuffle(order.begin(), order.end(), rng);

    float minx =+ INFINITY;
    float miny =+ INFINITY;
    float maxx =- INFINITY;
    float maxy =- INFINITY;

    for (const auto& point: points) {
        minx = std::min(minx,point.x);
        miny = std::min(miny,point.y);
        maxx = std::max(maxx,point.x);
        maxy = std::max(maxy,point.y);
    }
    float d = std::max(maxx - minx, maxy - miny) * 10.f + 1.f;
    simd_float2 s0{minx - 1.f, miny - 1.f - d};
    simd_float2 s1{minx - 1.f - d, maxy + 1.f};
    simd_float2 s2{maxx + 1.f + d, maxy + 1.f};
    uint32_t i0 = (uint32_t)points.size();
    points.push_back(s0);
    uint32_t i1 = (uint32_t)points.size();
    points.push_back(s1);
    uint32_t i2 = (uint32_t)points.size();
    points.push_back(s2);

    std::vector<Triangle> triangles;
    triangles.push_back({i0,i1,i2});

    auto ccw = [&](uint32_t a, uint32_t b, uint32_t c){
        simd_float2 p=points[a];
        simd_float2 q = points[b];
        simd_float2 r = points[c];
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x) >= 0.f;
    };

    for (uint32_t k = 0; k < vertexCount; ++k) {
        uint32_t pointIdx = order[k];
        const simd_float2& point = points[pointIdx];

        std::vector<int> bad;
        std::map<Edge,int> border;

        for (int triangleIdx = 0; triangleIdx < int(triangles.size()); ++triangleIdx) {
            const auto& triangle = triangles[triangleIdx];
            if (circumcircleContainsStrict(points[triangle.a],
                                           points[triangle.b],
                                           points[triangle.c],
                                           point)) {
                bad.push_back(triangleIdx);
                border[Edge(triangle.a, triangle.b)]++;
                border[Edge(triangle.b, triangle.c)]++;
                border[Edge(triangle.c, triangle.a)]++;
            }
        }

        if (!bad.empty()) {
            std::vector<Triangle> kept;
            kept.reserve(triangles.size() - bad.size());
            int bi = 0;
            for (int triangleIdx = 0; triangleIdx < int(triangles.size()); ++triangleIdx) {
                if (bi < (int)bad.size() && triangleIdx == bad[bi]) { ++bi; continue; }
                kept.push_back(triangles[triangleIdx]);
            }
            triangles.swap(kept);
        }

        for (auto &[edge, occurrence] : border) {
            if (occurrence == 1) {
                uint32_t a = edge.u;
                uint32_t b = edge.v;
                // enforce consistent winding (CCW) with the new point
                if (!ccw(a, b, pointIdx)) {
                    std::swap(a,b);
                }
                triangles.push_back({a, b, pointIdx});
            }
        }

        out.reserve(triangles.size()*3);
        for (auto& triangle : triangles) {
            if (triangle.a >= vertexCount || triangle.b >= vertexCount || triangle.c >= vertexCount) { continue;  // drop super-triangle
            }
            // final CCW if needed by your pipeline
            if (!ccw(triangle.a, triangle.b, triangle.c)) {
                std::swap(triangle.b, triangle.c);
            }
            out.push_back(triangle.a);
            out.push_back(triangle.b);
            out.push_back(triangle.c);
        }
    }
    return out;
}

std::vector<uint32_t> TriangleFactory::CreateMaxAreaTriangulation(const std::vector<Vertex>& vertices)
{
    std::vector<uint32_t> triangleIndices;
    const size_t n = vertices.size();
    if (n < 3) return triangleIndices;

    // Helper: triangle area (absolute), using Vertex.x / Vertex.y
    auto triArea = [&](uint32_t ia, uint32_t ib, uint32_t ic) -> double {
        const auto& A = vertices[ia]; const auto& B = vertices[ib]; const auto& C = vertices[ic];
        const double ax = double(A.position.x), ay = double(A.position.y);
        const double bx = double(B.position.x), by = double(B.position.y);
        const double cx = double(C.position.x), cy = double(C.position.y);
        return std::abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) * 0.5;
    };

    // Recursive greedy splitter: for the current convex polygon (as indices into 'vertices'),
    // find the globally largest-area triangle (any triple), emit it, split along its edges,
    // and recurse on the resulting convex sub-polygons.
    std::function<void(const std::vector<uint32_t>&)> solve =
    [&](const std::vector<uint32_t>& poly)
    {
        const size_t m = poly.size();
        if (m < 3) return;
        if (m == 3) {
            triangleIndices.insert(triangleIndices.end(), { poly[0], poly[1], poly[2] });
            return;
        }

        // 1) Find the largest-area triangle among all triples in this polygon (i < j < k in ring order).
        double bestArea = -1.0;
        size_t bi = 0, bj = 1, bk = 2;
        for (size_t i = 0; i + 2 < m; ++i) {
            for (size_t j = i + 1; j + 1 < m; ++j) {
                for (size_t k = j + 1; k < m; ++k) {
                    const double area = triArea(poly[i], poly[j], poly[k]);
                    if (area > bestArea) { bestArea = area; bi = i; bj = j; bk = k; }
                }
            }
        }

        // Emit the chosen triangle (A,B,C).
        const uint32_t A = poly[bi], B = poly[bj], C = poly[bk];
        triangleIndices.insert(triangleIndices.end(), { A, B, C });

        // 2) Split the polygon along the edges (A,B), (B,C), (C,A) into up to three sub-polygons.
        // Each sub-polygon is the boundary arc between those vertices, inclusive of endpoints.
        auto makeArc = [&](size_t start, size_t endInclusive) -> std::vector<uint32_t> {
            std::vector<uint32_t> sub;
            if (start <= endInclusive) {
                sub.insert(sub.end(), poly.begin() + ptrdiff_t(start), poly.begin() + ptrdiff_t(endInclusive + 1));
            } else {
                sub.insert(sub.end(), poly.begin() + ptrdiff_t(start), poly.end());
                sub.insert(sub.end(), poly.begin(),               poly.begin() + ptrdiff_t(endInclusive + 1));
            }
            return sub;
        };

        // Indices bi,bj,bk are in increasing order on the ring by construction.
        const auto subAB = makeArc(bi, bj);
        const auto subBC = makeArc(bj, bk);
        const auto subCA = makeArc(bk, bi);

        // 3) Recurse on any sub-polygon that still has 3+ vertices (a triangle or larger).
        if (subAB.size() >= 3) solve(subAB);
        if (subBC.size() >= 3) solve(subBC);
        if (subCA.size() >= 3) solve(subCA);
    };

    // Start with the full polygon as indices [0..n-1] in circular order.
    std::vector<uint32_t> full(n);
    for (uint32_t i = 0; i < n; ++i) full[i] = i;

    triangleIndices.reserve((n - 2) * 3);
    solve(full);
    return triangleIndices;
}
