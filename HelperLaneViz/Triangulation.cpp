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

#include <Eigen/Core>
#include <igl/triangle/triangulate.h>

// Assumes vertices are the boundary of a convex polygon
std::vector<uint32_t> TriangleFactory::CreateConvexMWT(const std::vector<Vertex>& vertices,
                                                       double& cumulatedEdgeLength) {
    cumulatedEdgeLength = 0.0;

    const int vertexCount = int(vertices.size());
    std::vector<uint32_t> out;
    float totalCost = 0;

    if (vertexCount < 3) {
        return out;
    }

    auto triPerimeter = [&](uint32_t ia, uint32_t ib, uint32_t ic) -> double {
        auto L = [&](uint32_t p, uint32_t q) {
            const auto& P = vertices[p]; const auto& Q = vertices[q];
            const double dx = double(Q.position.x) - double(P.position.x);
            const double dy = double(Q.position.y) - double(P.position.y);
            return std::sqrt(dx*dx + dy*dy);
        };
        return L(ia, ib) + L(ib, ic) + L(ic, ia);
    };

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
            cumulatedEdgeLength += triPerimeter(static_cast<uint32_t>(a),
                                                static_cast<uint32_t>(b),
                                                static_cast<uint32_t>(c));
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

std::vector<uint32_t>
TriangleFactory::CreateDelauneyTriangulation(std::vector<Vertex>& vertices)
{
    auto circumcircleContainsStrict = [&](const simd_float2& a,
                                          const simd_float2& b,
                                          const simd_float2& c,
                                          const simd_float2& p) -> bool
    {
        float A = b.x - a.x;
        float B = b.y - a.y;
        float C = c.x - a.x;
        float D = c.y - a.y;
        float E = A*(a.x+b.x) + B*(a.y+b.y);
        float F = C*(a.x+c.x) + D*(a.y+c.y);
        float G = 2.f*(A*(c.y-b.y) - B*(c.x-b.x));

        if (std::fabs(G) < 1e-12f) return false; // collinear

        simd_float2 center{ (D*E - B*F) / G, (A*F - C*E) / G };
        float r2 = simd_distance_squared(center, a);
        float d2 = simd_distance_squared(center, p);
        return d2 < r2; // strict
    };

    struct Triangle { uint32_t a,b,c; };

    struct Edge {
        uint32_t u,v;
        Edge(uint32_t a, uint32_t b) { // canonical (min,max)
            if (a < b) { u=a; v=b; } else { u=b; v=a; }
        }
        bool operator<(const Edge& o) const {
            return (u < o.u) || (u == o.u && v < o.v);
        }
    };

    const uint32_t vertexCount = static_cast<uint32_t>(vertices.size());
    std::vector<uint32_t> out;
    if (vertexCount < 3) return out;

    // Gather 2D points
    std::vector<simd_float2> points;
    points.reserve(vertexCount + 3);
    for (const auto& v : vertices)
        points.push_back(simd_make_float2(v.position.x, v.position.y));

    // Randomized insertion order (robustness)
    std::vector<uint32_t> order(vertexCount);
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 rng{12345};
    std::shuffle(order.begin(), order.end(), rng);

    // Bounding box
    float minx = +INFINITY, miny = +INFINITY;
    float maxx = -INFINITY, maxy = -INFINITY;
    for (const auto& p : points) {
        minx = std::min(minx, p.x); miny = std::min(miny, p.y);
        maxx = std::max(maxx, p.x); maxy = std::max(maxy, p.y);
    }

    // FIX: build a robust super-triangle that surely encloses all points
    // Use a big triangle centered at bbox center.
    const float cx = 0.5f*(minx + maxx), cy = 0.5f*(miny + maxy);
    const float r  = std::max(maxx - minx, maxy - miny) * 10.f + 1.f; // radius margin

    simd_float2 s0{ cx - 2.f*r, cy - r };
    simd_float2 s1{ cx,         cy + 2.f*r };
    simd_float2 s2{ cx + 2.f*r, cy - r };

    uint32_t i0 = (uint32_t)points.size(); points.push_back(s0);
    uint32_t i1 = (uint32_t)points.size(); points.push_back(s1);
    uint32_t i2 = (uint32_t)points.size(); points.push_back(s2);

    std::vector<Triangle> triangles;
    triangles.push_back({i0,i1,i2});

    auto ccw = [&](uint32_t a, uint32_t b, uint32_t c){
        const simd_float2 &p = points[a], &q = points[b], &r2 = points[c];
        return (q.x - p.x)*(r2.y - p.y) - (q.y - p.y)*(r2.x - p.x) > 0.f;
    };

    // Incremental Bowyer–Watson
    for (uint32_t k = 0; k < vertexCount; ++k) {
        uint32_t pi = order[k];
        const simd_float2& P = points[pi];

        std::vector<int> bad;
        bad.reserve(triangles.size());
        std::map<Edge,int> border;

        // Find all triangles whose circumcircle strictly contains P
        for (int t = 0; t < (int)triangles.size(); ++t) {
            const auto& T = triangles[t];
            if (circumcircleContainsStrict(points[T.a], points[T.b], points[T.c], P)) {
                bad.push_back(t);
                // Collect edges of the cavity boundary (count occurrences)
                border[Edge(T.a,T.b)]++;
                border[Edge(T.b,T.c)]++;
                border[Edge(T.c,T.a)]++;
            }
        }

        // FIX: remove "bad" triangles (sort first, then skip-copy)
        std::sort(bad.begin(), bad.end()); // FIX
        if (!bad.empty()) {
            std::vector<Triangle> kept;
            kept.reserve(triangles.size() - bad.size());
            int bi = 0;
            for (int t = 0; t < (int)triangles.size(); ++t) {
                if (bi < (int)bad.size() && t == bad[bi]) { ++bi; continue; }
                kept.push_back(triangles[t]);
            }
            triangles.swap(kept);
        }

        // Re-triangulate the cavity boundary with fan to P (only edges seen once)
        for (const auto& kv : border) {
            if (kv.second != 1) continue;  // interior edges appear twice
            uint32_t a = kv.first.u, b = kv.first.v;
            // enforce CCW with the new point
            if (!ccw(a,b,pi)) std::swap(a,b);
            triangles.push_back({a,b,pi});
        }

        // FIX: DO NOT append to 'out' inside the loop.
        // Output is generated once after all insertions are complete.
    }

    // FIX: build final output once, excluding any triangle touching the super-triangle
    out.clear(); // FIX
    out.reserve(triangles.size() * 3);
    for (auto T : triangles) {
        if (T.a >= vertexCount || T.b >= vertexCount || T.c >= vertexCount)
            continue; // drop super-triangle connections
        // final CCW
        if (!ccw(T.a, T.b, T.c)) std::swap(T.b, T.c);
        out.push_back(T.a);
        out.push_back(T.b);
        out.push_back(T.c);
    }

    return out;
}

std::vector<uint32_t> TriangleFactory::CreateMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                                  double& cumulatedEdgeLength)
{
    cumulatedEdgeLength = 0.0;
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

    auto triPerimeter = [&](uint32_t ia, uint32_t ib, uint32_t ic) -> double {
        auto L = [&](uint32_t p, uint32_t q) {
            const auto& P = vertices[p]; const auto& Q = vertices[q];
            const double dx = double(Q.position.x) - double(P.position.x);
            const double dy = double(Q.position.y) - double(P.position.y);
            return std::sqrt(dx*dx + dy*dy);
        };
        return L(ia, ib) + L(ib, ic) + L(ic, ia);
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
            cumulatedEdgeLength += triPerimeter(poly[0], poly[1], poly[2]);
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

        cumulatedEdgeLength += triPerimeter(A, B, C);

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

std::vector<uint32_t> TriangleFactory::CreateStripTriangulation(const std::vector<Vertex>& vertices)
{
    std::vector<uint32_t> indices;
    const size_t n = vertices.size();
    if (n < 3) return indices;

    auto signedArea = [&]() -> double {
        double A = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const auto& p = vertices[i].position;
            const auto& q = vertices[(i + 1) % n].position;
            A += double(p.x) * double(q.y) - double(p.y) * double(q.x);
        }
        return 0.5 * A;
    };
    const bool isCW = signedArea() < 0.0;

    std::vector<uint32_t> stripOrder;
    stripOrder.reserve(n);
    uint32_t L = 0, R = static_cast<uint32_t>(n - 1);
    while (L <= R) {
        stripOrder.push_back(L++);
        if (L > R) break;
        stripOrder.push_back(R--);
    }

    indices.reserve((n - 2) * 3);
    for (size_t k = 0; k + 2 < stripOrder.size(); ++k) {
        uint32_t a = stripOrder[k + 0];
        uint32_t b = stripOrder[k + 1];
        uint32_t c = stripOrder[k + 2];

        const bool swapAB = isCW ? ((k % 2) == 0) : ((k % 2) == 1);
        if (swapAB) std::swap(a, b);

        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }

    return indices;
}

std::vector<uint32_t>
TriangleFactory::CreateConvexMaxMinAreaTriangulation(const std::vector<Vertex>& vertices,
                                                     double& cumulatedEdgeLength)
{
    cumulatedEdgeLength = 0.0;

    const int n = (int)vertices.size();
    std::vector<uint32_t> out;
    if (n < 3) return out;

    // --- Helpers (same as above)
    auto signedAreaRing = [&]() -> double {
        double A = 0.0;
        for (int i = 0; i < n; ++i) {
            const auto& p = vertices[i].position;
            const auto& q = vertices[(i + 1) % n].position;
            A += double(p.x) * double(q.y) - double(p.y) * double(q.x);
        }
        return 0.5 * A;
    };
    auto areaTri = [&](int ia, int ib, int ic) -> double {
        const auto &A = vertices[ia].position, &B = vertices[ib].position, &C = vertices[ic].position;
        return std::abs((double(B.x)-A.x)*(double(C.y)-A.y) - (double(B.y)-A.y)*(double(C.x)-A.x)) * 0.5;
    };
    auto periTri = [&](uint32_t ia, uint32_t ib, uint32_t ic) -> double {
        auto L = [&](uint32_t p, uint32_t q){
            const auto &P = vertices[p].position, &Q = vertices[q].position;
            const double dx = double(Q.x) - double(P.x), dy = double(Q.y) - double(P.y);
            return std::sqrt(dx*dx + dy*dy);
        };
        return L(ia,ib) + L(ib,ic) + L(ic,ia);
    };
    auto orientCCW = [&](uint32_t a, uint32_t b, uint32_t c) -> bool {
        const auto &A = vertices[a].position, &B = vertices[b].position, &C = vertices[c].position;
        const double v = (double(B.x)-A.x)*(double(C.y)-A.y) - (double(B.y)-A.y)*(double(C.x)-A.x);
        return v > 0.0;
    };

    // CCW traversal via permutation
    std::vector<uint32_t> order(n);
    if (signedAreaRing() >= 0.0) {
        for (int i = 0; i < n; ++i) order[i] = (uint32_t)i;
    } else {
        for (int i = 0; i < n; ++i) order[i] = (uint32_t)(n - 1 - i);
    }

    // DP[i][j] = maximal achievable minimum triangle area on chain (i..j)
    std::vector<double> dp(n * n, 0.0);
    std::vector<int>    split(n * n, -1);
    auto DP = [&](int i,int j)->double& { return dp[i * n + j]; };
    auto SP = [&](int i,int j)->int&    { return split[i * n + j]; };

    // For segments (no triangle), the "minimum over an empty set" should be +INF
    const double INF = std::numeric_limits<double>::infinity();
    for (int i = 0; i + 1 < n; ++i) DP(i, i + 1) = INF;

    for (int len = 2; len < n; ++len) {
        for (int i = 0; i + len < n; ++i) {
            const int j = i + len;
            double best = 0.0;
            int bestK = -1;
            for (int k = i + 1; k < j; ++k) {
                const int a = (int)order[i], b = (int)order[k], c = (int)order[j];
                const double triA = areaTri(a, b, c);
                // Bottleneck = min of { left, right, this triangle }
                const double bottleneck = std::min({ DP(i,k), DP(k,j), triA });
                if (bottleneck > best) { best = bottleneck; bestK = k; }
            }
            DP(i,j) = best; SP(i,j) = bestK;
        }
    }

    // Reconstruct; emit CCW triangles in original index space
    std::vector<uint32_t> indices;
    indices.reserve(3 * (n - 2));
    std::function<void(int,int)> emit = [&](int i,int j){
        int k = SP(i,j); if (k < 0) return;
        uint32_t a = order[i], b = order[k], c = order[j];
        if (!orientCCW(a,b,c)) std::swap(b,c);
        indices.push_back(a); indices.push_back(b); indices.push_back(c);
        cumulatedEdgeLength += periTri(a,b,c);
        emit(i,k);
        emit(k,j);
    };
    emit(0, n - 1);

    out = std::move(indices);
    return out;
}

std::vector<uint32_t>
TriangleFactory::CreateConvexMinMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                                     double& cumulatedEdgeLength)
{
    cumulatedEdgeLength = 0.0;

    const int n = (int)vertices.size();
    std::vector<uint32_t> out;
    if (n < 3) return out;

    // --- Helpers
    auto signedAreaRing = [&]() -> double {
        double A = 0.0;
        for (int i = 0; i < n; ++i) {
            const auto& p = vertices[i].position;
            const auto& q = vertices[(i + 1) % n].position;
            A += double(p.x) * double(q.y) - double(p.y) * double(q.x);
        }
        return 0.5 * A;
    };
    auto areaTri = [&](int ia, int ib, int ic) -> double {
        const auto &A = vertices[ia].position, &B = vertices[ib].position, &C = vertices[ic].position;
        return std::abs((double(B.x)-A.x)*(double(C.y)-A.y) - (double(B.y)-A.y)*(double(C.x)-A.x)) * 0.5;
    };
    auto periTri = [&](uint32_t ia, uint32_t ib, uint32_t ic) -> double {
        auto L = [&](uint32_t p, uint32_t q){
            const auto &P = vertices[p].position, &Q = vertices[q].position;
            const double dx = double(Q.x) - double(P.x), dy = double(Q.y) - double(P.y);
            return std::sqrt(dx*dx + dy*dy);
        };
        return L(ia,ib) + L(ib,ic) + L(ic,ia);
    };
    auto orientCCW = [&](uint32_t a, uint32_t b, uint32_t c) -> bool {
        const auto &A = vertices[a].position, &B = vertices[b].position, &C = vertices[c].position;
        const double v = (double(B.x)-A.x)*(double(C.y)-A.y) - (double(B.y)-A.y)*(double(C.x)-A.x);
        return v > 0.0;
    };

    // --- Ensure we traverse CCW without touching the input array
    // If input is CW, we’ll use an index permutation “order” that reverses it.
    std::vector<uint32_t> order(n);
    if (signedAreaRing() >= 0.0) {
        for (int i = 0; i < n; ++i) order[i] = (uint32_t)i;             // CCW ring
    } else {
        for (int i = 0; i < n; ++i) order[i] = (uint32_t)(n - 1 - i);   // CW -> reversed to CCW
    }

    // DP over chain [0..n-1] in this CCW order; store splits in DP space.
    std::vector<double> dp(n * n, 0.0);
    std::vector<int>    split(n * n, -1);
    auto DP = [&](int i,int j)->double& { return dp[i * n + j]; };
    auto SP = [&](int i,int j)->int&    { return split[i * n + j]; };

    for (int i = 0; i + 1 < n; ++i) DP(i, i + 1) = 0.0;

    for (int len = 2; len < n; ++len) {
        for (int i = 0; i + len < n; ++i) {
            const int j = i + len;
            double best = std::numeric_limits<double>::infinity();
            int bestK = -1;
            for (int k = i + 1; k < j; ++k) {
                // Map chain indices to original vertex indices
                const int a = (int)order[i], b = (int)order[k], c = (int)order[j];
                const double triA = areaTri(a, b, c);
                const double cost = std::max({ DP(i,k), DP(k,j), triA });
                if (cost < best) { best = cost; bestK = k; }
            }
            DP(i,j) = best; SP(i,j) = bestK;
        }
    }

    // Reconstruct; emit CCW triangles in original index space
    std::vector<uint32_t> indices;
    indices.reserve(3 * (n - 2));
    std::function<void(int,int)> emit = [&](int i,int j){
        int k = SP(i,j); if (k < 0) return;
        uint32_t a = order[i], b = order[k], c = order[j];
        if (!orientCCW(a,b,c)) std::swap(b,c);
        indices.push_back(a); indices.push_back(b); indices.push_back(c);
        cumulatedEdgeLength += periTri(a,b,c);
        emit(i,k);
        emit(k,j);
    };
    emit(0, n - 1);

    out = std::move(indices);
    return out;
}

std::vector<uint32_t>
TriangleFactory::TriangulatePolygon_CDT(const std::vector<Vertex>& vertices)
{
    const int n = (int)vertices.size();
    Eigen::Matrix<double, Eigen::Dynamic, 2> V(n,2);
    for (int i=0;i<n;++i) {
        V(i,0) = vertices[i].position.x;
        V(i,1) = vertices[i].position.y;
    }

    // Boundary edges (i, i+1) with wrap
    Eigen::Matrix<int, Eigen::Dynamic, 2> E(n,2);
    for (int i=0;i<n;++i) {
        E(i,0) = i;
        E(i,1) = (i+1) % n;
    }

    Eigen::Matrix<double, Eigen::Dynamic, 2> H(0,2); // no holes
    const std::string flags = "pQz"; // p = PSLG (respect segments)

    Eigen::Matrix<double, Eigen::Dynamic, 2> Vout;
    Eigen::Matrix<int,    Eigen::Dynamic, 3> Fout;
    igl::triangle::triangulate(V, E, H, flags, Vout, Fout);

    std::vector<uint32_t> idx; idx.reserve(Fout.rows()*3);
    for (int t=0;t<Fout.rows();++t) {
        idx.push_back((uint32_t)Fout(t,0));
        idx.push_back((uint32_t)Fout(t,1));
        idx.push_back((uint32_t)Fout(t,2));
    }
    return idx;
}
