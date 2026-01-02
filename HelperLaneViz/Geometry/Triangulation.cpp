//
//  Triangulation.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#include "Triangulation.h"

#include "TriangulationHelpers.h"

#include <numeric>
#include <limits>
#include <cmath>

#include <Eigen/Core>
#include <igl/triangle/triangulate.h>
#include <mapbox/earcut.hpp>

namespace Triangulation {

// MARK: Simple triangulations
struct EdgeKey {
    uint32_t a;
    uint32_t b;
    bool operator==(const EdgeKey& o) const {
        return a == o.a && b == o.b;
    }
};
struct EdgeKeyHash {
    size_t operator()(const EdgeKey& k) const noexcept {
        // 64-bit mix of two 32-bit ints
        return (size_t(k.a) << 32) ^ size_t(k.b);
    }
};

struct EdgeAdj {
    int t0 = -1;
    int t1 = -1;
    uint32_t opp0 = 0;
    uint32_t opp1 = 0;
    uint32_t ver = 0; // bump on any mutation
};

static inline EdgeKey MakeKey(uint32_t u, uint32_t v) {
    return EdgeKey{ std::min(u, v), std::max(u, v) };
}

static inline std::array<uint32_t,3> MakeCCW(const std::vector<Vertex>& v,
                                             uint32_t i0,
                                             uint32_t i1,
                                             uint32_t i2) {
    // If (i0,i1,i2) is CCW keep, else swap i1/i2
    if (Helpers::Cross2D(v[i0].position, v[i1].position, v[i2].position) >= 0.0f) {
        return {i0, i1, i2};
    } else {
        return {i0, i2, i1};
    }
}

std::vector<uint32_t> OptimizeByMinLengthFlips_PQ(const std::vector<Vertex>& vertices,
                                                std::vector<uint32_t> indices,
    /*set e.g. 4*triCount for near-linear-ish*/ int maxFlips = -1,
    /* set e.g. 20*triCount to cap heap work*/  int maxPops  = -1) {
    const int triCount = int(indices.size() / 3);
    if (triCount <= 0) {
        return indices;
    }

    auto tri = [&](int t, int k) -> uint32_t& { return indices[3 * t + k]; };
    auto tric = [&](int t, int k) -> uint32_t { return indices[3 * t + k]; };

    std::unordered_map<EdgeKey, EdgeAdj, EdgeKeyHash> adj;
    adj.reserve(indices.size() * 2);

    // u, v are indices of the edge endpoints
    // triangle is the triangle index in which the edge lies
    // ov is the index of the vertex opposite of the edge in the respective triangle
    auto addEdge = [&](uint32_t u, uint32_t v, int triangle, uint32_t ov) {
        EdgeKey key = MakeKey(u, v);
        auto& edge = adj[key];
        // any write mutates bump version
        ++edge.ver;

        if (edge.t0 == -1) {
            edge.t0 = triangle;
            edge.opp0 = ov;
        } else {
            edge.t1 = triangle;
            edge.opp1 = ov;
        }
    };

    auto rebuildTriangle = [&](int t) {
        const uint32_t a = tric(t, 0);
        const uint32_t b = tric(t, 1);
        const uint32_t c = tric(t, 2);
        addEdge(a, b, t, c);
        addEdge(b, c, t, a);
        addEdge(c, a, t, b);
    };

    auto clearTriangle = [&](int t) {
        const uint32_t a = tric(t, 0);
        const uint32_t b = tric(t, 1);
        const uint32_t c = tric(t, 2);
        const EdgeKey e0 = MakeKey(a, b);
        const EdgeKey e1 = MakeKey(b, c);
        const EdgeKey e2 = MakeKey(c, a);

        auto clearEdge = [&](const EdgeKey& k) {
            auto it = adj.find(k);
            if (it == adj.end()) {
                return;
            }
            auto& E = it->second;
            // any write mutates bump version
            ++E.ver;

            if (E.t0 == t) {
                E.t0 = -1;
                E.opp0 = 0;
            }
            if (E.t1 == t) {
                E.t1 = -1;
                E.opp1 = 0;
            }
        };

        clearEdge(e0);
        clearEdge(e1);
        clearEdge(e2);
    };

    for (int t = 0; t < triCount; ++t) {
        rebuildTriangle(t);
    }

    // Priority queue of candidate flips (max gain first)
    struct Candidate {
        double gain; // >0 is improving
        EdgeKey key;
        uint32_t ver;
    };

    struct CandidateLess {
        bool operator()(const Candidate& a, const Candidate& b) const {
            return a.gain < b.gain; // max-heap
        }
    };
    std::priority_queue<Candidate, std::vector<Candidate>, CandidateLess> pq;

    auto len2 = [&](uint32_t i, uint32_t j) -> float {
        const simd_float2 diff = vertices[i].position.xy - vertices[j].position.xy;
        return simd_length_squared(diff);
    };

    auto tryPushEdge = [&](const EdgeKey& k) {
        auto it = adj.find(k);
        if (it == adj.end()) {
            return;
        }
        const auto& E = it->second;
        if (E.t0 == -1 || E.t1 == -1) {
            return; // Edge is not 2 sided (likely boundary)
        }

        const uint32_t a = k.a;
        const uint32_t b = k.b;
        const uint32_t c = E.opp0;
        const uint32_t d = E.opp1;

        if (!Helpers::IsConvexQuad(vertices, a, b, c, d)) {
            return;
        }

        // improving if new diagonal shorter
        const double oldD = len2(a, b);
        const double newD = len2(c, d);
        const double gain = oldD - newD;
        if (gain <= 0.0) {
            return;
        }

        pq.push(Candidate{gain, k, E.ver});
    };

    for (const auto& [k, e] : adj) {
        (void)e;
        tryPushEdge(k);
    }

    auto pushTriEdges = [&](int t) {
        const uint32_t a = tric(t, 0);
        const uint32_t b = tric(t, 1);
        const uint32_t c = tric(t, 2);
        tryPushEdge(MakeKey(a, b));
        tryPushEdge(MakeKey(b, c));
        tryPushEdge(MakeKey(c, a));
    };

    int flips = 0;
    int pops  = 0;

    while (!pq.empty()) {
        if (maxPops >= 0 && pops >= maxPops) {
            break;
        }
        ++pops;

        const Candidate candidate = pq.top();
        pq.pop();

        auto it = adj.find(candidate.key);
        if (it == adj.end()) {
            continue;
        }

        auto& E = it->second;
        if (E.ver != candidate.ver) {
            continue;
        }
        if (E.t0 == -1 || E.t1 == -1) {
            continue;  // not interior
        }

        const uint32_t a = candidate.key.a;
        const uint32_t b = candidate.key.b;
        const uint32_t c = E.opp0;
        const uint32_t d = E.opp1;
        const int t0 = E.t0;
        const int t1 = E.t1;

        // Re-check (neighbors may have moved but ver guard usually catches)
        if (!Helpers::IsConvexQuad(vertices, a, b, c, d)) {
            continue;
        }

        const double oldD = len2(a, b);
        const double newD = len2(c, d);
        if (newD >= oldD) {
            continue; // no longer improving
        }

        // Optional hard cap on number of flips
        if (maxFlips >= 0 && flips >= maxFlips) {
            break;
        }

        // Build new triangles using the other diagonal (c-d)
        const auto T0 = MakeCCW(vertices, c, d, a);
        const auto T1 = MakeCCW(vertices, d, c, b);

        // Update mesh
        clearTriangle(t0);
        clearTriangle(t1);

        tri(t0, 0) = T0[0];
        tri(t0, 1) = T0[1];
        tri(t0, 2) = T0[2];

        tri(t1, 0) = T1[0];
        tri(t1, 1) = T1[1];
        tri(t1, 2) = T1[2];

        rebuildTriangle(t0);
        rebuildTriangle(t1);

        // Re-enqueue affected neighborhood (only local)
        pushTriEdges(t0);
        pushTriEdges(t1);

        ++flips;
    }

    return indices;
}

Indices EarClipping(const std::vector<Vertex>& vertices) {
    Indices indices;
    const size_t n = vertices.size();

    if (n < 3) return indices;

    const bool isCCW = Helpers::PolygonSignedArea(vertices) >= 0.0;

    // Working list of remaining vertex indices
    std::vector<uint32_t> polygon(n);
    std::iota(polygon.begin(), polygon.end(), 0);

    indices.reserve((n - 2) * 3);

    // Clip ears until only 3 vertices remain
    while (polygon.size() > 3) {
        const size_t size = polygon.size();
        bool earFound = false;

        for (size_t i = 0; i < size; ++i) {
            const size_t prev = (i + size - 1) % size;
            const size_t next = (i + 1) % size;

            if (Helpers::IsValidEar(vertices, polygon, prev, i, next, isCCW)) {
                // Emit triangle
                indices.push_back(polygon[prev]);
                indices.push_back(polygon[i]);
                indices.push_back(polygon[next]);

                // Remove the ear vertex
                polygon.erase(polygon.begin() + static_cast<ptrdiff_t>(i));
                earFound = true;
                break;
            }
        }

        if (!earFound) {
            break;
        }
    }

    // Add final triangle
    if (polygon.size() == 3) {
        indices.push_back(polygon[0]);
        indices.push_back(polygon[1]);
        indices.push_back(polygon[2]);
    }

    return indices;
}

Indices EarClippingMapbox(const std::vector<Vertex>& vertices) {
    Indices indices;
    const size_t n = vertices.size();
    if (n < 3) {
        return indices;
    }

    // Convert Vertex format to earcut format (std::vector<std::vector<Point>>)
    // earcut expects: std::vector<std::vector<std::array<Coord, 2>>>
    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> polygon;

    // Create the main polygon contour
    std::vector<Point> contour;
    contour.reserve(n);
    for (const auto& v : vertices) {
        contour.push_back({static_cast<double>(v.position.x), static_cast<double>(v.position.y)});
    }
    polygon.push_back(std::move(contour));

    // Run earcut triangulation
    // Returns array of indices that refer to the vertices of the input polygon
    indices = mapbox::earcut<uint32_t>(polygon);
    return indices;
}

Indices EarClippingMapboxWithEdgeFlips(const std::vector<Vertex>& vertices) {
    Indices indices;
    const size_t n = vertices.size();
    if (n < 3) {
        return indices;
    }

    // Convert Vertex format to earcut format (std::vector<std::vector<std::array<Coord, 2>>>)
    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> polygon;
    
    // Create the main polygon contour
    std::vector<Point> contour;
    contour.reserve(n);
    for (const auto& v : vertices) {
        contour.push_back({static_cast<double>(v.position.x), static_cast<double>(v.position.y)});
    }
    polygon.push_back(std::move(contour));

    indices = mapbox::earcut<uint32_t>(polygon);
    indices = OptimizeByMinLengthFlips_PQ(vertices, indices);
    return indices;
}

Indices CentroidFan(std::vector<Vertex>& vertices) {
    Indices indices;
    const size_t originalVertexCount = vertices.size();

    if (originalVertexCount < 3) {
        return indices;
    }

    // Compute centroid by averaging all vertex positions
    simd_float2 centroid{0.0f, 0.0f};
    for (const auto& vertex : vertices) {
        centroid.x += vertex.position.x;
        centroid.y += vertex.position.y;
    }
    centroid.x /= static_cast<float>(originalVertexCount);
    centroid.y /= static_cast<float>(originalVertexCount);

    // Append centroid as new vertex
    Vertex centroidVertex{.position = {centroid.x, centroid.y, 1.0f}};
    vertices.push_back(centroidVertex);
    const uint32_t centroidIndex = static_cast<uint32_t>(vertices.size() - 1);

    indices.reserve(originalVertexCount * 3);

    for (uint32_t i = 0; i < originalVertexCount; ++i) {
        const uint32_t nextIndex = (i + 1) % static_cast<uint32_t>(originalVertexCount);

        indices.push_back(i);
        indices.push_back(nextIndex);
        indices.push_back(centroidIndex);
    }

    return indices;
}

Indices Strip(const std::vector<Vertex>& vertices) {
    Indices indices;
    const size_t vertexCount = vertices.size();

    if (vertexCount < 3) {
        return indices;
    }

    std::vector<uint32_t> stripOrder;
    stripOrder.reserve(vertexCount);

    size_t leftIdx = 0;
    size_t rightIdx = vertexCount - 1;

    while (leftIdx <= rightIdx) {
        stripOrder.push_back(static_cast<uint32_t>(leftIdx++));
        if (leftIdx > rightIdx) break;
        stripOrder.push_back(static_cast<uint32_t>(rightIdx--));
    }

    indices.reserve((vertexCount - 2) * 3);
    for (size_t i = 0; i + 2 < stripOrder.size(); ++i) {
        indices.push_back(stripOrder[i]);
        indices.push_back(stripOrder[i + 1]);
        indices.push_back(stripOrder[i + 2]);
    }

    return indices;
}

// MARK: DP triangulations
Indices MinimumWeight(const std::vector<Vertex>& vertices, bool shouldHandleConcave) {
    Indices indices;
    const int vertexCount = static_cast<uint32_t>(vertices.size());
    if (vertexCount < 3) {
        return indices;
    }

    Helpers::DiagonalTable diagTable;
    if (shouldHandleConcave) {
        diagTable = Helpers::BuildDiagonalTable(vertices);
    }

    // DP tables:
    // dp[i][j] = minimum edge weight to triangulate polygon from i to j
    // split[i][j] = optimal split point k for the interval [i, j]
    std::vector<double> dpTable(vertexCount * vertexCount, 0.0);
    std::vector<int> splitTable(vertexCount * vertexCount, -1);
    
    auto dp = [&](int i, int j) -> double& {
        return dpTable[i * vertexCount + j];
    };
    auto split = [&](int i, int j) -> int& {
        return splitTable[i * vertexCount + j];
    };
    
    // Initialize: adjacent vertices need no triangulation
    for (int i = 0; i < vertexCount - 1; ++i) {
        dp(i, i + 1) = 0.0;
    }
    
    // Fill DP table for increasing chain lengths
    for (int chainLength = 2; chainLength <= vertexCount - 1; ++chainLength) {
        for (int startIndex = 0; startIndex + chainLength < vertexCount; ++startIndex) {
            const int endIndex = startIndex + chainLength;
            
            double minimumCost = std::numeric_limits<double>::infinity();
            int optimalSplit = -1;
            
            for (int splitPoint = startIndex + 1; splitPoint < endIndex; ++splitPoint) {
                if (shouldHandleConcave) {
                    if (!IsTriangleInsidePolygon(vertices, startIndex, splitPoint, endIndex, diagTable)) {
                        continue;
                    }
                } else {
                    const double area = Helpers::TriangleArea(vertices, startIndex, splitPoint, endIndex);
                    if (area <= 0.0) {
                        continue;
                    }
                }

                // Cost = left subproblem + right subproblem + new internal edges
                const double internalEdgeCost = Helpers::EdgeLength(vertices[startIndex], vertices[splitPoint])
                                              + Helpers::EdgeLength(vertices[splitPoint], vertices[endIndex]);
                const double totalCost = dp(startIndex, splitPoint)
                                       + dp(splitPoint, endIndex)
                                       + internalEdgeCost;
                
                if (totalCost < minimumCost) {
                    minimumCost = totalCost;
                    optimalSplit = splitPoint;
                }
            }
            
            dp(startIndex, endIndex) = minimumCost;
            split(startIndex, endIndex) = optimalSplit;
        }
    }
    
    // Reconstruct triangles via recursive traversal
    indices.reserve(3 * (vertexCount - 2));
    
    std::function<void(int, int)> emitTriangles = [&](int startIndex, int endIndex) {
        const int splitPoint = split(startIndex, endIndex);
        if (splitPoint < 0) {
            // No split point found
            return;
        }
        
        indices.push_back(static_cast<uint32_t>(startIndex));
        indices.push_back(static_cast<uint32_t>(splitPoint));
        indices.push_back(static_cast<uint32_t>(endIndex));
        
        // Recurse on sub-chains
        if (splitPoint > startIndex + 1) {
            emitTriangles(startIndex, splitPoint);
        }
        if (endIndex > splitPoint + 1) {
            emitTriangles(splitPoint, endIndex);
        }
    };
    
    // For closed polygons, we need to triangulate the full chain [0, vertexCount-1]
    emitTriangles(0, vertexCount - 1);

    const size_t expectedTriangles = vertexCount - 2;
    if (indices.size() / 3 < expectedTriangles) {
        static_assert(true, "Triangulation not sussecful");
        return {};
    }

    return indices;
}

Indices MaxMinArea(const std::vector<Vertex>& vertices, bool shouldHandleConcave) {
    Indices indices;
    const int vertexCount = static_cast<int>(vertices.size());
    if (vertexCount < 3) {
        return indices;
    }

    Helpers::DiagonalTable diagTable;
    if (shouldHandleConcave) {
        diagTable = Helpers::BuildDiagonalTable(vertices);
    }
    
    // DP tables: dp[i][j] = maximum achievable minimum triangle area for chain [i, j]
    std::vector<double> dpTable(vertexCount * vertexCount, 0.0);
    std::vector<int> splitTable(vertexCount * vertexCount, -1);
    
    auto dp = [&](int i, int j) -> double& { return dpTable[i * vertexCount + j]; };
    auto split = [&](int i, int j) -> int& { return splitTable[i * vertexCount + j]; };

    // Initialize: adjacent pairs have infinite "minimum" (no triangles to constrain)
    constexpr double INFINITY_VALUE = std::numeric_limits<double>::infinity();
    for (int i = 0; i + 1 < vertexCount; ++i) {
        dp(i, i + 1) = INFINITY_VALUE;
    }
    
    // Fill DP table for increasing chain lengths
    for (int chainLength = 2; chainLength < vertexCount; ++chainLength) {
        for (int start = 0; start + chainLength < vertexCount; ++start) {
            const int end = start + chainLength;

            double bestMinArea = 0.0;
            int bestSplit = -1;

            for (int mid = start + 1; mid < end; ++mid) {
                if (shouldHandleConcave) {
                    if (!IsTriangleInsidePolygon(vertices, start, mid, end, diagTable)) {
                        continue;
                    }
                } else {
                    if (Helpers::TriangleArea(vertices, start, mid, end) <= 0.0) {
                        continue;
                    }
                }

                const double triArea = Helpers::TriangleArea(vertices, start, mid, end);

                // Bottleneck for the chain = min(left, right, this triangle)
                const double bottleneck = std::min({ dp(start, mid), dp(mid, end), triArea });

                if (bottleneck > bestMinArea) {
                    bestMinArea = bottleneck;
                    bestSplit   = mid;
                }
            }

            dp(start, end) = bestMinArea;
            split(start, end) = bestSplit;
        }
    }
    
    // Reconstruct triangles (respect CCW orientation)
    indices.reserve(3 * (vertexCount - 2));

    std::function<void(int, int)> emitTriangles = [&](int start, int end) {
        const int mid = split(start, end);
        if (mid < 0) return;

        indices.push_back(start);
        indices.push_back(mid);
        indices.push_back(end);

        if (mid > start + 1) {
            emitTriangles(start, mid);
        }
        if (end > mid + 1) {
            emitTriangles(mid, end);
        }
    };

    emitTriangles(0, vertexCount - 1);
    return indices;
}

Indices MinMaxArea(const std::vector<Vertex>& vertices, bool shouldHandleConcave) {
    Indices indices;
    const int vertexCount = static_cast<int>(vertices.size());
    if (vertexCount < 3) {
        return indices;
    }

    Helpers::DiagonalTable diagTable;
    if (shouldHandleConcave) {
        diagTable = Helpers::BuildDiagonalTable(vertices);
    }

    // DP tables: dp[i][j] = minimum achievable maximum triangle area for chain [i, j]
    std::vector<double> dpTable(vertexCount * vertexCount, 0.0);
    std::vector<int> splitTable(vertexCount * vertexCount, -1);

    auto dp = [&](int i, int j) -> double& { return dpTable[i * vertexCount + j]; };
    auto split = [&](int i, int j) -> int&    { return splitTable[i * vertexCount + j]; };

    // Initialize: adjacent pairs have zero cost (no triangles)
    for (int i = 0; i + 1 < vertexCount; ++i) {
        dp(i, i + 1) = 0.0;
    }

    // Fill DP table for increasing chain lengths
    for (int chainLength = 2; chainLength < vertexCount; ++chainLength) {
        for (int start = 0; start + chainLength < vertexCount; ++start) {
            const int end = start + chainLength;

            double bestMaxArea = std::numeric_limits<double>::infinity();
            int    bestSplit   = -1;

            for (int mid = start + 1; mid < end; ++mid) {
                if (shouldHandleConcave) {
                    if (!IsTriangleInsidePolygon(vertices, start, mid, end, diagTable)) {
                        continue;
                    }
                } else {
                    if (Helpers::TriangleArea(vertices, start, mid, end) <= 0.0) {
                        continue;
                    }
                }

                const double triArea = Helpers::TriangleArea(vertices, start, mid, end);

                // Cost = maximum of {left subproblem, right subproblem, this triangle}
                const double cost = std::max({ dp(start, mid), dp(mid, end), triArea });

                if (cost < bestMaxArea) {
                    bestMaxArea = cost;
                    bestSplit = mid;
                }
            }

            dp(start, end) = bestMaxArea;
            split(start, end) = bestSplit;
        }
    }

    // Reconstruct triangles with CCW orientation
    indices.reserve(3 * (vertexCount - 2));

    std::function<void(int, int)> emitTriangles = [&](int start, int end) {
        const int mid = split(start, end);
        if (mid < 0) return;

        indices.push_back(start);
        indices.push_back(mid);
        indices.push_back(end);

        if (mid > start + 1) emitTriangles(start, mid);
        if (end > mid + 1)   emitTriangles(mid,   end);
    };

    emitTriangles(0, vertexCount - 1);

    return indices;
}

// MARK: Max Area and CDT
Indices GreedyMaxArea(const std::vector<Vertex>& vertices, bool shouldHandleConcave) {
    Indices indices;
    const size_t vertexCount = vertices.size();
    const auto diagTable = Helpers::BuildDiagonalTable(vertices);

    if (vertexCount < 3) {
        return indices;
    }

    // For convex polygons, skip CCW order rebuild
    std::vector<uint32_t> ccwOrder;
    if (shouldHandleConcave) {
        ccwOrder = Helpers::BuildCCWOrder(vertices);
    } else {
        ccwOrder.resize(vertexCount);
        std::iota(ccwOrder.begin(), ccwOrder.end(), 0);
    }

    // Recursive solver: triangulates a sub-polygon by selecting the largest triangle
    // polygon is indices into ccwOrder array (which gives actual vertex indices)
    std::function<void(const std::vector<size_t>&)> triangulateSubPolygon =
        [&](const std::vector<size_t>& polygon) {
            const size_t polygonSize = polygon.size();

            if (polygonSize < 3) {
                return;
            }

            if (polygonSize == 3) {
                uint32_t va = ccwOrder[polygon[0]];
                uint32_t vb = ccwOrder[polygon[1]];
                uint32_t vc = ccwOrder[polygon[2]];
                indices.insert(indices.end(), {va, vb, vc});
                return;
            }

            // Find the largest-area triangle among all valid triples
            // polygon array is in boundary order (contiguous arc), so try all combinations
            double largestArea = -1.0;
            size_t bestI = 0, bestJ = 1, bestK = 2;

            for (size_t i = 0; i + 2 < polygonSize; ++i) {
                for (size_t j = i + 1; j + 1 < polygonSize; ++j) {
                    for (size_t k = j + 1; k < polygonSize; ++k) {
                        uint32_t vi = ccwOrder[polygon[i]];
                        uint32_t vj = ccwOrder[polygon[j]];
                        uint32_t vk = ccwOrder[polygon[k]];

                        const double area = Helpers::TriangleArea(vertices, vi, vj, vk);
                        if (shouldHandleConcave) {
                            if (!Helpers::IsTriangleInsidePolygon(vertices, static_cast<int>(vi), static_cast<int>(vj), static_cast<int>(vk), diagTable)) {
                                continue;
                            }
                        } else {
                            // For convex polygons, still ensure triangle has positive area (degenerate check)
                            if (area <= 0.0) {
                                continue;
                            }
                        }
                        if (area > largestArea) {
                            largestArea = area;
                            bestI = i;
                            bestJ = j;
                            bestK = k;
                        }
                    }
                }
            }

            if (largestArea < 0) {
                // No valid triangle found - this shouldn't happen for valid polygons
                // Fallback: use first three vertices
                if (polygonSize >= 3) {
                    uint32_t va = ccwOrder[polygon[0]];
                    uint32_t vb = ccwOrder[polygon[1]];
                    uint32_t vc = ccwOrder[polygon[2]];
                    indices.insert(indices.end(), {va, vb, vc});
                }
                return;
            }

            // Emit the selected triangle
            const uint32_t vertexA = ccwOrder[polygon[bestI]];
            const uint32_t vertexB = ccwOrder[polygon[bestJ]];
            const uint32_t vertexC = ccwOrder[polygon[bestK]];

            indices.insert(indices.end(), {vertexA, vertexB, vertexC});

            // Build sub-polygons from the arcs between selected vertices (respecting boundary order)
            // The polygon array represents a contiguous arc, so we need to handle wrap-around correctly
            auto buildArc = [&](size_t startIdx, size_t endIdx) -> std::vector<size_t> {
                std::vector<size_t> arc;
                if (startIdx < endIdx) {
                    // Normal case: arc from startIdx to endIdx (inclusive)
                    for (size_t i = startIdx; i <= endIdx; ++i) {
                        arc.push_back(polygon[i]);
                    }
                } else if (startIdx > endIdx) {
                    // Wrap-around case: from startIdx to end of array, then from start to endIdx
                    for (size_t i = startIdx; i < polygonSize; ++i) {
                        arc.push_back(polygon[i]);
                    }
                    for (size_t i = 0; i <= endIdx; ++i) {
                        arc.push_back(polygon[i]);
                    }
                } else {
                    // startIdx == endIdx: single vertex, return empty (will be filtered)
                    arc.push_back(polygon[startIdx]);
                }
                return arc;
            };

            // Three arcs: A→B, B→C, C→A (respecting boundary order)
            // Note: bestI < bestJ < bestK in the polygon array (since we iterate in order)
            // Each arc includes both endpoints to form closed sub-polygons
            const auto arcAB = buildArc(bestI, bestJ);
            const auto arcBC = buildArc(bestJ, bestK);
            // arcCA wraps from bestK back to bestI (closed polygon)
            std::vector<size_t> arcCA;
            arcCA.push_back(polygon[bestK]);  // Include endpoint
            for (size_t i = bestK + 1; i < polygonSize; ++i) {
                arcCA.push_back(polygon[i]);
            }
            for (size_t i = 0; i <= bestI; ++i) {
                arcCA.push_back(polygon[i]);  // Include endpoint
            }

            // Recurse on arcs with 3+ vertices
            if (arcAB.size() >= 3) triangulateSubPolygon(arcAB);
            if (arcBC.size() >= 3) triangulateSubPolygon(arcBC);
            if (arcCA.size() >= 3) triangulateSubPolygon(arcCA);
        };

    // Initialize with full polygon (indices into ccwOrder)
    std::vector<size_t> fullPolygon(vertexCount);
    std::iota(fullPolygon.begin(), fullPolygon.end(), 0);

    indices.reserve((vertexCount - 2) * 3);
    triangulateSubPolygon(fullPolygon);

    return indices;
}


Indices ConstrainedDelaunay(const std::vector<Vertex>& vertices) {
    const int vertexCount = static_cast<int>(vertices.size());
    
    // Prepare vertex matrix for libigl
    Eigen::Matrix<double, Eigen::Dynamic, 2> inputVertices(vertexCount, 2);
    for (int i = 0; i < vertexCount; ++i) {
        inputVertices(i, 0) = vertices[i].position.x;
        inputVertices(i, 1) = vertices[i].position.y;
    }
    
    // Define boundary edges (closed polygon)
    Eigen::Matrix<int, Eigen::Dynamic, 2> boundaryEdges(vertexCount, 2);
    for (int i = 0; i < vertexCount; ++i) {
        boundaryEdges(i, 0) = i;
        boundaryEdges(i, 1) = (i + 1) % vertexCount;
    }
    
    // No interior holes
    Eigen::Matrix<double, Eigen::Dynamic, 2> holes(0, 2);
    
    // Triangle flags: p = PSLG mode (respects boundary segments), Q = quiet, z = zero-indexed
    const std::string triangleFlags = "pQz";
    
    Eigen::Matrix<double, Eigen::Dynamic, 2> outputVertices;
    Eigen::Matrix<int, Eigen::Dynamic, 3> outputFaces;
    
    igl::triangle::triangulate(inputVertices, boundaryEdges, holes, triangleFlags,
                               outputVertices, outputFaces);
    
    // Convert face matrix to flat index array
    Indices triangleIndices;
    triangleIndices.reserve(static_cast<size_t>(outputFaces.rows()) * 3);
    
    for (int faceIndex = 0; faceIndex < outputFaces.rows(); ++faceIndex) {
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 0)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 1)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 2)));
    }

    return triangleIndices;
}

Indices ConstrainedDelaunayWithEdgeFlips(const std::vector<Vertex>& vertices) {
    const int vertexCount = static_cast<int>(vertices.size());

    // Prepare vertex matrix for libigl
    Eigen::Matrix<double, Eigen::Dynamic, 2> inputVertices(vertexCount, 2);
    for (int i = 0; i < vertexCount; ++i) {
        inputVertices(i, 0) = vertices[i].position.x;
        inputVertices(i, 1) = vertices[i].position.y;
    }

    // Define boundary edges (closed polygon)
    Eigen::Matrix<int, Eigen::Dynamic, 2> boundaryEdges(vertexCount, 2);
    for (int i = 0; i < vertexCount; ++i) {
        boundaryEdges(i, 0) = i;
        boundaryEdges(i, 1) = (i + 1) % vertexCount;
    }

    // No interior holes
    Eigen::Matrix<double, Eigen::Dynamic, 2> holes(0, 2);

    // Triangle flags: p = PSLG mode (respects boundary segments), Q = quiet, z = zero-indexed
    const std::string triangleFlags = "pQz";

    Eigen::Matrix<double, Eigen::Dynamic, 2> outputVertices;
    Eigen::Matrix<int, Eigen::Dynamic, 3> outputFaces;

    igl::triangle::triangulate(inputVertices, boundaryEdges, holes, triangleFlags,
                               outputVertices, outputFaces);

    // Convert face matrix to flat index array
    Indices triangleIndices;
    triangleIndices.reserve(static_cast<size_t>(outputFaces.rows()) * 3);

    for (int faceIndex = 0; faceIndex < outputFaces.rows(); ++faceIndex) {
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 0)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 1)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 2)));
    }

    triangleIndices = OptimizeByMinLengthFlips_PQ(vertices, triangleIndices);
    return triangleIndices;
}

} // namespace Triangulation
