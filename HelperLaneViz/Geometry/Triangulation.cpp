//
//  Triangulation.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#include "Triangulation.h"

#include "TriangulationHelpers.h"

#include <numeric>

#include <Eigen/Core>
#include <igl/triangle/triangulate.h>
#include "triangulator.h"

namespace Triangulation {

double CalculateTotalEdgeLength(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices) {
    double total = 0.0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        total += Helpers::TrianglePerimeter(vertices, indices[i], indices[i+1], indices[i+2]);
    }
    return total;
}

// MARK: Simple triangulations
std::vector<uint32_t> EarClippingTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
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

std::vector<uint32_t> EarClippingTriangulation_MinDiagonalPQ(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const uint32_t n = static_cast<uint32_t>(vertices.size());
    if (n < 3)
    {
        return indices;
    }

    const bool isCCW = Helpers::PolygonSignedArea(vertices) >= 0.0;

    // Doubly-linked list on top of the polygon index list
    std::vector<uint32_t> polygon(n);
    std::iota(polygon.begin(), polygon.end(), 0);

    std::vector<int> prev(n);
    std::vector<int> next(n);
    std::vector<bool> alive(n, true);

    for (uint32_t i = 0; i < n; ++i) {
        prev[i] = (i - 1 + n) % n;
        next[i] = (i + 1) % n;
    }

    uint32_t remaining = n;

    // Versioning for lazy PQ invalidation: bump when a vertex's ear candidate must be recomputed
    std::vector<uint32_t> version(n, 0);

    struct EarCandidate {
        double score;       // lower is better
        int i;              // vertex in linked list (index into [0..n))
        uint32_t version;   // version at insertion time
    };
    struct EarCandidateGreater {
        bool operator()(const EarCandidate& a, const EarCandidate& b) const {
            return a.score > b.score; // min-heap
        }
    };

    std::priority_queue<EarCandidate, std::vector<EarCandidate>, EarCandidateGreater> pq;

    // Builds a small "polygon list" view for IsValidEar:
    // it expects a list of remaining vertex indices in order.
    // To keep diff minimal, we rebuild it only for the ear check (local cost),
    // BUT doing this would destroy performance.
    //
    // So instead: we create ONE ordered polygon list (initially [0..n-1]) and maintain it
    // via alive/prev/next, and we provide IsValidEar with that list plus indices.
    //
    // Your Helpers::IsValidEar signature is:
    // IsValidEar(vertices, polygon, prevIndexInPolygonArray, iIndexInPolygonArray, nextIndexInPolygonArray, isCCW)
    //
    // That API uses "indices into polygon array", not vertex ids.
    // For min diff, we keep a mapping from "current linked-list vertex" -> its position in
    // a dynamically rebuilt polygon array when needed.
    //
    // HOWEVER: rebuilding that array every time kills the PQ benefit.
    //
    // So: we assume (as in your current code) that IsValidEar needs the actual ordered list.
    // The best minimal-diff path is: keep an ordered vector of the current polygon and
    // update it with erase (like you do). But then PQ can’t work.
    //
    // Therefore, below we implement a small adapter: we maintain a current ordered polygon
    // vector AND a position map, and we update them incrementally when clipping an ear.
    // This keeps diff small and makes PQ viable.

    std::vector<uint32_t> poly = polygon;          // ordered list of vertex ids
    std::vector<int> pos(n);                       // vertex id -> position in poly
    for (int k = 0; k < n; ++k) pos[poly[k]] = k;

    auto pushIfEar = [&](uint32_t v) {
        if (!alive[v]) return;

        // Refresh version
        ++version[v];

        // Need current positions in poly
        const int pi = pos[v];
        if (pi < 0) return;

        const int sz = static_cast<int>(poly.size());
        if (sz < 3) return;

        const int ppos = (pi - 1 + sz) % sz;
        const int npos = (pi + 1) % sz;

        if (!Helpers::IsValidEar(vertices, poly,
                                static_cast<size_t>(ppos),
                                static_cast<size_t>(pi),
                                static_cast<size_t>(npos),
                                isCCW)) {
            return;
        }

        const uint32_t pv = poly[ppos];
        const uint32_t nv = poly[npos];

        // Objective: minimize diagonal (prev-next). Use squared or actual?
        // Use squared length if your Helpers doesn't expose squared; otherwise EdgeLength is fine.
        const double score = Helpers::EdgeLength(vertices[pv], vertices[nv]);

        pq.push(EarCandidate{score, static_cast<int>(v), version[v]});
    };

    // Initialize heap with all ears
    for (uint32_t v = 0; v < n; ++v) {
        pushIfEar(v);
    }

    indices.reserve((n - 2) * 3);

    auto eraseFromPoly = [&](int removePos) {
        const uint32_t v = poly[removePos];

        // erase from ordered vector
        poly.erase(poly.begin() + removePos);

        // update positions for elements after removePos
        for (int k = removePos; k < static_cast<int>(poly.size()); ++k) {
            pos[poly[k]] = k;
        }
        pos[v] = -1;
    };

    while (remaining > 3) {
        bool found = false;
        uint32_t earV = 0;

        while (!pq.empty()) {
            const EarCandidate cand = pq.top();
            pq.pop();

            const uint32_t v = static_cast<uint32_t>(cand.i);
            if (v >= n || !alive[v]) continue;
            if (cand.version != version[v]) continue; // stale heap entry

            // Re-validate because neighborhood could have changed
            const int pi = pos[v];
            if (pi < 0) continue;

            const int sz = static_cast<int>(poly.size());
            const int ppos = (pi - 1 + sz) % sz;
            const int npos = (pi + 1) % sz;

            if (!Helpers::IsValidEar(vertices, poly,
                                    static_cast<size_t>(ppos),
                                    static_cast<size_t>(pi),
                                    static_cast<size_t>(npos),
                                    isCCW)) {
                // it might become an ear later; refresh it now
                pushIfEar(v);
                continue;
            }

            earV = v;
            found = true;
            break;
        }

        if (!found) {
            // fallback: behave like your original code (avoid total failure)
            bool earFound = false;
            const size_t sz = poly.size();
            for (size_t i = 0; i < sz; ++i) {
                const size_t p = (i + sz - 1) % sz;
                const size_t q = (i + 1) % sz;
                if (Helpers::IsValidEar(vertices, poly, p, i, q, isCCW)) {
                    indices.push_back(poly[p]);
                    indices.push_back(poly[i]);
                    indices.push_back(poly[q]);

                    alive[poly[i]] = false;
                    eraseFromPoly(static_cast<int>(i));
                    --remaining;

                    // neighbors changed
                    if (poly.size() >= 3) {
                        const int newSz = static_cast<int>(poly.size());
                        const int leftPos  = (static_cast<int>(i) - 1 + newSz) % newSz;
                        const int rightPos = static_cast<int>(i) % newSz;
                        pushIfEar(poly[leftPos]);
                        pushIfEar(poly[rightPos]);
                    }
                    earFound = true;
                    break;
                }
            }
            if (!earFound) break;
            continue;
        }

        // Clip chosen ear
        const int pi = pos[earV];
        const int sz = static_cast<int>(poly.size());
        const int ppos = (pi - 1 + sz) % sz;
        const int npos = (pi + 1) % sz;

        const uint32_t pv = poly[ppos];
        const uint32_t v  = poly[pi];
        const uint32_t nv = poly[npos];

        indices.push_back(pv);
        indices.push_back(v);
        indices.push_back(nv);

        alive[v] = false;

        // Remove v from the ordered polygon
        eraseFromPoly(pi);
        --remaining;

        // Only neighbors can change ear status: pv and nv (now adjacent)
        if (poly.size() >= 3) {
            pushIfEar(pv);
            pushIfEar(nv);
        }
    }

    if (poly.size() == 3) {
        indices.push_back(poly[0]);
        indices.push_back(poly[1]);
        indices.push_back(poly[2]);
    }

    return indices;
}

std::vector<uint32_t> EarClippingTriangulation_Triangulator(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const size_t n = vertices.size();
    
    if (n < 3) {
        return indices;
    }
    
    // Convert Vertex format to Triangulator format (2D points)
    std::vector<std::array<double, 2>> contour;
    contour.reserve(n);
    for (const auto& v : vertices) {
        contour.push_back({static_cast<double>(v.position.x), static_cast<double>(v.position.y)});
    }
    
    // Call Triangulator's ear clipping (delaunay=false for pure ear clipping)
    Triangulator::Diagnostics diagnostics;
    auto triangles = Triangulator::triangulate1(contour, diagnostics, false, true);
    
    // Convert output format: std::vector<std::array<std::size_t,3>> -> std::vector<uint32_t>
    indices.reserve(triangles.size() * 3);
    for (const auto& tri : triangles) {
        indices.push_back(static_cast<uint32_t>(tri[0]));
        indices.push_back(static_cast<uint32_t>(tri[1]));
        indices.push_back(static_cast<uint32_t>(tri[2]));
    }
    
    return indices;
}

std::vector<uint32_t> CentroidFanTriangulation(std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
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

std::vector<uint32_t> StripTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
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
std::vector<uint32_t> MinimumWeightTriangulation(const std::vector<Vertex>& vertices,
                                                 bool shouldHandleConcave) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
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

std::vector<uint32_t> MaxMinAreaTriangulation(const std::vector<Vertex>& vertices,
                                              bool shouldHandleConcave) {
    std::vector<uint32_t> indices;
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

std::vector<uint32_t> MinMaxAreaTriangulation(const std::vector<Vertex>& vertices,
                                              bool shouldHandleConcave) {
    std::vector<uint32_t> indices;
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
std::vector<uint32_t> GreedyMaxAreaTriangulation(const std::vector<Vertex>& vertices, bool shouldHandleConcave) {
    std::vector<uint32_t> indices;
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


std::vector<uint32_t> ConstrainedDelaunayTriangulation(const std::vector<Vertex>& vertices) {
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
    std::vector<uint32_t> triangleIndices;
    triangleIndices.reserve(static_cast<size_t>(outputFaces.rows()) * 3);
    
    for (int faceIndex = 0; faceIndex < outputFaces.rows(); ++faceIndex) {
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 0)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 1)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 2)));
    }
    
    return triangleIndices;
}

} // namespace Triangulation
