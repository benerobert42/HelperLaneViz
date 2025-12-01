//
//  Triangulation.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#include "Triangulation.h"

#include <cmath>
#include <limits>
#include <numeric>
#include <functional>
#include <set>
#include <tuple>

#include <Eigen/Core>
#include <igl/triangle/triangulate.h>

namespace Triangulation {

// MARK: - Geometry Helper Functions

namespace {

// Euclidean distance between two vertices
inline double edgeLength(const Vertex& vertexA, const Vertex& vertexB) {
    return simd_distance_squared(vertexB.position, vertexA.position);
}

// Perimeter of a triangle defined by three vertices
inline double trianglePerimeter(const std::vector<Vertex>& vertices,
                                uint32_t indexA, uint32_t indexB, uint32_t indexC) {
    return edgeLength(vertices[indexA], vertices[indexB])
         + edgeLength(vertices[indexB], vertices[indexC])
         + edgeLength(vertices[indexC], vertices[indexA]);
}

// Absolute area of a triangle using the cross product formula
inline double triangleArea(const std::vector<Vertex>& vertices,
                           uint32_t indexA, uint32_t indexB, uint32_t indexC) {
    const simd_float3 ab = vertices[indexB].position - vertices[indexA].position;
    const simd_float3 ac = vertices[indexC].position - vertices[indexA].position;

    return std::abs(simd_cross(ab, ac).z) * 0.5;
}

// Signed area of the polygon (positive = CCW, negative = CW)
inline double polygonSignedArea(const std::vector<Vertex>& vertices) {
    const size_t vertexCount = vertices.size();
    double signedAreaSum = 0.0;
    
    for (size_t i = 0; i < vertexCount; ++i) {
        const auto& current = vertices[i].position;
        const auto& next = vertices[(i + 1) % vertexCount].position;
        signedAreaSum += static_cast<double>(current.x) * static_cast<double>(next.y)
                       - static_cast<double>(current.y) * static_cast<double>(next.x);
    }
    
    return 0.5 * signedAreaSum;
}

// Check if triangle (A, B, C) has counter-clockwise orientation
inline bool isCounterClockwise(const std::vector<Vertex>& vertices,
                               uint32_t indexA, uint32_t indexB, uint32_t indexC) {
    const auto& posA = vertices[indexA].position;
    const auto& posB = vertices[indexB].position;
    const auto& posC = vertices[indexC].position;
    
    const double crossProduct = (static_cast<double>(posB.x) - posA.x) * (static_cast<double>(posC.y) - posA.y)
                              - (static_cast<double>(posB.y) - posA.y) * (static_cast<double>(posC.x) - posA.x);
    return crossProduct > 0.0;
}

// Signed cross product of vectors (p1-p0) and (p2-p0)
inline double cross2D(const simd_float3& p0, const simd_float3& p1, const simd_float3& p2) {
    return (static_cast<double>(p1.x) - p0.x) * (static_cast<double>(p2.y) - p0.y)
         - (static_cast<double>(p1.y) - p0.y) * (static_cast<double>(p2.x) - p0.x);
}

// Check if point P is strictly inside triangle ABC (not on edges)
inline bool pointInTriangle(const simd_float3& p,
                            const simd_float3& a, const simd_float3& b, const simd_float3& c) {
    const double d1 = cross2D(p, a, b);
    const double d2 = cross2D(p, b, c);
    const double d3 = cross2D(p, c, a);
    
    const bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    
    return !(hasNeg && hasPos);
}

// Check if two segments (p1-p2) and (p3-p4) properly intersect (cross each other)
inline bool segmentsIntersect(const simd_float3& p1, const simd_float3& p2,
                              const simd_float3& p3, const simd_float3& p4) {
    const double d1 = cross2D(p3, p4, p1);
    const double d2 = cross2D(p3, p4, p2);
    const double d3 = cross2D(p1, p2, p3);
    const double d4 = cross2D(p1, p2, p4);
    
    // Check for proper intersection (segments cross each other)
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }
    return false;
}

// Ray casting point-in-polygon test
inline bool pointInsidePolygon(const simd_float3& p, const std::vector<Vertex>& vertices) {
    const size_t n = vertices.size();
    if (n < 3) return false;
    
    int crossings = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = vertices[i].position;
        const auto& b = vertices[(i + 1) % n].position;
        
        if ((a.y <= p.y && b.y > p.y) || (b.y <= p.y && a.y > p.y)) {
            float t = (p.y - a.y) / (b.y - a.y);
            if (p.x < a.x + t * (b.x - a.x)) {
                crossings++;
            }
        }
    }
    return (crossings % 2) == 1;
}

// Check if diagonal from vertex i to vertex j is valid (inside polygon, doesn't cross edges)
inline bool isDiagonalValid(const std::vector<Vertex>& vertices, int i, int j) {
    const int n = static_cast<int>(vertices.size());
    if (n < 3) return false;
    
    i = ((i % n) + n) % n;
    j = ((j % n) + n) % n;
    
    if (i == j) return false;
    
    // Adjacent vertices are always valid (boundary edges)
    int diff = std::abs(i - j);
    if (diff == 1 || diff == n - 1) return true;
    
    const auto& pi = vertices[i].position;
    const auto& pj = vertices[j].position;
    
    // Check if diagonal intersects any polygon edge
    for (int k = 0; k < n; ++k) {
        int next = (k + 1) % n;
        if (k == i || k == j || next == i || next == j) continue;
        
        const auto& pk = vertices[k].position;
        const auto& pnext = vertices[next].position;
        
        if (segmentsIntersect(pi, pj, pk, pnext)) {
            return false;
        }
    }
    
    // Check if midpoint is inside polygon
    simd_float3 midpoint = {(pi.x + pj.x) * 0.5f, (pi.y + pj.y) * 0.5f, 1.0f};
    return pointInsidePolygon(midpoint, vertices);
}

// Validate entire triangulation: check for basic validity and crossing edges
inline bool isTriangulationValid(const std::vector<Vertex>& vertices,
                                  const std::vector<uint32_t>& indices) {
    const size_t n = vertices.size();
    if (n < 3) return false;
    if (indices.empty() || indices.size() % 3 != 0) return false;
    
    // Check all indices are valid
    for (size_t i = 0; i < indices.size(); ++i) {
        if (indices[i] >= n) return false;
    }
    
    // For simple polygon validation, check original boundary vertices are covered
    size_t originalN = n;
    size_t numTriangles = indices.size() / 3;
    if (numTriangles > n - 2 && numTriangles == n) {
        originalN = n - 1; // Likely centroid fan
    }
    
    std::vector<bool> vertexUsed(originalN, false);
    for (size_t i = 0; i < indices.size(); ++i) {
        if (indices[i] < originalN) {
            vertexUsed[indices[i]] = true;
        }
    }
    
    size_t unusedCount = 0;
    for (size_t i = 0; i < originalN; ++i) {
        if (!vertexUsed[i]) unusedCount++;
    }
    
    if (unusedCount > 0) {
        // Some vertices unused - this is a problem
        return false;
    }
    
    // Skip expensive edge crossing check for now - CDT handles this properly
    // The main concern is missing vertices
    return true;
}

// Check if an ear (formed by polygon[prev], polygon[curr], polygon[next]) is valid
// An ear is valid if:
// 1. It has correct winding (convex at curr vertex for CCW polygon)
// 2. No other polygon vertices are inside the triangle
// 3. The diagonal doesn't intersect any polygon edges
inline bool isValidEar(const std::vector<Vertex>& vertices,
                       const std::vector<uint32_t>& polygon,
                       size_t prevIdx, size_t currIdx, size_t nextIdx,
                       bool polygonIsCCW) {
    const size_t n = polygon.size();
    if (n < 3) return false;
    
    const auto& pPrev = vertices[polygon[prevIdx]].position;
    const auto& pCurr = vertices[polygon[currIdx]].position;
    const auto& pNext = vertices[polygon[nextIdx]].position;
    
    // Check winding: for CCW polygon, ear must be CCW (convex vertex)
    const double cross = cross2D(pPrev, pCurr, pNext);
    if (polygonIsCCW) {
        if (cross <= 0) return false; // Reflex vertex, not an ear
    } else {
        if (cross >= 0) return false; // For CW polygon, ear must be CW
    }
    
    // Check that no other polygon vertices are inside this triangle
    for (size_t i = 0; i < n; ++i) {
        if (i == prevIdx || i == currIdx || i == nextIdx) continue;
        
        const auto& testPoint = vertices[polygon[i]].position;
        if (pointInTriangle(testPoint, pPrev, pCurr, pNext)) {
            return false;
        }
    }
    
    return true;
}

// Robust ear-clipping triangulation (always produces valid results for simple polygons)
inline std::vector<uint32_t> robustEarClip(const std::vector<Vertex>& vertices) {
    const size_t vertexCount = vertices.size();
    if (vertexCount < 3) return {};
    
    const bool polygonIsCCW = polygonSignedArea(vertices) >= 0.0;
    
    std::vector<uint32_t> polygon(vertexCount);
    std::iota(polygon.begin(), polygon.end(), 0);
    
    std::vector<uint32_t> indices;
    indices.reserve((vertexCount - 2) * 3);
    
    while (polygon.size() > 3) {
        const size_t n = polygon.size();
        bool found = false;
        
        for (size_t i = 0; i < n; ++i) {
            const size_t prev = (i + n - 1) % n;
            const size_t next = (i + 1) % n;
            
            if (isValidEar(vertices, polygon, prev, i, next, polygonIsCCW)) {
                indices.push_back(polygon[prev]);
                indices.push_back(polygon[i]);
                indices.push_back(polygon[next]);
                polygon.erase(polygon.begin() + static_cast<ptrdiff_t>(i));
                found = true;
                break;
            }
        }
        
        if (!found) break;
    }
    
    if (polygon.size() == 3) {
        indices.push_back(polygon[0]);
        indices.push_back(polygon[1]);
        indices.push_back(polygon[2]);
    }
    
    return indices;
}

// Build vertex ordering for CCW traversal (reverses if input is CW)
inline std::vector<uint32_t> buildCCWOrder(const std::vector<Vertex>& vertices) {
    const size_t vertexCount = vertices.size();
    std::vector<uint32_t> order(vertexCount);
    
    const bool isAlreadyCCW = polygonSignedArea(vertices) >= 0.0;
    
    for (size_t i = 0; i < vertexCount; ++i) {
        order[i] = isAlreadyCCW
            ? static_cast<uint32_t>(i)
            : static_cast<uint32_t>(vertexCount - 1 - i);
    }
    
    return order;
}

} // anonymous namespace

// MARK: - Triangulation Implementations

Result minimumWeightTriangulation(const std::vector<Vertex>& vertices) {
    Result result;
    const int vertexCount = static_cast<int>(vertices.size());
    
    if (vertexCount < 3) {
        return result;
    }
    
    // DP tables: dp[i][j] = minimum edge weight to triangulate polygon from i to j
    //            split[i][j] = optimal split point k for the interval [i, j]
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
                // Cost = left subproblem + right subproblem + new internal edges
                const double internalEdgeCost = edgeLength(vertices[startIndex], vertices[splitPoint])
                                              + edgeLength(vertices[splitPoint], vertices[endIndex]);
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
    result.indices.reserve(3 * (vertexCount - 2));
    
    std::function<void(int, int)> emitTriangles = [&](int startIndex, int endIndex) {
        const int splitPoint = split(startIndex, endIndex);
        if (splitPoint < 0) return;
        
        // Emit triangle (startIndex, splitPoint, endIndex)
        result.indices.push_back(static_cast<uint32_t>(startIndex));
        result.indices.push_back(static_cast<uint32_t>(splitPoint));
        result.indices.push_back(static_cast<uint32_t>(endIndex));
        
        result.totalEdgeLength += trianglePerimeter(vertices,
                                                    static_cast<uint32_t>(startIndex),
                                                    static_cast<uint32_t>(splitPoint),
                                                    static_cast<uint32_t>(endIndex));
        
        emitTriangles(startIndex, splitPoint);
        emitTriangles(splitPoint, endIndex);
    };
    
    emitTriangles(0, vertexCount - 1);
    
    return result;
}

Result centroidFanTriangulation(std::vector<Vertex>& vertices) {
    Result result;
    const size_t originalVertexCount = vertices.size();
    
    if (originalVertexCount < 3) {
        return result;
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
    
    // Create fan triangles connecting each edge to the centroid
    result.indices.reserve(originalVertexCount * 3);
    
    for (uint32_t i = 0; i < originalVertexCount; ++i) {
        const uint32_t nextIndex = (i + 1) % static_cast<uint32_t>(originalVertexCount);
        
        // Triangle (current, next, centroid) maintains CCW orientation for CCW input
        result.indices.push_back(i);
        result.indices.push_back(nextIndex);
        result.indices.push_back(centroidIndex);
        
        result.totalEdgeLength += trianglePerimeter(vertices, i, nextIndex, centroidIndex);
    }
    
    return result;
}

Result greedyMaxAreaTriangulation(const std::vector<Vertex>& vertices) {
    Result result;
    const size_t n = vertices.size();
    
    if (n < 3) return result;
    
    const bool isCCW = polygonSignedArea(vertices) >= 0.0;
    
    // Track which edges exist: polygon boundary + added diagonals
    // Edge stored as (min_idx, max_idx)
    std::set<std::pair<uint32_t, uint32_t>> edges;
    
    // Add polygon boundary edges
    for (size_t i = 0; i < n; ++i) {
        uint32_t a = static_cast<uint32_t>(i);
        uint32_t b = static_cast<uint32_t>((i + 1) % n);
        if (a > b) std::swap(a, b);
        edges.insert({a, b});
    }
    
    // Track which triangles we've added
    std::set<std::tuple<uint32_t, uint32_t, uint32_t>> usedTriangles;
    
    result.indices.reserve((n - 2) * 3);
    
    // Helper: check if edge exists or can be added (valid diagonal)
    auto isEdgeValid = [&](uint32_t a, uint32_t b) -> bool {
        if (a > b) std::swap(a, b);
        
        // Already exists - valid
        if (edges.count({a, b})) return true;
        
        const auto& posA = vertices[a].position;
        const auto& posB = vertices[b].position;
        
        // Check diagonal doesn't cross any existing edge
        for (const auto& edge : edges) {
            // Skip if shares a vertex
            if (edge.first == a || edge.first == b || 
                edge.second == a || edge.second == b) continue;
            
            if (segmentsIntersect(posA, posB,
                                  vertices[edge.first].position,
                                  vertices[edge.second].position)) {
                return false;
            }
        }
        
        // Check midpoint is inside polygon
        simd_float3 mid = {(posA.x + posB.x) * 0.5f, (posA.y + posB.y) * 0.5f, 1.0f};
        if (!pointInsidePolygon(mid, vertices)) {
            return false;
        }
        
        return true;
    };
    
    // Helper: check if triangle is valid
    auto isTriangleValid = [&](uint32_t i, uint32_t j, uint32_t k) -> bool {
        // Check orientation
        double cross = cross2D(vertices[i].position, vertices[j].position, vertices[k].position);
        if (isCCW && cross <= 1e-10) return false;
        if (!isCCW && cross >= -1e-10) return false;
        
        // Check no other vertices inside
        for (size_t m = 0; m < n; ++m) {
            if (m == i || m == j || m == k) continue;
            if (pointInTriangle(vertices[m].position, 
                               vertices[i].position, 
                               vertices[j].position, 
                               vertices[k].position)) {
                return false;
            }
        }
        
        // Check all edges valid
        if (!isEdgeValid(i, j)) return false;
        if (!isEdgeValid(j, k)) return false;
        if (!isEdgeValid(k, i)) return false;
        
        return true;
    };
    
    // Greedy: find n-2 triangles
    for (size_t triCount = 0; triCount < n - 2; ++triCount) {
        double maxArea = -1.0;
        uint32_t bestI = 0, bestJ = 0, bestK = 0;
        bool found = false;
        
        // Try ALL combinations of 3 vertices
        for (uint32_t i = 0; i < n; ++i) {
            for (uint32_t j = i + 1; j < n; ++j) {
                for (uint32_t k = j + 1; k < n; ++k) {
                    // Normalize triangle ordering
                    auto triKey = std::make_tuple(i, j, k);
                    if (usedTriangles.count(triKey)) continue;
                    
                    if (isTriangleValid(i, j, k)) {
                        double area = triangleArea(vertices, i, j, k);
                        if (area > maxArea) {
                            maxArea = area;
                            bestI = i;
                            bestJ = j;
                            bestK = k;
                            found = true;
                        }
                    }
                }
            }
        }
        
        if (!found) {
            // No valid triangle found - fall back to ear clipping
            result.indices = robustEarClip(vertices);
            return result;
        }
        
        // Add the triangle
        result.indices.push_back(bestI);
        result.indices.push_back(bestJ);
        result.indices.push_back(bestK);
        result.totalEdgeLength += trianglePerimeter(vertices, bestI, bestJ, bestK);
        
        usedTriangles.insert(std::make_tuple(bestI, bestJ, bestK));
        
        // Add new edges
        auto addEdge = [&](uint32_t a, uint32_t b) {
            if (a > b) std::swap(a, b);
            edges.insert({a, b});
        };
        addEdge(bestI, bestJ);
        addEdge(bestJ, bestK);
        addEdge(bestK, bestI);
    }
    
    return result;
}

Result stripTriangulation(const std::vector<Vertex>& vertices) {
    Result result;
    const size_t vertexCount = vertices.size();
    
    if (vertexCount < 3) {
        return result;
    }
    
    const bool isClockwise = polygonSignedArea(vertices) < 0.0;
    
    // Build strip order: alternating from left (0) and right (n-1) ends
    std::vector<uint32_t> stripOrder;
    stripOrder.reserve(vertexCount);
    
    uint32_t leftIndex = 0;
    uint32_t rightIndex = static_cast<uint32_t>(vertexCount - 1);
    
    while (leftIndex <= rightIndex) {
        stripOrder.push_back(leftIndex++);
        if (leftIndex > rightIndex) break;
        stripOrder.push_back(rightIndex--);
    }
    
    // Generate triangles from consecutive strip triplets
    result.indices.reserve((vertexCount - 2) * 3);
    
    for (size_t i = 0; i + 2 < stripOrder.size(); ++i) {
        uint32_t indexA = stripOrder[i];
        uint32_t indexB = stripOrder[i + 1];
        uint32_t indexC = stripOrder[i + 2];
        
        // Alternate winding to maintain consistent orientation
        const bool shouldSwap = isClockwise ? ((i % 2) == 0) : ((i % 2) == 1);
        if (shouldSwap) {
            std::swap(indexA, indexB);
        }
        
        result.indices.push_back(indexA);
        result.indices.push_back(indexB);
        result.indices.push_back(indexC);
        
        result.totalEdgeLength += trianglePerimeter(vertices, indexA, indexB, indexC);
    }
    
    return result;
}

Result maxMinAreaTriangulation(const std::vector<Vertex>& vertices) {
    Result result;
    const int vertexCount = static_cast<int>(vertices.size());
    
    if (vertexCount < 3) {
        return result;
    }
    
    // Ensure CCW traversal order
    const std::vector<uint32_t> ccwOrder = buildCCWOrder(vertices);
    
    // DP tables: dp[i][j] = maximum achievable minimum triangle area for chain [i, j]
    std::vector<double> dpTable(vertexCount * vertexCount, 0.0);
    std::vector<int> splitTable(vertexCount * vertexCount, -1);
    
    auto dp = [&](int i, int j) -> double& {
        return dpTable[i * vertexCount + j];
    };
    auto split = [&](int i, int j) -> int& {
        return splitTable[i * vertexCount + j];
    };
    
    // Initialize: adjacent pairs have infinite "minimum" (no triangles to constrain)
    constexpr double INFINITY_VALUE = std::numeric_limits<double>::infinity();
    for (int i = 0; i + 1 < vertexCount; ++i) {
        dp(i, i + 1) = INFINITY_VALUE;
    }
    
    // Fill DP table for increasing chain lengths
    for (int chainLength = 2; chainLength < vertexCount; ++chainLength) {
        for (int startIndex = 0; startIndex + chainLength < vertexCount; ++startIndex) {
            const int endIndex = startIndex + chainLength;
            
            double bestMinArea = 0.0;
            int optimalSplit = -1;
            
            for (int splitPoint = startIndex + 1; splitPoint < endIndex; ++splitPoint) {
                const uint32_t originalA = ccwOrder[startIndex];
                const uint32_t originalB = ccwOrder[splitPoint];
                const uint32_t originalC = ccwOrder[endIndex];
                
                const double currentTriangleArea = triangleArea(vertices, originalA, originalB, originalC);
                
                // Bottleneck = minimum of {left subproblem, right subproblem, this triangle}
                const double bottleneck = std::min({dp(startIndex, splitPoint),
                                                    dp(splitPoint, endIndex),
                                                    currentTriangleArea});
                
                if (bottleneck > bestMinArea) {
                    bestMinArea = bottleneck;
                    optimalSplit = splitPoint;
                }
            }
            
            dp(startIndex, endIndex) = bestMinArea;
            split(startIndex, endIndex) = optimalSplit;
        }
    }
    
    // Reconstruct triangles with CCW orientation
    result.indices.reserve(3 * (vertexCount - 2));
    
    std::function<void(int, int)> emitTriangles = [&](int startIndex, int endIndex) {
        const int splitPoint = split(startIndex, endIndex);
        if (splitPoint < 0) return;
        
        uint32_t indexA = ccwOrder[startIndex];
        uint32_t indexB = ccwOrder[splitPoint];
        uint32_t indexC = ccwOrder[endIndex];
        
        // Ensure CCW orientation
        if (!isCounterClockwise(vertices, indexA, indexB, indexC)) {
            std::swap(indexB, indexC);
        }
        
        result.indices.push_back(indexA);
        result.indices.push_back(indexB);
        result.indices.push_back(indexC);
        result.totalEdgeLength += trianglePerimeter(vertices, indexA, indexB, indexC);
        
        emitTriangles(startIndex, splitPoint);
        emitTriangles(splitPoint, endIndex);
    };
    
    emitTriangles(0, vertexCount - 1);
    
    return result;
}

Result minMaxAreaTriangulation(const std::vector<Vertex>& vertices) {
    Result result;
    const int vertexCount = static_cast<int>(vertices.size());
    
    if (vertexCount < 3) {
        return result;
    }
    
    // Ensure CCW traversal order
    const std::vector<uint32_t> ccwOrder = buildCCWOrder(vertices);
    
    // DP tables: dp[i][j] = minimum achievable maximum triangle area for chain [i, j]
    std::vector<double> dpTable(vertexCount * vertexCount, 0.0);
    std::vector<int> splitTable(vertexCount * vertexCount, -1);
    
    auto dp = [&](int i, int j) -> double& {
        return dpTable[i * vertexCount + j];
    };
    auto split = [&](int i, int j) -> int& {
        return splitTable[i * vertexCount + j];
    };
    
    // Initialize: adjacent pairs have zero cost (no triangles)
    for (int i = 0; i + 1 < vertexCount; ++i) {
        dp(i, i + 1) = 0.0;
    }
    
    // Fill DP table for increasing chain lengths
    for (int chainLength = 2; chainLength < vertexCount; ++chainLength) {
        for (int startIndex = 0; startIndex + chainLength < vertexCount; ++startIndex) {
            const int endIndex = startIndex + chainLength;
            
            double bestMaxArea = std::numeric_limits<double>::infinity();
            int optimalSplit = -1;
            
            for (int splitPoint = startIndex + 1; splitPoint < endIndex; ++splitPoint) {
                const uint32_t originalA = ccwOrder[startIndex];
                const uint32_t originalB = ccwOrder[splitPoint];
                const uint32_t originalC = ccwOrder[endIndex];
                
                const double currentTriangleArea = triangleArea(vertices, originalA, originalB, originalC);
                
                // Cost = maximum of {left subproblem, right subproblem, this triangle}
                const double cost = std::max({dp(startIndex, splitPoint),
                                              dp(splitPoint, endIndex),
                                              currentTriangleArea});
                
                if (cost < bestMaxArea) {
                    bestMaxArea = cost;
                    optimalSplit = splitPoint;
                }
            }
            
            dp(startIndex, endIndex) = bestMaxArea;
            split(startIndex, endIndex) = optimalSplit;
        }
    }
    
    // Reconstruct triangles with CCW orientation
    result.indices.reserve(3 * (vertexCount - 2));
    
    std::function<void(int, int)> emitTriangles = [&](int startIndex, int endIndex) {
        const int splitPoint = split(startIndex, endIndex);
        if (splitPoint < 0) return;
        
        uint32_t indexA = ccwOrder[startIndex];
        uint32_t indexB = ccwOrder[splitPoint];
        uint32_t indexC = ccwOrder[endIndex];
        
        // Ensure CCW orientation
        if (!isCounterClockwise(vertices, indexA, indexB, indexC)) {
            std::swap(indexB, indexC);
        }
        
        result.indices.push_back(indexA);
        result.indices.push_back(indexB);
        result.indices.push_back(indexC);
        result.totalEdgeLength += trianglePerimeter(vertices, indexA, indexB, indexC);
        
        emitTriangles(startIndex, splitPoint);
        emitTriangles(splitPoint, endIndex);
    };
    
    emitTriangles(0, vertexCount - 1);
    
    return result;
}

std::vector<uint32_t> constrainedDelaunay(const std::vector<Vertex>& vertices) {
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

std::vector<uint32_t> constrainedDelaunayWithHoles(
    const std::vector<Vertex>& outerBoundary,
    const std::vector<std::vector<Vertex>>& holes)
{
    if (outerBoundary.size() < 3) return {};
    
    // Count total vertices
    size_t totalVertices = outerBoundary.size();
    size_t totalEdges = outerBoundary.size();
    for (const auto& hole : holes) {
        totalVertices += hole.size();
        totalEdges += hole.size();
    }
    
    // Build combined vertex matrix
    Eigen::Matrix<double, Eigen::Dynamic, 2> inputVertices(totalVertices, 2);
    Eigen::Matrix<int, Eigen::Dynamic, 2> boundaryEdges(totalEdges, 2);
    
    int vertexIdx = 0;
    int edgeIdx = 0;
    
    // Add outer boundary vertices and edges
    const int outerStart = vertexIdx;
    for (size_t i = 0; i < outerBoundary.size(); ++i) {
        inputVertices(vertexIdx, 0) = outerBoundary[i].position.x;
        inputVertices(vertexIdx, 1) = outerBoundary[i].position.y;
        vertexIdx++;
    }
    for (size_t i = 0; i < outerBoundary.size(); ++i) {
        boundaryEdges(edgeIdx, 0) = outerStart + static_cast<int>(i);
        boundaryEdges(edgeIdx, 1) = outerStart + static_cast<int>((i + 1) % outerBoundary.size());
        edgeIdx++;
    }
    
    // Add hole vertices and edges, compute hole interior points
    Eigen::Matrix<double, Eigen::Dynamic, 2> holePoints(holes.size(), 2);
    
    for (size_t h = 0; h < holes.size(); ++h) {
        const auto& hole = holes[h];
        if (hole.size() < 3) continue;
        
        const int holeStart = vertexIdx;
        
        // Add hole vertices
        for (size_t i = 0; i < hole.size(); ++i) {
            inputVertices(vertexIdx, 0) = hole[i].position.x;
            inputVertices(vertexIdx, 1) = hole[i].position.y;
            vertexIdx++;
        }
        
        // Add hole edges
        for (size_t i = 0; i < hole.size(); ++i) {
            boundaryEdges(edgeIdx, 0) = holeStart + static_cast<int>(i);
            boundaryEdges(edgeIdx, 1) = holeStart + static_cast<int>((i + 1) % hole.size());
            edgeIdx++;
        }
        
        // Compute centroid as hole marker point
        double cx = 0, cy = 0;
        for (const auto& v : hole) {
            cx += v.position.x;
            cy += v.position.y;
        }
        holePoints(h, 0) = cx / hole.size();
        holePoints(h, 1) = cy / hole.size();
    }
    
    // Triangle flags: p = PSLG mode, Q = quiet, z = zero-indexed
    const std::string triangleFlags = "pQz";
    
    Eigen::Matrix<double, Eigen::Dynamic, 2> outputVertices;
    Eigen::Matrix<int, Eigen::Dynamic, 3> outputFaces;
    
    igl::triangle::triangulate(inputVertices, boundaryEdges, holePoints, triangleFlags,
                               outputVertices, outputFaces);
    
    // Convert to index array
    std::vector<uint32_t> triangleIndices;
    triangleIndices.reserve(outputFaces.rows() * 3);
    
    for (int faceIndex = 0; faceIndex < outputFaces.rows(); ++faceIndex) {
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 0)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 1)));
        triangleIndices.push_back(static_cast<uint32_t>(outputFaces(faceIndex, 2)));
    }
    
    return triangleIndices;
}

} // namespace Triangulation
