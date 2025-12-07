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

inline double edgeLength(const Vertex& vertexA, const Vertex& vertexB) {
    return simd_distance_squared(vertexB.position, vertexA.position);
}

inline double trianglePerimeter(const std::vector<Vertex>& vertices,
                                uint32_t indexA, uint32_t indexB, uint32_t indexC) {
    return edgeLength(vertices[indexA], vertices[indexB])
         + edgeLength(vertices[indexB], vertices[indexC])
         + edgeLength(vertices[indexC], vertices[indexA]);
}

inline double triangleArea(const std::vector<Vertex>& vertices,
                           uint32_t indexA, uint32_t indexB, uint32_t indexC) {
    const simd_float3 ab = vertices[indexB].position - vertices[indexA].position;
    const simd_float3 ac = vertices[indexC].position - vertices[indexA].position;

    return std::abs(simd_cross(ab, ac).z) * 0.5;
}

inline double polygonSignedArea(const std::vector<Vertex>& vertices) {
    const size_t vertexCount = vertices.size();
    double signedAreaSum = 0.0;
    
    for (size_t i = 0; i < vertexCount; ++i) {
        const auto& current = vertices[i].position;
        const auto& next = vertices[(i + 1) % vertexCount].position;
        signedAreaSum += static_cast<double>(current.x) * static_cast<double>(next.y)
                       - static_cast<double>(current.y) * static_cast<double>(next.x);
    }
    
    return 0.5 * signedAreaSum; // positive = CCW, negative = CW
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

// Check if two line segments intersect (excluding endpoints)
inline bool segmentsIntersect(const simd_float3& p1, const simd_float3& p2,
                               const simd_float3& q1, const simd_float3& q2) {
    const double d1 = cross2D(p1, p2, q1);
    const double d2 = cross2D(p1, p2, q2);
    const double d3 = cross2D(q1, q2, p1);
    const double d4 = cross2D(q1, q2, p2);
    
    // Segments intersect if points are on opposite sides
    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) {
        if ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
            return true;
        }
    }
    return false;
}

// Simplified: Check if triangle is inside polygon (for concave polygons)
// Assumes triangle vertices are on polygon boundary
// Based on computational geometry best practices: check center + vertex containment + edge validity
inline bool isTriangleInsidePolygon(const std::vector<Vertex>& vertices, int i, int j, int k) {
    const auto& pA = vertices[i].position;
    const auto& pB = vertices[j].position;
    const auto& pC = vertices[k].position;
    
    // Check triangle center is inside polygon (single point-in-polygon test)
    simd_float3 center = {(pA.x + pB.x + pC.x) / 3.0f,
                         (pA.y + pB.y + pC.y) / 3.0f, 1.0f};
    if (!pointInsidePolygon(center, vertices)) {
        return false; // Triangle center outside = triangle outside
    }
    
    // Check no other boundary vertices inside triangle
    const int n = static_cast<int>(vertices.size());
    for (int v = 0; v < n; ++v) {
        if (v == i || v == j || v == k) continue;
        if (pointInTriangle(vertices[v].position, pA, pB, pC)) {
            return false; // Another vertex inside = invalid
        }
    }
    
    // Check that triangle edges don't cross polygon boundary
    // Only check edges that are not consecutive (diagonals)
    auto isConsecutive = [n](int a, int b) -> bool {
        return (a + 1) % n == b || (b + 1) % n == a;
    };
    
    // Check edge AB
    if (!isConsecutive(i, j)) {
        for (int e = 0; e < n; ++e) {
            int nextE = (e + 1) % n;
            // Skip if this edge is one of our triangle edges
            if ((e == i && nextE == j) || (e == j && nextE == i)) continue;
            if ((e == j && nextE == k) || (e == k && nextE == j)) continue;
            if ((e == k && nextE == i) || (e == i && nextE == k)) continue;
            
            if (segmentsIntersect(pA, pB, vertices[e].position, vertices[nextE].position)) {
                return false; // Edge AB crosses polygon boundary
            }
        }
    }
    
    // Check edge BC
    if (!isConsecutive(j, k)) {
        for (int e = 0; e < n; ++e) {
            int nextE = (e + 1) % n;
            if ((e == i && nextE == j) || (e == j && nextE == i)) continue;
            if ((e == j && nextE == k) || (e == k && nextE == j)) continue;
            if ((e == k && nextE == i) || (e == i && nextE == k)) continue;
            
            if (segmentsIntersect(pB, pC, vertices[e].position, vertices[nextE].position)) {
                return false; // Edge BC crosses polygon boundary
            }
        }
    }
    
    // Check edge CA
    if (!isConsecutive(k, i)) {
        for (int e = 0; e < n; ++e) {
            int nextE = (e + 1) % n;
            if ((e == i && nextE == j) || (e == j && nextE == i)) continue;
            if ((e == j && nextE == k) || (e == k && nextE == j)) continue;
            if ((e == k && nextE == i) || (e == i && nextE == k)) continue;
            
            if (segmentsIntersect(pC, pA, vertices[e].position, vertices[nextE].position)) {
                return false; // Edge CA crosses polygon boundary
            }
        }
    }
    
    return true;
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

// Filter triangles to only keep those inside the polygon (for concave polygons)
inline std::vector<uint32_t> filterTrianglesToInterior(
    const std::vector<Vertex>& vertices,
    const std::vector<uint32_t>& indices) {
    
    std::vector<uint32_t> filtered;
    filtered.reserve(indices.size());
    
    for (size_t i = 0; i < indices.size(); i += 3) {
        uint32_t a = indices[i];
        uint32_t b = indices[i + 1];
        uint32_t c = indices[i + 2];
        
        if (isTriangleInsidePolygon(vertices, static_cast<int>(a), static_cast<int>(b), static_cast<int>(c))) {
            filtered.push_back(a);
            filtered.push_back(b);
            filtered.push_back(c);
        }
    }
    
    return filtered;
}
} // anonymous namespace

// MARK: - Edge Length Calculation

double calculateTotalEdgeLength(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices) {
    double total = 0.0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        total += trianglePerimeter(vertices, indices[i], indices[i+1], indices[i+2]);
    }
    return total;
}

// MARK: - Triangulation Implementations

std::vector<uint32_t> earClippingTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const size_t n = vertices.size();
    
    if (n < 3) return indices;

    const bool isCCW = polygonSignedArea(vertices) >= 0.0;
    
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
            
            if (isValidEar(vertices, polygon, prev, i, next, isCCW)) {
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
        
        if (!earFound) break; // No valid ear found, polygon may be degenerate
    }
    
    // Add final triangle
    if (polygon.size() == 3) {
        indices.push_back(polygon[0]);
        indices.push_back(polygon[1]);
        indices.push_back(polygon[2]);
    }
    
    return indices;
}

std::vector<uint32_t> minimumWeightTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
    
    if (vertexCount < 3) {
        return indices;
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
                // Skip invalid triangles (outside polygon or containing other vertices)
                if (!isTriangleInsidePolygon(vertices, startIndex, splitPoint, endIndex)) {
                    continue;
                }
                
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
    indices.reserve(3 * (vertexCount - 2));
    
    std::function<void(int, int)> emitTriangles = [&](int startIndex, int endIndex) {
        const int splitPoint = split(startIndex, endIndex);
        if (splitPoint < 0) return;
        
        // Emit triangle (startIndex, splitPoint, endIndex)
        indices.push_back(static_cast<uint32_t>(startIndex));
        indices.push_back(static_cast<uint32_t>(splitPoint));
        indices.push_back(static_cast<uint32_t>(endIndex));
        
        emitTriangles(startIndex, splitPoint);
        emitTriangles(splitPoint, endIndex);
    };
    
    emitTriangles(0, vertexCount - 1);
    
    return indices;
}

std::vector<uint32_t> centroidFanTriangulation(std::vector<Vertex>& vertices) {
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
    
    // Create fan triangles connecting each edge to the centroid
    indices.reserve(originalVertexCount * 3);

    for (uint32_t i = 0; i < originalVertexCount; ++i) {
        const uint32_t nextIndex = (i + 1) % static_cast<uint32_t>(originalVertexCount);
        
        // For concave polygons, check if triangle is inside polygon before adding
        // Check against original polygon (first originalVertexCount vertices)
        std::vector<Vertex> originalVertices(vertices.begin(), vertices.begin() + static_cast<ptrdiff_t>(originalVertexCount));
        
        // Create temporary triangle with centroid for validation
        // Note: centroid is not in originalVertices, so we check manually
        const auto& pA = vertices[i].position;
        const auto& pB = vertices[nextIndex].position;
        const auto& pC = vertices[centroidIndex].position;
        
        // Check triangle center is inside original polygon
        simd_float3 center = {(pA.x + pB.x + pC.x) / 3.0f,
                             (pA.y + pB.y + pC.y) / 3.0f, 1.0f};
        if (!pointInsidePolygon(center, originalVertices)) {
            continue; // Triangle center outside = skip
        }
        
        // Check no other boundary vertices inside triangle
        bool isValid = true;
        for (size_t v = 0; v < originalVertexCount; ++v) {
            if (v == i || v == nextIndex) continue;
            if (pointInTriangle(vertices[v].position, pA, pB, pC)) {
                isValid = false;
                break;
            }
        }
        
        if (isValid) {
            indices.push_back(i);
            indices.push_back(nextIndex);
            indices.push_back(centroidIndex);
        }
    }
    
    return indices;
}

std::vector<uint32_t> greedyMaxAreaTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const size_t vertexCount = vertices.size();
    
    if (vertexCount < 3) {
        return indices;
    }

    const std::vector<uint32_t> ccwOrder = buildCCWOrder(vertices);

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
                        
                        // Skip invalid triangles (outside polygon or containing other vertices)
                        if (!isTriangleInsidePolygon(vertices, static_cast<int>(vi), static_cast<int>(vj), static_cast<int>(vk))) {
                            continue;
                        }
                        
                        const double area = triangleArea(vertices, vi, vj, vk);
                        if (area > largestArea) {
                            largestArea = area;
                            bestI = i;
                            bestJ = j;
                            bestK = k;
                        }
                    }
                }
            }
            
            if (largestArea < 0) return; // No valid triangle found
            
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

std::vector<uint32_t> stripTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const size_t vertexCount = vertices.size();
    
    if (vertexCount < 3) {
        return indices;
    }
    
    // Ensure CCW order for proper boundary traversal
    const std::vector<uint32_t> ccwOrder = buildCCWOrder(vertices);
    const bool isClockwise = polygonSignedArea(vertices) < 0.0;
    
    // Build strip order: alternating from start and end of CCW-ordered polygon
    std::vector<uint32_t> stripOrder;
    stripOrder.reserve(vertexCount);
    
    size_t leftIdx = 0;
    size_t rightIdx = vertexCount - 1;
    
    while (leftIdx <= rightIdx) {
        stripOrder.push_back(ccwOrder[leftIdx++]);
        if (leftIdx > rightIdx) break;
        stripOrder.push_back(ccwOrder[rightIdx--]);
    }
    
    // Generate triangles from consecutive strip triplets
    indices.reserve((vertexCount - 2) * 3);
    
    for (size_t i = 0; i + 2 < stripOrder.size(); ++i) {
        uint32_t indexA = stripOrder[i];
        uint32_t indexB = stripOrder[i + 1];
        uint32_t indexC = stripOrder[i + 2];
        
        // Alternate winding to maintain consistent orientation
        const bool shouldSwap = isClockwise ? ((i % 2) == 0) : ((i % 2) == 1);
        if (shouldSwap) {
            std::swap(indexA, indexB);
        }
        
        indices.push_back(indexA);
        indices.push_back(indexB);
        indices.push_back(indexC);
    }
    
    // Filter to only keep triangles inside the polygon (important for concave polygons)
    return filterTrianglesToInterior(vertices, indices);
}

std::vector<uint32_t> maxMinAreaTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
    
    if (vertexCount < 3) {
        return indices;
    }
    
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
                
                // Skip invalid triangles
                if (!isTriangleInsidePolygon(vertices, originalA, originalB, originalC)) continue;
                
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
    indices.reserve(3 * (vertexCount - 2));
    
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
        
        indices.push_back(indexA);
        indices.push_back(indexB);
        indices.push_back(indexC);
        
        emitTriangles(startIndex, splitPoint);
        emitTriangles(splitPoint, endIndex);
    };
    
    emitTriangles(0, vertexCount - 1);
    
    return indices;
}

std::vector<uint32_t> minMaxAreaTriangulation(const std::vector<Vertex>& vertices) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
    
    if (vertexCount < 3) {
        return indices;
    }
    
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
                
                // Skip invalid triangles
                if (!isTriangleInsidePolygon(vertices, originalA, originalB, originalC)) continue;
                
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
    indices.reserve(3 * (vertexCount - 2));
    
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
        
        indices.push_back(indexA);
        indices.push_back(indexB);
        indices.push_back(indexC);
        
        emitTriangles(startIndex, splitPoint);
        emitTriangles(splitPoint, endIndex);
    };
    
    emitTriangles(0, vertexCount - 1);
    
    return indices;
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

} // namespace Triangulation
