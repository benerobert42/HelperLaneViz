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

#include <Eigen/Core>
#include <igl/triangle/triangulate.h>

namespace Triangulation {

// MARK: - Geometry Helper Functions

namespace {

// Euclidean distance between two vertices
inline double edgeLength(const Vertex& vertexA, const Vertex& vertexB) {
    const double deltaX = static_cast<double>(vertexB.position.x) - static_cast<double>(vertexA.position.x);
    const double deltaY = static_cast<double>(vertexB.position.y) - static_cast<double>(vertexA.position.y);
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
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
    const auto& posA = vertices[indexA].position;
    const auto& posB = vertices[indexB].position;
    const auto& posC = vertices[indexC].position;
    
    const double abX = static_cast<double>(posB.x) - static_cast<double>(posA.x);
    const double abY = static_cast<double>(posB.y) - static_cast<double>(posA.y);
    const double acX = static_cast<double>(posC.x) - static_cast<double>(posA.x);
    const double acY = static_cast<double>(posC.y) - static_cast<double>(posA.y);
    
    return std::abs(abX * acY - abY * acX) * 0.5;
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
    const size_t vertexCount = vertices.size();
    
    if (vertexCount < 3) {
        return result;
    }
    
    // Determine polygon winding
    const bool polygonIsCCW = polygonSignedArea(vertices) >= 0.0;
    
    // Create mutable polygon (indices into vertices array)
    std::vector<uint32_t> polygon(vertexCount);
    std::iota(polygon.begin(), polygon.end(), 0);
    
    result.indices.reserve((vertexCount - 2) * 3);
    
    // Ear clipping: repeatedly find the maximum-area valid ear and clip it
    while (polygon.size() > 3) {
        const size_t n = polygon.size();
        
        double maxArea = -1.0;
        size_t bestEarIdx = SIZE_MAX;
        
        // Find all valid ears and pick the one with maximum area
        for (size_t i = 0; i < n; ++i) {
            const size_t prev = (i + n - 1) % n;
            const size_t next = (i + 1) % n;
            
            if (isValidEar(vertices, polygon, prev, i, next, polygonIsCCW)) {
                const double area = triangleArea(vertices, polygon[prev], polygon[i], polygon[next]);
                if (area > maxArea) {
                    maxArea = area;
                    bestEarIdx = i;
                }
            }
        }
        
        // If no valid ear found, polygon may be degenerate - fall back to first valid ear
        if (bestEarIdx == SIZE_MAX) {
            for (size_t i = 0; i < n; ++i) {
                const size_t prev = (i + n - 1) % n;
                const size_t next = (i + 1) % n;
                
                // Relaxed check: just ensure convexity
                const auto& pPrev = vertices[polygon[prev]].position;
                const auto& pCurr = vertices[polygon[i]].position;
                const auto& pNext = vertices[polygon[next]].position;
                const double cross = cross2D(pPrev, pCurr, pNext);
                
                if ((polygonIsCCW && cross > 0) || (!polygonIsCCW && cross < 0)) {
                    bestEarIdx = i;
                    break;
                }
            }
        }
        
        // Last resort: just pick vertex 0
        if (bestEarIdx == SIZE_MAX) {
            bestEarIdx = 0;
        }
        
        // Emit the ear triangle
        const size_t prev = (bestEarIdx + n - 1) % n;
        const size_t next = (bestEarIdx + 1) % n;
        
        const uint32_t idxA = polygon[prev];
        const uint32_t idxB = polygon[bestEarIdx];
        const uint32_t idxC = polygon[next];
        
        result.indices.push_back(idxA);
        result.indices.push_back(idxB);
        result.indices.push_back(idxC);
        result.totalEdgeLength += trianglePerimeter(vertices, idxA, idxB, idxC);
        
        // Remove the ear tip vertex from polygon
        polygon.erase(polygon.begin() + static_cast<ptrdiff_t>(bestEarIdx));
    }
    
    // Emit the final triangle
    if (polygon.size() == 3) {
        result.indices.push_back(polygon[0]);
        result.indices.push_back(polygon[1]);
        result.indices.push_back(polygon[2]);
        result.totalEdgeLength += trianglePerimeter(vertices, polygon[0], polygon[1], polygon[2]);
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

} // namespace Triangulation
