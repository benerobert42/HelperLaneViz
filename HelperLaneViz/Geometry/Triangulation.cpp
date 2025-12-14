//
//  Triangulation.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#include "Triangulation.h"

#include "TriangulationHelpers.h"
#include "TriangulationHoles.h"

#include <cmath>
#include <limits>
#include <numeric>
#include <functional>
#include <set>
#include <tuple>

#include <Eigen/Core>
#include <igl/triangle/triangulate.h>

namespace Triangulation {

double calculateTotalEdgeLength(const std::vector<Vertex>& vertices,
                                const std::vector<uint32_t>& indices) {
    double total = 0.0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        total += Helpers::TrianglePerimeter(vertices, indices[i], indices[i+1], indices[i+2]);
    }
    return total;
}

std::vector<Vertex> buildPolygonWithHoles(const std::vector<Vertex>& outer,
                                          const std::vector<std::vector<Vertex>>& holes) {
    auto outVector = outer;
    for (const auto& hole: holes) {
        outVector = Helpers::BuildBridgedPolygonImpl(outVector, hole);
    }
    return outVector;
}


// MARK: Simple triangulations
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

    const bool likelyConvex = Helpers::PolygonSignedArea(vertices) >= 0.0;

    // Build strip order: alternating from start and end
    std::vector<uint32_t> stripOrder;
    stripOrder.reserve(vertexCount);

    size_t leftIdx = 0;
    size_t rightIdx = vertexCount - 1;

    if (likelyConvex) {
        // For convex: use direct indices (assume already in order)
        while (leftIdx <= rightIdx) {
            stripOrder.push_back(static_cast<uint32_t>(leftIdx++));
            if (leftIdx > rightIdx) break;
            stripOrder.push_back(static_cast<uint32_t>(rightIdx--));
        }
    } else {
        // For concave: ensure CCW order first
        const std::vector<uint32_t> ccwOrder = Helpers::BuildCCWOrder(vertices);
        while (leftIdx <= rightIdx) {
            stripOrder.push_back(ccwOrder[leftIdx++]);
            if (leftIdx > rightIdx) break;
            stripOrder.push_back(ccwOrder[rightIdx--]);
        }
    }

    // Generate triangles from consecutive strip triplets
    indices.reserve((vertexCount - 2) * 3);

    for (size_t i = 0; i + 2 < stripOrder.size(); ++i) {
        uint32_t indexA = stripOrder[i];
        uint32_t indexB = stripOrder[i + 1];
        uint32_t indexC = stripOrder[i + 2];

        // Alternate winding to maintain consistent orientation (only needed for concave)
        if (!likelyConvex) {
            const bool isClockwise = Helpers::PolygonSignedArea(vertices) < 0.0;
            const bool shouldSwap = isClockwise ? ((i % 2) == 0) : ((i % 2) == 1);
            if (shouldSwap) {
                std::swap(indexA, indexB);
            }
        }

        indices.push_back(indexA);
        indices.push_back(indexB);
        indices.push_back(indexC);
    }

    return indices;
}

// MARK: DP triangulations
std::vector<uint32_t> MinimumWeightTriangulation(const std::vector<Vertex>& vertices,
                                                 bool shouldHandleConcave,
                                                 bool handleHoles,
                                                 const std::vector<Vertex>& outerVertices,
                                                 const std::vector<std::vector<Vertex>>& holes) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
    if (vertexCount < 3) {
        return indices;
    }

    const bool needsTopologyChecks = shouldHandleConcave || handleHoles;

    Helpers::DiagonalTable diagTable;
    if (needsTopologyChecks) {
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
                if (needsTopologyChecks) {
                    if (shouldHandleConcave) {
                        if (!IsTriangleInsidePolygon(vertices, startIndex, splitPoint, endIndex, diagTable)) {
                            continue;
                        }
                    }
                    if (handleHoles) {
                        if (!Helpers::IsTriangleValidWithHoles(vertices, startIndex, splitPoint, endIndex, outerVertices, holes)) {
                            continue;
                        }
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
        
        // Emit triangle (startIndex, splitPoint, endIndex)
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
                                              bool shouldHandleConcave,
                                              bool handleHoles,
                                              const std::vector<Vertex>& outerVertices,
                                              const std::vector<std::vector<Vertex>>& holes) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
    if (vertexCount < 3) {
        return indices;
    }

    const bool needsTopologyChecks = shouldHandleConcave || handleHoles;

    Helpers::DiagonalTable diagTable;
    if (needsTopologyChecks) {
        diagTable = Helpers::BuildDiagonalTable(vertices);
    }

    // For convex polygons (shouldHandleConcave=false), skip CCW order rebuild
    std::vector<uint32_t> ccwOrder;
    if (shouldHandleConcave) {
        ccwOrder = Helpers::BuildCCWOrder(vertices);
    } else {
        ccwOrder.resize(vertexCount);
        std::iota(ccwOrder.begin(), ccwOrder.end(), 0);
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
                const uint32_t A = ccwOrder[start];
                const uint32_t B = ccwOrder[mid];
                const uint32_t C = ccwOrder[end];

                // Topology checks: concave validity + holes
                if (needsTopologyChecks) {
                    if (shouldHandleConcave &&
                        !IsTriangleInsidePolygon(vertices, A, B, C, diagTable)) {
                        continue;
                    }
                    if (handleHoles &&
                        !Helpers::IsTriangleValidWithHoles(vertices, A, B, C,
                                                           outerVertices, holes)) {
                        continue;
                    }
                } else {
                    if (Helpers::TriangleArea(vertices, A, B, C) <= 0.0) {
                        continue;
                    }
                }

                const double triArea = Helpers::TriangleArea(vertices, A, B, C);

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

        uint32_t A = ccwOrder[start];
        uint32_t B = ccwOrder[mid];
        uint32_t C = ccwOrder[end];

        // Ensure orientation matches polygon winding
        if (!Helpers::IsCounterClockwise(vertices, A, B, C)) {
            std::swap(B, C);
        }

        indices.push_back(A);
        indices.push_back(B);
        indices.push_back(C);

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
                                              bool shouldHandleConcave,
                                              bool handleHoles,
                                              const std::vector<Vertex>& outerVertices,
                                              const std::vector<std::vector<Vertex>>& holes) {
    std::vector<uint32_t> indices;
    const int vertexCount = static_cast<int>(vertices.size());
    if (vertexCount < 3) {
        return indices;
    }

    const bool needsTopologyChecks = shouldHandleConcave || handleHoles;

    Helpers::DiagonalTable diagTable;
    if (needsTopologyChecks) {
        diagTable = Helpers::BuildDiagonalTable(vertices);
    }

    // For convex polygons skip CCW order rebuild
    std::vector<uint32_t> ccwOrder(vertexCount);
    if (shouldHandleConcave) {
        ccwOrder = Helpers::BuildCCWOrder(vertices);
    } else {
        std::iota(ccwOrder.begin(), ccwOrder.end(), 0);
    }

    // DP tables: dp[i][j] = minimum achievable maximum triangle area for chain [i, j]
    std::vector<double> dpTable(vertexCount * vertexCount, 0.0);
    std::vector<int>    splitTable(vertexCount * vertexCount, -1);

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
                const uint32_t A = ccwOrder[start];
                const uint32_t B = ccwOrder[mid];
                const uint32_t C = ccwOrder[end];

                if (needsTopologyChecks) {
                    if (shouldHandleConcave &&
                        !IsTriangleInsidePolygon(vertices, A, B, C, diagTable)) {
                        continue;
                    }
                    if (handleHoles &&
                        !Helpers::IsTriangleValidWithHoles(vertices, A, B, C,
                                                           outerVertices, holes)) {
                        continue;
                    }
                } else {
                    // Convex, no holes: skip degenerate triangles
                    if (Helpers::TriangleArea(vertices, A, B, C) <= 0.0) {
                        continue;
                    }
                }

                const double triArea = Helpers::TriangleArea(vertices, A, B, C);

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

        uint32_t A = ccwOrder[start];
        uint32_t B = ccwOrder[mid];
        uint32_t C = ccwOrder[end];

        // Ensure CCW orientation
        if (!Helpers::IsCounterClockwise(vertices, A, B, C)) {
            std::swap(B, C);
        }

        indices.push_back(A);
        indices.push_back(B);
        indices.push_back(C);

        if (mid > start + 1) emitTriangles(start, mid);
        if (end > mid + 1)   emitTriangles(mid,   end);
    };

    emitTriangles(0, vertexCount - 1);

    return indices;
}

// MARK: Max Area and CDT
std::vector<uint32_t> GreedyMaxAreaTriangulation(const std::vector<Vertex>& vertices, bool shouldHandleConcave, bool handleHoles, const std::vector<Vertex>& outerVertices, const std::vector<std::vector<Vertex>>& holes) {
    std::vector<uint32_t> indices;
    const size_t vertexCount = vertices.size();
    const auto diagTable = Helpers::BuildDiagonalTable(vertices);

    if (vertexCount < 3) {
        return indices;
    }

    // For convex polygons (shouldHandleConcave=false), skip CCW order rebuild
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

                        // Calculate area first (needed for both validation and selection)
                        const double area = Helpers::TriangleArea(vertices, vi, vj, vk);

                        // For convex polygons (shouldHandleConcave=false), check triangle has positive area
                        // For concave polygons, we need full validity check
                        if (shouldHandleConcave) {
                            if (!Helpers::IsTriangleInsidePolygon(vertices, static_cast<int>(vi), static_cast<int>(vj), static_cast<int>(vk), diagTable)) {
                                continue;
                            }
                            if (handleHoles) {
                                if (!Helpers::IsTriangleValidWithHoles(vertices, static_cast<int>(vi), static_cast<int>(vj), static_cast<int>(vk), outerVertices, holes)) {
                                    continue;;
                                }
                            }
                        } else {
                            // For convex polygons, still ensure triangle has positive area (degenerate check)
                            if (area <= 0.0) continue;
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
