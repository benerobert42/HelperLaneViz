//
//  SVGLoader.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 11..
//

#include "SVGLoader.h"
#include "../Geometry/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <simd/simd.h>
#include <vector>

#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

namespace {

float GetSignedArea(const std::vector<simd_float2>& p) {
    if (p.size() < 3) return 0.0f;
    double area = 0.0;
    size_t n = p.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += static_cast<double>(p[i].x) * p[j].y;
        area -= static_cast<double>(p[j].x) * p[i].y;
    }
    return static_cast<float>(0.5 * area);
}

float PointLineDistance(const simd_float2& p, const simd_float2& a, const simd_float2& b) {
    const simd_float2 ab = b - a;
    const float lenSq = simd_length_squared(ab);
    if (lenSq < 1e-12f) return simd_distance(p, a);
    const float t = simd_dot(p - a, ab) / lenSq;
    return simd_distance(p, a + simd_clamp(t, 0.0f, 1.0f) * ab);
}

// Adaptive Bézier subdivision
void SampleCubicBezier(const simd_float2& p0, const simd_float2& p1,
                       const simd_float2& p2, const simd_float2& p3,
                       float maxDev, std::vector<simd_float2>& out) {
    float d1 = PointLineDistance(p1, p0, p3);
    float d2 = PointLineDistance(p2, p0, p3);
    if (std::max(d1, d2) <= maxDev) {
        out.push_back(p3);
        return;
    }
    simd_float2 p01 = (p0 + p1) * 0.5f;
    simd_float2 p12 = (p1 + p2) * 0.5f;
    simd_float2 p23 = (p2 + p3) * 0.5f;
    simd_float2 p012 = (p01 + p12) * 0.5f;
    simd_float2 p123 = (p12 + p23) * 0.5f;
    simd_float2 mid = (p012 + p123) * 0.5f;
    SampleCubicBezier(p0, p01, p012, mid, maxDev, out);
    SampleCubicBezier(mid, p123, p23, p3, maxDev, out);
}

std::vector<simd_float2> TessellatePath(NSVGpath* path, float bezierMaxDev) {
    std::vector<simd_float2> poly;
    const float* pts = path->pts;
    const int npts = path->npts;
    
    // NanoSVG stores cubic Bezier curves: first point + N segments of 3 control points each
    // So valid paths have npts = 1 + 3*N for some N >= 1, meaning npts >= 4
    if (npts < 4) return poly;
    
    simd_float2 p0{pts[0], pts[1]};
    poly.push_back(p0);
    
    // Process each cubic Bezier segment
    int numSegments = (npts - 1) / 3;
    for (int seg = 0; seg < numSegments; seg++) {
        int i = 1 + seg * 3;
        simd_float2 p1{pts[i*2], pts[i*2+1]};
        simd_float2 p2{pts[(i+1)*2], pts[(i+1)*2+1]};
        simd_float2 p3{pts[(i+2)*2], pts[(i+2)*2+1]};
        SampleCubicBezier(p0, p1, p2, p3, bezierMaxDev, poly);
        p0 = p3;
    }
    
    // Remove duplicate closing vertex if present
    if (poly.size() > 1) {
        if (simd_distance(poly.front(), poly.back()) < 1.0f) {
            poly.pop_back();
        }
    }
    return poly;
}

SVGLoader::AABB2 ComputeAABB(const std::vector<simd_float2>& poly) {
    SVGLoader::AABB2 bb;
    bb.min = simd_make_float2(INFINITY, INFINITY);
    bb.max = simd_make_float2(-INFINITY, -INFINITY);
    for (const auto& p : poly) {
        bb.min = simd_min(bb.min, p);
        bb.max = simd_max(bb.max, p);
    }
    return bb;
}

bool PointInPolygon(const simd_float2& pt, const std::vector<simd_float2>& poly) {
    int crossings = 0;
    size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % n];
        if ((a.y <= pt.y && b.y > pt.y) || (b.y <= pt.y && a.y > pt.y)) {
            float t = (pt.y - a.y) / (b.y - a.y);
            if (pt.x < a.x + t * (b.x - a.x)) {
                crossings++;
            }
        }
    }
    return (crossings % 2) == 1;
}

bool IsPolygonInsidePolygon(const std::vector<simd_float2>& inner,
                            const std::vector<simd_float2>& outer) {
    // Check if any point of inner is inside outer
    for (const auto& p : inner) {
        if (PointInPolygon(p, outer)) return true;
    }
    return false;
}

std::vector<Vertex> PolyToVertices(const std::vector<simd_float2>& poly) {
    std::vector<Vertex> verts;
    verts.reserve(poly.size());
    for (const auto& p : poly) {
        verts.push_back(Vertex{.position = {p.x, p.y, 1.0f}});
    }
    return verts;
}

SVGLoader::AABB2 ComputeAABB2(const std::vector<Vertex>& vertices) {
    SVGLoader::AABB2 bb;
    bb.min = simd_make_float2(INFINITY, INFINITY);
    bb.max = simd_make_float2(-INFINITY, -INFINITY);
    for (const auto& v : vertices) {
        simd_float2 p = simd_make_float2(v.position.x, v.position.y);
        bb.min = simd_min(bb.min, p);
        bb.max = simd_max(bb.max, p);
    }
    return bb;
}

} // anonymous namespace

std::vector<SVGLoader::ShapeWithHoles> SVGLoader::ParseSvgToShapes(
    const std::string& filePath, float bezierMaxDeviationPx)
{
    std::vector<ShapeWithHoles> result;
    
    NSVGimage* image = nsvgParseFromFile(filePath.c_str(), "px", 96.0f);
    if (!image) return result;
    
    int shapeCount = 0;
    int pathCount = 0;
    
    fprintf(stderr, "SVGLoader: Image size: %.0f x %.0f\n", image->width, image->height);
    
    // Process each SVG shape - each NSVGshape is treated as independent
    for (NSVGshape* shape = image->shapes; shape; shape = shape->next) {
        shapeCount++;
        if ((shape->flags & NSVG_FLAGS_VISIBLE) == 0) {
            fprintf(stderr, "SVGLoader: Shape %d invisible, skipping\n", shapeCount);
            continue;
        }
        
        // Count paths in this shape
        int pathsInShape = 0;
        for (NSVGpath* p = shape->paths; p; p = p->next) pathsInShape++;
        fprintf(stderr, "SVGLoader: Shape %d: %d paths, fill=0x%08X\n", 
                shapeCount, pathsInShape, shape->fill.color);
        
        // Collect all closed paths in this shape
        struct PathInfo {
            std::vector<simd_float2> poly;
            float signedArea;
        };
        std::vector<PathInfo> paths;
        
        int localPathCount = 0;
        for (NSVGpath* path = shape->paths; path; path = path->next) {
            pathCount++;
            localPathCount++;
            
            fprintf(stderr, "SVGLoader: Shape %d Path %d: npts=%d, closed=%d\n",
                    shapeCount, localPathCount, path->npts, path->closed);
            
            std::vector<simd_float2> poly = TessellatePath(path, bezierMaxDeviationPx);
            if (poly.size() < 3) {
                fprintf(stderr, "SVGLoader: Shape %d Path %d: tessellated to only %zu vertices, skipping\n", 
                        shapeCount, localPathCount, poly.size());
                continue;
            }
            
            // Check if path is closed or implicitly closed (first ≈ last point)
            bool isClosed = path->closed;
            if (!isClosed && poly.size() >= 3) {
                // Compute bounding box to determine threshold
                AABB2 bb = ComputeAABB(poly);
                float bbDiag = simd_distance(bb.min, bb.max);
                float dist = simd_distance(poly.front(), poly.back());
                
                // Consider closed if gap is less than 5% of bounding box diagonal
                float threshold = std::max(5.0f, bbDiag * 0.05f);
                if (dist < threshold) {
                    isClosed = true;
                    fprintf(stderr, "SVGLoader: Shape %d Path %d: implicitly closed (dist=%.2f, bbDiag=%.1f)\n",
                            shapeCount, localPathCount, dist, bbDiag);
                    // Remove duplicate last point if very close
                    if (dist < 1.0f) {
                        poly.pop_back();
                    }
                }
            }
            
            if (!isClosed) {
                fprintf(stderr, "SVGLoader: Shape %d Path %d: open path (%zu verts), skipping\n", 
                        shapeCount, localPathCount, poly.size());
                continue;
            }
            
            if (poly.size() < 3) continue;
            
            float area = GetSignedArea(poly);
            fprintf(stderr, "SVGLoader: Shape %d Path %d: %zu vertices, area=%.1f\n", 
                    shapeCount, localPathCount, poly.size(), area);
            paths.push_back({std::move(poly), area});
        }
        
        if (paths.empty()) {
            fprintf(stderr, "SVGLoader: Shape %d: no valid paths\n", shapeCount);
            continue;
        }
        
        // If only one path in this shape, it's a simple shape
        if (paths.size() == 1) {
            ShapeWithHoles shapeWithHoles;
            auto& poly = paths[0].poly;
            // Ensure CCW orientation
            if (paths[0].signedArea < 0) {
                std::reverse(poly.begin(), poly.end());
            }
            shapeWithHoles.outerBoundary = PolyToVertices(poly);
            result.push_back(std::move(shapeWithHoles));
            continue;
        }
        
        // Multiple paths in one shape - detect holes by winding order
        // Sort by absolute area (largest first)
        std::sort(paths.begin(), paths.end(), [](const PathInfo& a, const PathInfo& b) {
            return std::abs(a.signedArea) > std::abs(b.signedArea);
        });
        
        // The largest path is the outer boundary
        // Paths with opposite winding that are inside it are holes
        auto& outerPoly = paths[0].poly;
        float outerArea = paths[0].signedArea;
        
        // Ensure outer is CCW (positive area)
        if (outerArea < 0) {
            std::reverse(outerPoly.begin(), outerPoly.end());
            outerArea = -outerArea;
        }
        
        ShapeWithHoles shapeWithHoles;
        shapeWithHoles.outerBoundary = PolyToVertices(outerPoly);
        
        // Check remaining paths - holes have opposite original winding and are inside outer
        for (size_t i = 1; i < paths.size(); ++i) {
            auto& innerPoly = paths[i].poly;
            float innerArea = paths[i].signedArea;
            
            // A hole has opposite winding from outer (before normalization)
            // Since we normalized outer to CCW, original CW paths (negative area) are holes
            bool isLikelyHole = (paths[0].signedArea > 0) ? (paths[i].signedArea < 0) 
                                                          : (paths[i].signedArea > 0);
            
            if (isLikelyHole && IsPolygonInsidePolygon(innerPoly, outerPoly)) {
                // Ensure hole is CW
                if (innerArea > 0) {
                    std::reverse(innerPoly.begin(), innerPoly.end());
                }
                shapeWithHoles.holes.push_back(PolyToVertices(innerPoly));
            } else {
                // Not a hole - treat as separate shape
                ShapeWithHoles separateShape;
                if (innerArea < 0) {
                    std::reverse(innerPoly.begin(), innerPoly.end());
                }
                separateShape.outerBoundary = PolyToVertices(innerPoly);
                result.push_back(std::move(separateShape));
            }
        }
        
        result.push_back(std::move(shapeWithHoles));
    }
    
    nsvgDelete(image);
    
    fprintf(stderr, "SVGLoader: Parsed %d NSVGshapes, %d NSVGpaths, produced %zu output shapes\n",
            shapeCount, pathCount, result.size());
    
    return result;
}

bool SVGLoader::TessellateSvgToMesh(const std::string& filePath,
                                    std::vector<Vertex>& outPositions,
                                    std::vector<uint32_t>& outIndices,
                                    Triangulator triangulator,
                                    float bezierMaxDeviationPx)
{
    outPositions.clear();
    outIndices.clear();
    
    auto shapes = ParseSvgToShapes(filePath, bezierMaxDeviationPx);
    if (shapes.empty()) {
        fprintf(stderr, "SVGLoader: No shapes found in %s\n", filePath.c_str());
        return false;
    }
    
    fprintf(stderr, "SVGLoader: Found %zu shapes in %s\n", shapes.size(), filePath.c_str());
    
    uint32_t baseVertex = 0;
    int shapeIdx = 0;
    int successfulShapes = 0;
    
    for (auto& shape : shapes) {
        if (shape.outerBoundary.size() < 3) {
            shapeIdx++;
            continue;
        }
        
        std::vector<uint32_t> localIndices;
        std::vector<Vertex> allVertices;
        
        if (shape.holes.empty()) {
            // Simple shape - try custom triangulator first
            allVertices = shape.outerBoundary;
            size_t expectedTris = allVertices.size() - 2;
            
            // Try custom triangulator
            localIndices = triangulator(allVertices);
            
            // Check if we got a reasonable result
            // Note: triangulator might modify allVertices (e.g., centroidFan adds vertex)
            size_t actualTris = localIndices.size() / 3;
            bool needsFallback = localIndices.empty() || 
                                 (actualTris < expectedTris && allVertices.size() == shape.outerBoundary.size());
            
            if (needsFallback) {
                fprintf(stderr, "SVGLoader: Shape %d: custom tri gave %zu tris (expected %zu), using CDT\n",
                        shapeIdx, actualTris, expectedTris);
                        
                // Reset vertices and use CDT
                allVertices = shape.outerBoundary;
                localIndices = Triangulation::constrainedDelaunay(allVertices);
                
                if (localIndices.empty()) {
                    fprintf(stderr, "SVGLoader: Shape %d: CDT also failed!\n", shapeIdx);
                }
            }
        } else {
            // Shape with holes - must use CDT
            allVertices = shape.outerBoundary;
            for (const auto& hole : shape.holes) {
                allVertices.insert(allVertices.end(), hole.begin(), hole.end());
            }
            
            localIndices = Triangulation::constrainedDelaunayWithHoles(
                shape.outerBoundary, shape.holes);
        }
        
        if (localIndices.empty()) {
            fprintf(stderr, "SVGLoader: Shape %d FAILED (%zu verts)\n",
                    shapeIdx, shape.outerBoundary.size());
            shapeIdx++;
            continue;
        }
        
        // Append to output
        for (const auto& v : allVertices) {
            outPositions.push_back(v);
        }
        for (uint32_t idx : localIndices) {
            outIndices.push_back(baseVertex + idx);
        }
        baseVertex += static_cast<uint32_t>(allVertices.size());
        successfulShapes++;
        shapeIdx++;
    }
    
    fprintf(stderr, "SVGLoader: Successfully triangulated %d/%zu shapes, %zu verts, %zu tris\n",
            successfulShapes, shapes.size(), outPositions.size(), outIndices.size() / 3);
    
    // Normalize by bounding box
    if (!outPositions.empty()) {
        AABB2 bb = ComputeAABB2(outPositions);
        float size = simd_length(bb.max - bb.min);
        if (size > 1e-6f) {
            for (auto& v : outPositions) {
                v.position /= size;
            }
        }
    }
    
    return !outPositions.empty() && !outIndices.empty();
}

bool SVGLoader::TessellateSvgToMesh(const std::string& filePath,
                                    std::vector<Vertex>& outPositions,
                                    std::vector<uint32_t>& outIndices,
                                    float bezierMaxDeviationPx)
{
    // Default: use CDT for all shapes (handles both simple and holed shapes)
    auto defaultTriangulator = [](std::vector<Vertex>& verts) {
        return Triangulation::constrainedDelaunay(verts);
    };
    return TessellateSvgToMesh(filePath, outPositions, outIndices, 
                               defaultTriangulator, bezierMaxDeviationPx);
}
