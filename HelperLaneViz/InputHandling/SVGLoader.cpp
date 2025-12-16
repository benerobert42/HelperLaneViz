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
    
    // Remove duplicate closing vertex if very close to first
    while (poly.size() > 3 && simd_distance(poly.front(), poly.back()) < 0.5f) {
        poly.pop_back();
    }
    
    return poly;
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

std::vector<SVGLoader::ShapeWithHoles> SVGLoader::ParseSvgToShapes(const std::string& filePath,
                                                                   float bezierMaxDeviationPx) {
    std::vector<ShapeWithHoles> result;
    
    NSVGimage* image = nsvgParseFromFile(filePath.c_str(), "px", 96.0f);
    if (!image) return result;
    
    for (NSVGshape* shape = image->shapes; shape; shape = shape->next) {
        if ((shape->flags & NSVG_FLAGS_VISIBLE) == 0) continue;
        
        for (NSVGpath* path = shape->paths; path; path = path->next) {
            std::vector<simd_float2> poly = TessellatePath(path, bezierMaxDeviationPx);
            if (poly.size() < 3) continue;
            
            // Check if closed (explicitly or implicitly)
            bool isClosed = path->closed;
            if (!isClosed) {
                float dist = simd_distance(poly.front(), poly.back());
                // Very generous threshold - if first/last are close, treat as closed
                if (dist < 50.0f) {
                    isClosed = true;
                }
            }
            
            if (!isClosed) continue;

            ShapeWithHoles shapeWithHoles;

            if (GetSignedArea(poly) < 0) {
                std::reverse(poly.begin(), poly.end());
            }
            shapeWithHoles.outerBoundary = PolyToVertices(poly);

            result.push_back(std::move(shapeWithHoles));
        }
    }
    
    nsvgDelete(image);
    
    fprintf(stderr, "SVGLoader: Parsed %zu shapes from %s\n", result.size(), filePath.c_str());
    
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
        fprintf(stderr, "SVGLoader: No shapes found\n");
        return false;
    }

    // Helper to triangulate and append a single polygon
    auto triangulateAndAppend = [&](const std::vector<Vertex>& polygon) {
        if (polygon.size() < 3) return;
        
        const uint32_t baseIndex = static_cast<uint32_t>(outPositions.size());
        std::vector<uint32_t> indices = triangulator(polygon, true);
        
        if (indices.empty()) {
            fprintf(stderr, "SVGLoader: Triangulation failed for polygon with %zu vertices\n", polygon.size());
            return;
        }
        
        // Append vertices
        for (const auto& v : polygon) {
            outPositions.push_back(v);
        }
        
        // Append indices (offset by base)
        for (uint32_t idx : indices) {
            outIndices.push_back(baseIndex + idx);
        }
    };

    // Process each shape - outer boundary and holes as separate polygons
    for (auto& shape : shapes) {
        triangulateAndAppend(shape.outerBoundary);
        
        for (auto& hole : shape.holes) {
            triangulateAndAppend(hole);
        }
    }
    
    fprintf(stderr, "SVGLoader: Output: %zu vertices, %zu triangles\n",
            outPositions.size(), outIndices.size() / 3);
    
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
    // Default: use CDT
    auto defaultTriangulator = [](const std::vector<Vertex>& verts, bool) {
        return Triangulation::ConstrainedDelaunayTriangulation(verts);
    };
    return TessellateSvgToMesh(filePath,
                               outPositions,
                               outIndices,
                               defaultTriangulator,
                               bezierMaxDeviationPx);
}
