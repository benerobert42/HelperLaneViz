//
//  SVGLoader.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 11..
//

#include "SVGLoader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <simd/simd.h>
#include <string>
#include <vector>

#include <filesystem>
#include <fstream>
#include <sstream>

#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

namespace {
float GetOrientation(const simd_float2& a, const simd_float2& b, const simd_float2& c) {
    return simd_cross(b-a, c-a).z;
}

float GetSignedArea(const std::vector<simd_float2>& p) {
    double area = 0.0;
    size_t n = p.size();
    for (size_t i = 0; i < n - 1; ++i) {
        area += simd_cross(p[i], p[i + 1]).z;
    }
    area += simd_cross(p[n - 1], p[0]).z;
    return float(0.5 * area);
}

bool IsPointInTriangle(const simd_float2& p,
                       const simd_float2& a,
                       const simd_float2& b,
                       const simd_float2& c) {
    const float c1 = GetOrientation(a, b, p);
    const float c2 = GetOrientation(b, c, p);
    const float c3 = GetOrientation(c, a, p);
    const bool hasNeg = (c1 < 0) || (c2 < 0) || (c3 < 0);
    const bool hasPos = (c1 > 0) || (c2 > 0) || (c3 > 0);
    return !(hasNeg && hasPos);
}

float PointLineDistance(const simd_float2& p, const simd_float2& a, const simd_float2& b) {
    const simd_float2 abDiff = a - b;
    const float squaredDistance = simd_length_squared(abDiff);
    if (squaredDistance <= 1e-12f) {
        return simd_distance(p, a);
    }
    const float t = ((p.x - a.x) * abDiff.x + (p.y - a.y) * abDiff.y) / squaredDistance;
    const simd_float2 q = a + t * abDiff;
    return simd_distance(p, q);
}

// Adaptive subdivision of a cubic Bézier (p0, p1, p2, p3) into points.
// maxDeviation: maximum allowed distance of control points from chord (pixels).
static void SampleCubicBezier(const simd_float2& p0,
                              const simd_float2& p1,
                              const simd_float2& p2,
                              const simd_float2& p3,
                              float maxDeviation,
                              std::vector<simd_float2>& outPoints)
{
    // Flatness check: distance of control points from chord p0-p3
    const float d1 = PointLineDistance(p1, p0, p3);
    const float d2 = PointLineDistance(p2, p0, p3);
    if (std::max(d1, d2) <= maxDeviation) {
        // Append end point only; caller should have added p0 already.
        outPoints.push_back(p3);
        return;
    }
    
    // de Casteljau split at t=0.5
    simd_float2 p01 = (p0 + p1) * 0.5f;
    simd_float2 p12 = (p1 + p2) *0.5f;
    simd_float2 p23 = (p2 + p3) *0.5f;
    simd_float2 p012 = (p01 + p12) * 0.5f;
    simd_float2 p123 = (p12 + p23) * 0.5f;
    simd_float2 p0123 = (p012 + p123) * 0.5f;
    
    SampleCubicBezier(p0, p01, p012, p0123, maxDeviation, outPoints);
    SampleCubicBezier(p0123, p123, p23, p3, maxDeviation, outPoints);
}

// Simple ear-clipping triangulation (single simple polygon, CCW).
// Returns local indices (0..N-1) of triangles.
static std::vector<uint32_t> TriangulateEarClipping(const std::vector<simd_float2>& polygon)
{
    std::vector<uint32_t> indices(polygon.size());
    for (uint32_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }
    
    std::vector<uint32_t> triangles;
    triangles.reserve(polygon.size() * 3);
    
    auto isConvex = [&](uint32_t i0, uint32_t i1, uint32_t i2) {
        return GetOrientation(polygon[i0], polygon[i1], polygon[i2]) > 0.0f; // CCW
    };
    
    size_t guard = 0;
    while (indices.size() >= 3 && guard++ < 100000) {
        bool foundEar = false;
        const size_t n = indices.size();
        for (size_t i = 0; i < n; ++i) {
            const uint32_t i0 = indices[(i + n - 1) % n];
            const uint32_t i1 = indices[i];
            const uint32_t i2 = indices[(i + 1) % n];
            
            if (!isConvex(i0, i1, i2))
                continue;
            
            bool anyInside = false;
            for (size_t j = 0; j < n; ++j) {
                const uint32_t k = indices[j];
                if (k == i0 || k == i1 || k == i2) {
                    continue;
                }
                if (IsPointInTriangle(polygon[k], polygon[i0], polygon[i1], polygon[i2])) {
                    anyInside = true;
                    break;
                }
            }
            if (anyInside) {
                continue;
            }
            
            // Ear found
            triangles.push_back(i0);
            triangles.push_back(i1);
            triangles.push_back(i2);
            indices.erase(indices.begin() + i);
            foundEar = true;
            break;
        }
        if (!foundEar) {
            break; // polygon may be non-simple or numerically degenerate
        }
    }
    return triangles;
}

SVGLoader::AABB2 ComputeAABB2(const std::vector<Vertex>& vertices)
{
    SVGLoader::AABB2 bb;
    bb.min = simd_make_float2(std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity());
    bb.max = simd_make_float2(-std::numeric_limits<float>::infinity(),
                              -std::numeric_limits<float>::infinity());
    for (const Vertex& vertex : vertices) {
        const simd_float2 p = simd_make_float2(vertex.position.x, vertex.position.y);
        bb.min = simd_min(bb.min, p);
        bb.max = simd_max(bb.max, p);
    }
    return bb;
}

// Helper: tessellate a single NSVGpath into polygon vertices
std::vector<simd_float2> TessellatePath(NSVGpath* path, float bezierMaxDeviationPx) {
    std::vector<simd_float2> poly;
    
    const float* pts = path->pts;
    const int npts = path->npts;
    if (npts < 4) return poly;
    
    simd_float2 p0{pts[0], pts[1]};
    poly.push_back(p0);
    
    for (int i = 1; i + 2 < npts; i += 3) {
        simd_float2 p1{pts[(i + 0) * 2 + 0], pts[(i + 0) * 2 + 1]};
        simd_float2 p2{pts[(i + 1) * 2 + 0], pts[(i + 1) * 2 + 1]};
        simd_float2 p3{pts[(i + 2) * 2 + 0], pts[(i + 2) * 2 + 1]};
        SampleCubicBezier(p0, p1, p2, p3, bezierMaxDeviationPx, poly);
        p0 = p3;
    }
    
    // Deduplicate last == first
    if (poly.size() > 1) {
        const simd_float2& a = poly.front();
        const simd_float2& b = poly.back();
        if (std::fabs(a.x - b.x) < 1e-6f && std::fabs(a.y - b.y) < 1e-6f) {
            poly.pop_back();
        }
    }
    
    return poly;
}
}

// Main entry: loads an SVG, tessellates each closed path into polylines.
// If triangulate=true, also performs ear-clipping triangulation.
// Outputs packed 2D positions (x,y) and 32-bit indices (empty if triangulate=false).
bool SVGLoader::TessellateSvgToMesh(const std::string& filePath,
                                    std::vector<Vertex>& outPositions,
                                    std::vector<uint32_t>& outIndices,
                                    float bezierMaxDeviationPx,
                                    bool triangulate)
{
    outPositions.clear();
    outIndices.clear();

    namespace fs = std::filesystem;
    if (!fs::exists(filePath)) {
        fprintf(stderr, "SVG not found: %s\n", filePath.c_str());
        return false;
    }

    NSVGimage* image = nsvgParseFromFile(filePath.c_str(), "px", 96.0f);
    if (!image) {
        fprintf(stderr, "Failed to parse SVG: %s\n", filePath.c_str());
        return false;
    }

    // We build one big mesh by concatenating path meshes.
    uint32_t baseVertex = 0;

    for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
        if ((shape->flags & NSVG_FLAGS_VISIBLE) == 0) continue;

        for (NSVGpath* path = shape->paths; path != nullptr; path = path->next) {
            if (!path->closed) continue;

            std::vector<simd_float2> poly = TessellatePath(path, bezierMaxDeviationPx);
            if (poly.size() < 3) continue;

            // Ensure CCW orientation
            if (GetSignedArea(poly) <= 0.0f) {
                std::reverse(poly.begin(), poly.end());
            }

            if (triangulate) {
                // Triangulate this polygon with ear-clipping
                std::vector<uint32_t> local = TriangulateEarClipping(poly);
                if (local.empty()) continue;

                // Append to global buffers
                const uint32_t pathVertexStart = baseVertex;
                for (const simd_float2& v : poly) {
                    outPositions.push_back(Vertex{.position = {v.x, v.y, 1.0f}});
                }
                for (uint32_t idx : local) {
                    outIndices.push_back(pathVertexStart + idx);
                }
                baseVertex += static_cast<uint32_t>(poly.size());
            } else {
                // Just append vertices (no triangulation)
                for (const simd_float2& v : poly) {
                    outPositions.push_back(Vertex{.position = {v.x, v.y, 1.0f}});
                }
            }
        }
    }

    // Normalize by bounding box
    if (!outPositions.empty()) {
        AABB2 boundingBox = ComputeAABB2(outPositions);
        float bbSize = simd_length(boundingBox.max - boundingBox.min);
        if (bbSize > 1e-6f) {
            for (auto& vertex : outPositions) {
                vertex.position /= bbSize;
            }
        }
    }

    nsvgDelete(image);
    return !outPositions.empty();
}

