//
//  TileMetrics.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 23..
//

#pragma once
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <simd/simd.h>

namespace ttm {

struct TrianglePx{
    // triangle vertices in pixel coordinates
    simd_int2 a;
    simd_int2 b;
    simd_int2 c;
};

struct Metrics {
    // Primary signals
    double TTO            = 0.0;   // sum of tiles touched by all triangles
    double HTP_P95        = 0.0;   // P95 of triangles-per-tile distribution
    double HTP_mean        = 0.0;   // P95 of triangles-per-tile distribution
    double HTP_median        = 0.0;   // P95 of triangles-per-tile distribution
    double SS_P95         = 0.0;   // P95 of tiles-per-triangle distribution
    double SS_mean        = 0.0;
    double SS_median      = 0.0;
    double BCI            = 0.0;   // TTO / (sum(area_px) / tile_area)

    // Optional context (can be useful for debugging/plots)
    double triCount       = 0.0;
    double nonEmptyTiles  = 0.0;
    double sumAreaPx      = 0.0;
};

// MARK: Math utilities

inline float triangleAreaPx(const TrianglePx& t) {
    // Shoelace formula, absolute area in pixel^2
    return 0.5f * std::fabs(t.a.x * (t.b.y - t.c.y) +
                            t.b.x * (t.c.y - t.a.y) +
                            t.c.x * (t.a.y - t.b.y));
}

static inline float cross2(const simd_int2 p,
                           const simd_int2 q,
                           const simd_int2 r)
{
    const simd_int2 qp = q - p;
    const simd_int2 rp = r - p;
    return qp.x * rp.y - qp.y * rp.x;
}

inline void triangleAABB(const TrianglePx& triangle,
                         simd_int2& triangleBoxMin,
                         simd_int2& triangleBoxMax) {
    triangleBoxMin.x = std::min({triangle.a.x, triangle.b.x, triangle.c.x});
    triangleBoxMax.x = std::max({triangle.a.x, triangle.b.x, triangle.c.x});
    triangleBoxMin.y = std::min({triangle.a.y, triangle.b.y, triangle.c.y});
    triangleBoxMax.y = std::max({triangle.a.y, triangle.b.y, triangle.c.y});
}

static inline bool pointInTriangleInclusive(const TrianglePx& tri,
                                            const simd_int2 pt)
{
    const float edge0 = cross2(tri.a, tri.b, pt);
    const float edge1 = cross2(tri.b, tri.c, pt);
    const float edge2 = cross2(tri.c, tri.a, pt);

    const bool allNonNegative = (edge0 >= 0.0f) && (edge1 >= 0.0f) && (edge2 >= 0.0f);
    const bool allNonPositive = (edge0 <= 0.0f) && (edge1 <= 0.0f) && (edge2 <= 0.0f);
    return allNonNegative || allNonPositive;
}

// Conservative triangle–AABB overlap in pixel space.
// boxMin: (minX, minY), boxMax: (maxX, maxY), inclusive bounds.
static inline bool triangleOverlapsBox(const TrianglePx& tri,
                                       const simd_int2 boxMin,
                                       const simd_int2 boxMax)
{
    // 1) Quick reject via AABB–AABB
    simd_int2 triangleBoxMin;
    simd_int2 triangleBoxMax;
    triangleAABB(tri, triangleBoxMin, triangleBoxMax);

    if (triangleBoxMax.x < boxMin.x || triangleBoxMin.x > boxMax.x ||
        triangleBoxMax.y < boxMin.y || triangleBoxMin.y > boxMax.y) {
        return false;
    }

    // 2) Corner inclusion test (accept if any box corner is inside the triangle)
    const simd_int2 bl = {boxMin.x, boxMin.y}; // bottom-left
    const simd_int2 br = {boxMax.x, boxMin.y}; // bottom-right
    const simd_int2 tl = {boxMin.x, boxMax.y}; // top-left
    const simd_int2 tr = {boxMax.x, boxMax.y}; // top-right

    if (pointInTriangleInclusive(tri, bl) ||
        pointInTriangleInclusive(tri, br) ||
        pointInTriangleInclusive(tri, tl) ||
        pointInTriangleInclusive(tri, tr)) {
        return true;
    }

    // 3) Conservative fallback after overlapping AABBs:
    // Count as overlapping. This intentionally overcounts a little,
    // which is useful for binning-pressure metrics.
    return true;
}

// nth-element percentile (returns P in [0,100], e.g., 95 → P95)
// Empty input returns 0. For 1 element, returns that element.
template <class T>
inline double percentile_inplace(std::vector<T>& v, double P) {
    if (v.empty()) {
        return 0.0;
    };
    if (v.size() == 1) {
        return double(v[0]);
    }
    P = std::clamp(P, 0.0, 100.0);
    const size_t k = size_t(std::floor((P / 100.0) * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return double(v[k]);
}

template <class T>
inline double mean(std::vector<T>& v) {
    if (v.empty()) {
        return 0.0;
    };
    if (v.size() == 1) {
        return double(v[0]);
    }

    size_t size = v.size();
    double sum = 0;
    for (const auto& elem: v) {
        sum += elem;
    }
    return sum / size;
}

template <class T>
inline double median(std::vector<T>& v) {
    if (v.empty()) {
        return 0.0;
    };
    if (v.size() == 1) {
        return double(v[0]);
    }

    size_t size = v.size();
    if (size % 2 == 1) {
        return v[size / 2];
    } else {
        return (v[size / 2 - 1] + v[size / 2]) / 2;
    }
}

// ------------------------ core computation ------------------------
//
// compute_metrics:
//   Input: list of triangles in *pixel* space, framebuffer (W,H), tile size (Tw,Th)
//   Output: Metrics { TTO, HTP_P95, SS_P95, BCI, ... }
//
inline Metrics compute_metrics(const std::vector<TrianglePx>& triangles,
                               const simd_int2 framebuffer,
                               const simd_int2 tileSize) {
    Metrics out{};
    if (framebuffer.x == 0 || framebuffer.y == 0 || triangles.empty()) {
        return out;
    }

    const simd_int2 maxTilePos = (framebuffer - 1) / tileSize;
    std::vector<uint32_t> tilesPerTriangle(triangles.size(), 0);

    struct TileKey {
        int x;
        int y;
    };
    struct TileKeyHash {
        size_t operator()(const TileKey& key) const noexcept {
            return (uint64_t(uint32_t(key.x)) << 32) ^ uint32_t(key.y);
        }
    };
    struct TileKeyEq { bool operator()(const TileKey& lhs, const TileKey& rhs) const noexcept {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }};

    std::unordered_map<TileKey, std::unordered_set<uint32_t>, TileKeyHash, TileKeyEq> trisPerTile;
    trisPerTile.reserve(size_t((maxTilePos.x + 1) * (maxTilePos.y + 1)));

    double sumAreaPx = 0.0;

    for (uint32_t i = 0; i < triangles.size(); ++i) {
        const TrianglePx& triangle = triangles[i];
        sumAreaPx += double(triangleAreaPx(triangle));

        // Triangle AABB → tile range
        simd_int2 triangleBoxMin;
        simd_int2 triangleBoxMax;
        triangleAABB(triangle, triangleBoxMin, triangleBoxMax);

        // Clip to framebuffer
        triangleBoxMin = simd_clamp(triangleBoxMin, 0, framebuffer - 1);
        triangleBoxMax = simd_clamp(triangleBoxMax, 0, framebuffer - 1);

        if (triangleBoxMin.x > triangleBoxMax.x || triangleBoxMin.y > triangleBoxMax.y) {
            continue;
        } // fully offscreen

        simd_int2 triangleMinTilePos = triangleBoxMin / tileSize;
        simd_int2 triangleMaxTilePos = triangleBoxMax / tileSize;

        triangleMinTilePos = simd_clamp(triangleMinTilePos, 0, maxTilePos);
        triangleMaxTilePos = simd_clamp(triangleMaxTilePos, 0, maxTilePos);

        uint32_t touched = 0;
        for (int ty = triangleMinTilePos.y; ty <= triangleMaxTilePos.y; ++ty) {
            int by0 = ty * tileSize.y;
            int by1 = by0 + tileSize.y - 1;
            for (int tx = triangleMinTilePos.x; tx <= triangleMaxTilePos.x; ++tx) {
                int bx0 = tx * tileSize.x;
                int bx1 = bx0 + tileSize.x - 1;

                if (!triangleOverlapsBox(triangle,
                                         simd_make_int2(bx0, by0),
                                         simd_make_int2(bx1, by1))) {
                    continue;
                }

                ++touched;
                trisPerTile[{tx, ty}].insert(i);
            }
        }
        tilesPerTriangle[i] = touched;
    }

    // TTO = sum of tiles per triangle
    uint64_t TTO_u64 = 0;
    for (auto v : tilesPerTriangle) {
        TTO_u64 += v;
    };

    // SS_P95 = P95 of tiles-per-triangle
    std::vector<uint32_t> ssVec = tilesPerTriangle;
    double SS_P95 = percentile_inplace(ssVec, 95.0);
    double SS_mean = mean(ssVec);
    double SS_median = median(ssVec);

    // HTP_P95 = P95 of triangles-per-tile
    std::vector<uint32_t> tpt;
    tpt.reserve(trisPerTile.size());
    uint64_t nonEmpty = 0;
    for (auto& kv : trisPerTile) {
        const uint32_t n = (uint32_t)kv.second.size();
        if (n > 0) { ++nonEmpty; tpt.push_back(n); }
    }
    double HTP_P95 = percentile_inplace(tpt, 95.0);
    double HTP_mean = mean(tpt);
    double HTP_median = median(tpt);

    // BCI = TTO / ( sum(area_px) / (Tw*Th) )
    // Intuition: tile hits per "area-equivalent tile".
    const double tileArea = double(tileSize.x) * double(tileSize.y);
    const double denom = (sumAreaPx > 0.0) ? (sumAreaPx / tileArea) : 1.0; // avoid div0
    const double BCI = (denom > 0.0) ? (double(TTO_u64) / denom) : 0.0;

    out.TTO           = double(TTO_u64);
    out.SS_P95        = SS_P95;
    out.SS_mean       = SS_mean;
    out.SS_median     = SS_median;
    out.HTP_P95       = HTP_P95;
    out.HTP_mean      = HTP_mean;
    out.HTP_median    = HTP_median;
    out.BCI           = BCI;
    out.triCount      = double(triangles.size());
    out.nonEmptyTiles = double(nonEmpty);
    out.sumAreaPx     = sumAreaPx;
    return out;
}

// MARK: convenience: NDC → pixel
// Converts vertex coordinates from NDC ([-1,1] x [-1,1]) to pixel coordinates

inline simd_int2 ndcToPixel(const simd_float2 ndc, const simd_int2 fb) {
    return simd_int2((ndc * 0.5f + 0.5f) * fb);
}

// Build pixel-space triangles from an indexed triangle list of NDC-space vertices.
inline std::vector<TrianglePx> build_pixel_tris_from_ndc(const std::vector<simd_float2>& ndcPositions,
                                                    const std::vector<uint32_t>& indicesTri,
                                                    const simd_int2 fb) {
    std::vector<TrianglePx> out;
    out.reserve(indicesTri.size() / 3);
    for (size_t i = 0; i + 2 < indicesTri.size(); i += 3) {
        const simd_int2 p0 = ndcToPixel(ndcPositions[indicesTri[i+0]], fb);
        const simd_int2 p1 = ndcToPixel(ndcPositions[indicesTri[i+1]], fb);
        const simd_int2 p2 = ndcToPixel(ndcPositions[indicesTri[i+2]], fb);
        out.push_back({p0, p1, p2});
    }
    return out;
}

} // namespace ttm

