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

namespace TileMetrics {

struct TrianglePx {
    simd_int2 a;
    simd_int2 b;
    simd_int2 c;
};

struct Metrics {
    double totalTilesTouched = 0.0;      // Sum over all triangles of how many tiles each triangle overlaps (Σ tilesPerTriangle[i]).

    double trianglesPerTileP95 = 0.0;    // 95th percentile of “#triangles touching a tile”, over non-empty tiles.
    double trianglesPerTileMean = 0.0;   // Mean of “#triangles touching a tile”, over non-empty tiles.
    double trianglesPerTileMedian = 0.0; // Median of “#triangles touching a tile”, over non-empty tiles.

    double tilesPerTriangleP95 = 0.0;    // 95th percentile of “#tiles overlapped by a triangle”, over all triangles.
    double tilesPerTriangleMean = 0.0;   // Mean of “#tiles overlapped by a triangle”, over all triangles.
    double tilesPerTriangleMedian = 0.0; // Median of “#tiles overlapped by a triangle”, over all triangles.

    double binningCostIndex = 0.0;       // Normalized binning cost:
                                         // totalTilesTouched / (sumAreaPx / tileArea).
                                         // Roughly: tile overlaps per ideal tile-coverage unit.

    double triangleCount = 0.0;          // Total number of triangles in the input.
    double nonEmptyTileCount = 0.0;      // Number of tiles that have at least one triangle touching them.
    double sumAreaPx = 0.0;              // Sum of triangle areas in pixel space.
};

// MARK: Geometry helpers (pixel space)

float GetTriangleAreaPx(const TrianglePx& t) {
    return 0.5f * std::fabs(t.a.x * (t.b.y - t.c.y) +
                            t.b.x * (t.c.y - t.a.y) +
                            t.c.x * (t.a.y - t.b.y));
}

float Cross2d(simd_int2 p, simd_int2 q, simd_int2 r) {
    const simd_int2 vectorPq = q - p;
    const simd_int2 vectorPr = r - p;
    return vectorPq.x * vectorPr.y - vectorPq.y * vectorPr.x;
}

void TriangleAabb(const TrianglePx& tri, simd_int2& outMin, simd_int2& outMax) {
    outMin.x = std::min({tri.a.x, tri.b.x, tri.c.x});
    outMax.x = std::max({tri.a.x, tri.b.x, tri.c.x});
    outMin.y = std::min({tri.a.y, tri.b.y, tri.c.y});
    outMax.y = std::max({tri.a.y, tri.b.y, tri.c.y});
}

bool PointInTriangleInclusive(const TrianglePx& tri, simd_int2 p) {
    const float e0 = Cross2d(tri.a, tri.b, p);
    const float e1 = Cross2d(tri.b, tri.c, p);
    const float e2 = Cross2d(tri.c, tri.a, p);
    const bool nn = (e0 >= 0.0f) && (e1 >= 0.0f) && (e2 >= 0.0f);
    const bool pp = (e0 <= 0.0f) && (e1 <= 0.0f) && (e2 <= 0.0f);
    return nn || pp;
}

// Conservative triangle–AABB overlap (inclusive box bounds)
bool TriangleOverlapsBox(const TrianglePx& tri, simd_int2 boxMin, simd_int2 boxMax) {
    simd_int2 triMin, triMax;
    TriangleAabb(tri, triMin, triMax);

    if (triMax.x < boxMin.x || triMin.x > boxMax.x ||
        triMax.y < boxMin.y || triMin.y > boxMax.y) {
        return false;
    }

    return true;
}

// MARK: Simple stats

template <class T>
inline double percentileInPlace(std::vector<T>& v, double p) {
    if (v.empty()) {
        return 0.0;
    }
    if (v.size() == 1) {
        return double(v[0]);
    }

    p = std::clamp(p, 0.0, 100.0);
    const size_t k = size_t(std::floor((p / 100.0) * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return double(v[k]);
}

template <class T>
inline double mean(const std::vector<T>& v) {
    if (v.empty()) {
        return 0.0;
    }
    double s = 0.0;
    for (const auto& x : v) {
        s += x;
    }
    return s / double(v.size());
}

template <class T>
inline double medianInPlace(std::vector<T>& v) {
    if (v.empty()) {
        return 0.0;
    }

    const size_t n = v.size();
    const size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    const double hi = double(v[mid]);
    if ((n % 2) == 1) {
        return hi;
    }
    const auto maxIt = std::max_element(v.begin(), v.begin() + mid);
    return (hi + double(*maxIt)) * 0.5;
}

// MARK: core computations

Metrics ComputeTileMetrics(const std::vector<TrianglePx>& tris,
                           simd_int2 framebufferPx,
                           simd_int2 tileSizePx) {
    Metrics metrics{};
    if (framebufferPx.x <= 0 || framebufferPx.y <= 0 || tris.empty()) {
        return metrics;
    }

    const simd_int2 maxTile = (framebufferPx - 1) / tileSizePx;

    struct TileKey {
        int x;
        int y;
    };

    struct TileKeyHash {
        size_t operator()(const TileKey& k) const noexcept {
            return (uint64_t(uint32_t(k.x)) << 32) ^ uint32_t(k.y);
        }
    };

    struct TileKeyEq {
        bool operator()(const TileKey& a, const TileKey& b) const noexcept {
            return a.x == b.x && a.y == b.y;
        }
    };

    std::unordered_map<TileKey, std::unordered_set<uint32_t>, TileKeyHash, TileKeyEq> triSetPerTile;
    triSetPerTile.reserve(size_t((maxTile.x + 1) * (maxTile.y + 1)));

    std::vector<uint32_t> tilesPerTri(tris.size(), 0);
    double sumAreaPx = 0.0;

    for (uint32_t i = 0; i < tris.size(); ++i) {
        const TrianglePx& trianglePx = tris[i];
        sumAreaPx += double(GetTriangleAreaPx(trianglePx));

        simd_int2 triMin, triMax;
        TriangleAabb(trianglePx, triMin, triMax);

        triMin = simd_clamp(triMin, 0, framebufferPx - 1);
        triMax = simd_clamp(triMax, 0, framebufferPx - 1);
        if (triMin.x > triMax.x || triMin.y > triMax.y) {
            // invalid bounding box/triangle
            continue;
        }

        simd_int2 minTile = simd_clamp(triMin / tileSizePx, 0, maxTile);
        simd_int2 maxTileForTri = simd_clamp(triMax / tileSizePx, 0, maxTile);

        uint32_t touched = 0;
        for (int yTile = minTile.y; yTile <= maxTileForTri.y; ++yTile) {
            const int yTilePxBottom = yTile * tileSizePx.y;
            const int yTilePxTop = yTilePxBottom + tileSizePx.y - 1;
            for (int xTile = minTile.x; xTile <= maxTileForTri.x; ++xTile) {
                const int xTilePxLeft = xTile * tileSizePx.x;
                const int xTilePxRight = xTilePxLeft + tileSizePx.x - 1;

                if (!TriangleOverlapsBox(trianglePx,
                                         simd_make_int2(xTilePxLeft, yTilePxBottom),
                                         simd_make_int2(xTilePxRight, yTilePxTop))) {
                    continue;
                }
                ++touched;
                triSetPerTile[{xTile, yTile}].insert(i);
            }
        }
        tilesPerTri[i] = touched;
    }

    uint64_t tto = 0;
    for (auto v : tilesPerTri) {
        tto += v;
    }

    std::vector<uint32_t> ss = tilesPerTri;
    const double ssP95    = percentileInPlace(ss, 95.0);
    const double ssMean   = mean(ss);
    const double ssMedian = medianInPlace(ss);

    std::vector<uint32_t> tpt;
    tpt.reserve(triSetPerTile.size());
    uint64_t nonEmpty = 0;
    for (auto& kv : triSetPerTile) {
        const uint32_t n = (uint32_t)kv.second.size();
        if (n == 0) continue;
        ++nonEmpty;
        tpt.push_back(n);
    }
    const double tptP95    = percentileInPlace(tpt, 95.0);
    const double tptMean   = mean(tpt);
    const double tptMedian = medianInPlace(tpt);

    const double tileArea = double(tileSizePx.x) * double(tileSizePx.y);
    const double denom    = (sumAreaPx > 0.0) ? (sumAreaPx / tileArea) : 1.0;
    const double bci      = (denom > 0.0) ? (double(tto) / denom) : 0.0;

    metrics.totalTilesTouched = double(tto);
    metrics.tilesPerTriangleP95 = ssP95;
    metrics.tilesPerTriangleMean = ssMean;
    metrics.tilesPerTriangleMedian = ssMedian;
    metrics.trianglesPerTileP95 = tptP95;
    metrics.trianglesPerTileMean = tptMean;
    metrics.trianglesPerTileMedian = tptMedian;
    metrics.binningCostIndex = bci;
    metrics.triangleCount = double(tris.size());
    metrics.nonEmptyTileCount = double(nonEmpty);
    metrics.sumAreaPx = sumAreaPx;
    return metrics;
}

} // namespace TileMetrics
