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

struct TrianglePx {
    simd_int2 a;
    simd_int2 b;
    simd_int2 c;
};

struct Metrics {
    double totalTilesTouched = 0.0;

    double trianglesPerTileP95   = 0.0;
    double trianglesPerTileMean  = 0.0;
    double trianglesPerTileMedian= 0.0;

    double tilesPerTriangleP95   = 0.0;
    double tilesPerTriangleMean  = 0.0;
    double tilesPerTriangleMedian= 0.0;

    double binningCostIndex      = 0.0;   // totalTilesTouched / (sum(area_px)/tile_area)

    double triangleCount         = 0.0;
    double nonEmptyTileCount     = 0.0;
    double sumAreaPx             = 0.0;
};

// ---- geometry helpers (pixel space) ----

inline float triangleAreaPx(const TrianglePx& t) {
    return 0.5f * std::fabs(t.a.x * (t.b.y - t.c.y) +
                            t.b.x * (t.c.y - t.a.y) +
                            t.c.x * (t.a.y - t.b.y));
}

static inline float cross2d(simd_int2 p, simd_int2 q, simd_int2 r) {
    const simd_int2 qp = q - p;
    const simd_int2 rp = r - p;
    return qp.x * rp.y - qp.y * rp.x;
}

inline void triangleAabb(const TrianglePx& tri, simd_int2& outMin, simd_int2& outMax) {
    outMin.x = std::min({tri.a.x, tri.b.x, tri.c.x});
    outMax.x = std::max({tri.a.x, tri.b.x, tri.c.x});
    outMin.y = std::min({tri.a.y, tri.b.y, tri.c.y});
    outMax.y = std::max({tri.a.y, tri.b.y, tri.c.y});
}

static inline bool pointInTriangleInclusive(const TrianglePx& tri, simd_int2 p) {
    const float e0 = cross2d(tri.a, tri.b, p);
    const float e1 = cross2d(tri.b, tri.c, p);
    const float e2 = cross2d(tri.c, tri.a, p);
    const bool nn = (e0 >= 0.0f) && (e1 >= 0.0f) && (e2 >= 0.0f);
    const bool pp = (e0 <= 0.0f) && (e1 <= 0.0f) && (e2 <= 0.0f);
    return nn || pp;
}

// Conservative triangle–AABB overlap (inclusive box bounds).
static inline bool triangleOverlapsBox(const TrianglePx& tri, simd_int2 boxMin, simd_int2 boxMax) {
    simd_int2 triMin, triMax;
    triangleAabb(tri, triMin, triMax);

    if (triMax.x < boxMin.x || triMin.x > boxMax.x ||
        triMax.y < boxMin.y || triMin.y > boxMax.y) {
        return false;
    }

    const simd_int2 bl = { boxMin.x, boxMin.y };
    const simd_int2 br = { boxMax.x, boxMin.y };
    const simd_int2 tl = { boxMin.x, boxMax.y };
    const simd_int2 tr = { boxMax.x, boxMax.y };

    if (pointInTriangleInclusive(tri, bl) ||
        pointInTriangleInclusive(tri, br) ||
        pointInTriangleInclusive(tri, tl) ||
        pointInTriangleInclusive(tri, tr)) {
        return true;
    }

    return true;
}

// ---- simple stats (in-place where useful) ----

template <class T>
inline double percentileInPlace(std::vector<T>& v, double p) {
    if (v.empty()) return 0.0;
    if (v.size() == 1) return double(v[0]);
    p = std::clamp(p, 0.0, 100.0);
    const size_t k = size_t(std::floor((p / 100.0) * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return double(v[k]);
}

template <class T>
inline double mean(const std::vector<T>& v) {
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (const auto& x : v) s += x;
    return s / double(v.size());
}

template <class T>
inline double medianInPlace(std::vector<T>& v) {
    if (v.empty()) return 0.0;
    const size_t n = v.size();
    const size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    const double hi = double(v[mid]);
    if ((n % 2) == 1) return hi;
    const auto maxIt = std::max_element(v.begin(), v.begin() + mid);
    return (hi + double(*maxIt)) * 0.5;
}

// ---- core computation ----

inline Metrics computeTileMetrics(const std::vector<TrianglePx>& tris,
                                  simd_int2 framebufferPx,
                                  simd_int2 tileSizePx)
{
    Metrics m{};
    if (framebufferPx.x <= 0 || framebufferPx.y <= 0 || tris.empty()) {
        return m;
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
        const TrianglePx& t = tris[i];
        sumAreaPx += double(triangleAreaPx(t));

        simd_int2 triMin, triMax;
        triangleAabb(t, triMin, triMax);

        triMin = simd_clamp(triMin, 0, framebufferPx - 1);
        triMax = simd_clamp(triMax, 0, framebufferPx - 1);
        if (triMin.x > triMax.x || triMin.y > triMax.y) continue;

        simd_int2 minTile = simd_clamp(triMin / tileSizePx, 0, maxTile);
        simd_int2 maxTileForTri = simd_clamp(triMax / tileSizePx, 0, maxTile);

        uint32_t touched = 0;
        for (int ty = minTile.y; ty <= maxTileForTri.y; ++ty) {
            const int by0 = ty * tileSizePx.y;
            const int by1 = by0 + tileSizePx.y - 1;
            for (int tx = minTile.x; tx <= maxTileForTri.x; ++tx) {
                const int bx0 = tx * tileSizePx.x;
                const int bx1 = bx0 + tileSizePx.x - 1;

                if (!triangleOverlapsBox(t, simd_make_int2(bx0, by0), simd_make_int2(bx1, by1))) continue;
                ++touched;
                triSetPerTile[{tx, ty}].insert(i);
            }
        }
        tilesPerTri[i] = touched;
    }

    uint64_t tto = 0;
    for (auto v : tilesPerTri) tto += v;

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

    m.totalTilesTouched       = double(tto);
    m.tilesPerTriangleP95     = ssP95;
    m.tilesPerTriangleMean    = ssMean;
    m.tilesPerTriangleMedian  = ssMedian;
    m.trianglesPerTileP95     = tptP95;
    m.trianglesPerTileMean    = tptMean;
    m.trianglesPerTileMedian  = tptMedian;
    m.binningCostIndex        = bci;
    m.triangleCount           = double(tris.size());
    m.nonEmptyTileCount       = double(nonEmpty);
    m.sumAreaPx               = sumAreaPx;
    return m;
}

// NDC → pixel helpers

inline simd_int2 ndcToPixel(simd_float2 ndc, simd_int2 fb) {
    return simd_int2((ndc * 0.5f + 0.5f) * fb);
}

inline std::vector<TrianglePx>
buildPixelTrianglesFromNDC(const std::vector<simd_float2>& ndcPositions,
                           const std::vector<uint32_t>& indicesTri,
                           simd_int2 fb)
{
    std::vector<TrianglePx> out;
    out.reserve(indicesTri.size() / 3);
    for (size_t i = 0; i + 2 < indicesTri.size(); i += 3) {
        const simd_int2 p0 = ndcToPixel(ndcPositions[indicesTri[i + 0]], fb);
        const simd_int2 p1 = ndcToPixel(ndcPositions[indicesTri[i + 1]], fb);
        const simd_int2 p2 = ndcToPixel(ndcPositions[indicesTri[i + 2]], fb);
        out.push_back({ p0, p1, p2 });
    }
    return out;
}

} // namespace ttm
