//
//  TriangulationMetrics.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#include "TriangulationMetrics.h"
#include "Triangulation.h"
#include "TileMetrics.h"

namespace TriMetrics {

static std::vector<ttm::TrianglePx>
toTrianglesPx(const std::vector<Vertex>& V,
              const std::vector<uint32_t>& I,
              simd_int2 fb)
{
    std::vector<simd_float2> ndc;
    ndc.reserve(V.size());
    for (const auto& v : V) {
        ndc.push_back(simd_make_float2(v.position.x, v.position.y));
    }
    return ttm::buildPixelTrianglesFromNDC(ndc, I, fb);
}

void printEdgeAndTileMetrics(const std::vector<Vertex>& V_mwt,
                             const std::vector<uint32_t>& I_mwt,
                             const std::vector<Vertex>& V_del,
                             const std::vector<uint32_t>& I_del,
                             simd_float4x4 VP,
                             simd_int2 fb,
                             simd_int2 ts)
{
    auto S_mwt = TriangleFactory::interior_edge_set(I_mwt);
    auto S_del = TriangleFactory::interior_edge_set(I_del);

    size_t common = 0;
    for (const auto& e : S_mwt) if (S_del.count(e)) ++common;

    const size_t uni = S_mwt.size() + S_del.size() - common;
    const double jaccard = uni ? double(common) / double(uni) : 1.0;

    printf("[EDGES] interior: MWT=%zu Delaunay=%zu common=%zu Jaccard=%.3f\n",
           S_mwt.size(), S_del.size(), common, jaccard);

    const double Lpx_mwt = TriangleFactory::interior_projected_length_px(V_mwt, I_mwt, VP, fb);
    const double Lpx_del = TriangleFactory::interior_projected_length_px(V_del, I_del, VP, fb);
    printf("[EDGES] projected_px: MWT=%.2f Delaunay=%.2f Δ=%.2f (px)\n",
           Lpx_mwt, Lpx_del, Lpx_mwt - Lpx_del);

    const double L3_mwt = TriangleFactory::interior_length_3d(V_mwt, I_mwt);
    const double L3_del = TriangleFactory::interior_length_3d(V_del, I_del);
    printf("[EDGES] interior_3D:  MWT=%.6f Delaunay=%.6f Δ=%.6f\n",
           L3_mwt, L3_del, L3_mwt - L3_del);

    const double U3_mwt = TriangleFactory::unique_length_3d(V_mwt, I_mwt);
    const double U3_del = TriangleFactory::unique_length_3d(V_del, I_del);
    printf("[EDGES] unique_3D:    MWT=%.6f Delaunay=%.6f Δ=%.6f\n",
           U3_mwt, U3_del, U3_mwt - U3_del);

    const auto triPx = toTrianglesPx(V_mwt, I_mwt, fb);
    const ttm::Metrics m = ttm::computeTileMetrics(triPx, fb, ts);

    printf("[TTM] TTO=%.0f TPT_P95=%.1f TPT_mean=%.1f TPT_median=%.1f "
           "SPT_P95=%.1f SPT_mean=%.1f SPT_median=%.1f BCI=%.3f "
           "(tris=%.0f tiles=%dx%d)\n",
           m.totalTilesTouched,
           m.trianglesPerTileP95,  m.trianglesPerTileMean,  m.trianglesPerTileMedian,
           m.tilesPerTriangleP95,  m.tilesPerTriangleMean,  m.tilesPerTriangleMedian,
           m.binningCostIndex,
           m.triangleCount, ts.x, ts.y);
}

double GetCumulativeEdgeLength2D(const std::vector<Vertex>& V,
                                 const std::vector<uint32_t>& I) {
    std::unordered_set<uint64_t> edges;
    edges.reserve(I.size());

    auto key = [](uint32_t a, uint32_t b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (uint64_t(a) << 32) | uint64_t(b);
    };

    const size_t T = I.size() / 3;
    for (size_t t = 0; t < T; ++t) {
        uint32_t i0 = I[3*t+0], i1 = I[3*t+1], i2 = I[3*t+2];
        edges.insert(key(i0, i1));
        edges.insert(key(i1, i2));
        edges.insert(key(i2, i0));
    }

    double sum = 0.0;
    for (uint64_t k : edges) {
        uint32_t a = uint32_t(k >> 32), b = uint32_t(k & 0xffffffffu);
        const auto& A = V[a].position;
        const auto& B = V[b].position;
        const double dx = double(B.x - A.x);
        const double dy = double(B.y - A.y);
        sum += std::sqrt(dx*dx + dy*dy);
    }
    return sum;
}

void Print2DMeshMetrics(const std::vector<Vertex>& V,
                        const std::vector<uint32_t>& I,
                        simd_int2 framebufferPx,
                        simd_int2 tileSizePx)
{
    const auto triPx = toTrianglesPx(V, I, framebufferPx);
    const ttm::Metrics m = ttm::computeTileMetrics(triPx, framebufferPx, tileSizePx);
    const double cumLen = GetCumulativeEdgeLength2D(V, I);

    // TTO  HTP_95  HTP mean  HTP median  SS P95  SS mean  SS median  BCI  cumulatededgelength
    printf("[2D] TTO=%.0f  HTP_P95=%.1f  HTP_mean=%.1f  HTP_median=%.1f  "
           "SS_P95=%.1f  SS_mean=%.1f  SS_median=%.1f  BCI=%.3f  cumulatededgelength=%.6f\n",
           m.totalTilesTouched,
           m.trianglesPerTileP95,  m.trianglesPerTileMean,  m.trianglesPerTileMedian,
           m.tilesPerTriangleP95,  m.tilesPerTriangleMean,  m.tilesPerTriangleMedian,
           m.binningCostIndex,
           cumLen);
}

} // namespace TriMetrics
