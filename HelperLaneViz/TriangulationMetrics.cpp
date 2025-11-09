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
to_tri_px(const std::vector<Vertex>& V, const std::vector<uint32_t>& I,
          simd_int2 fb) {
    std::vector<simd_float2> ndc; ndc.reserve(V.size());
    for (auto& v : V) {
        ndc.push_back(simd_make_float2(v.position.x, v.position.y));
    }
    return ttm::build_pixel_tris_from_ndc(ndc, I, fb);
}

void print_edge_and_tile_metrics(const std::vector<Vertex>& V_mwt,
                                 const std::vector<uint32_t>& I_mwt,
                                 const std::vector<Vertex>& V_del,
                                 const std::vector<uint32_t>& I_del,
                                 simd_float4x4 VP,
                                 simd_int2 fb,
                                 simd_int2 ts) {
    // Edge-set comparison
    auto S_mwt = TriangleFactory::interior_edge_set(I_mwt);
    auto S_del = TriangleFactory::interior_edge_set(I_del);
    size_t common=0; for (auto& e : S_mwt) if (S_del.count(e)) ++common;
    const size_t uni = S_mwt.size() + S_del.size() - common;
    const double jac = uni ? double(common)/double(uni) : 1.0;

    printf("[EDGES] interior: mwt=%zu del=%zu common=%zu  Jaccard=%.3f\n",
           S_mwt.size(), S_del.size(), common, jac);

    // Projected / 3D lengths
    double Lpx_mwt = TriangleFactory::interior_projected_length_px(V_mwt, I_mwt, VP, fb);
    double Lpx_del = TriangleFactory::interior_projected_length_px(V_del, I_del, VP, fb);
    printf("[EDGES] interior_projected_px: MWT=%.2f  Delaunay=%.2f  Δ=%.2f (px)\n",
           Lpx_mwt, Lpx_del, Lpx_mwt - Lpx_del);

    double L3_mwt = TriangleFactory::interior_length_3d(V_mwt, I_mwt);
    double L3_del = TriangleFactory::interior_length_3d(V_del, I_del);
    printf("[EDGES] interior_3D:          MWT=%.6f  Delaunay=%.6f  Δ=%.6f\n",
           L3_mwt, L3_del, L3_mwt - L3_del);

    double U3_mwt = TriangleFactory::unique_length_3d(V_mwt, I_mwt);
    double U3_del = TriangleFactory::unique_length_3d(V_del, I_del);
    printf("[EDGES] unique_all_3D:        MWT=%.6f  Delaunay=%.6f  Δ=%.6f\n",
           U3_mwt, U3_del, U3_mwt - U3_del);

    // Tile metrics on one of them (or both if you like)
    auto triPx = to_tri_px(V_mwt, I_mwt, fb);
    ttm::Metrics m = ttm::compute_metrics(triPx, fb, ts);
    printf("[TTM] TTO=%.0f  HTP_P95=%.1f  HTP_mean=%.1f  HTP_median=%.1f  SS_P95=%.1f  SS_mean=%.1f  SS_median=%.1f  BCI=%.3f  (tris=%.0f tiles=%dx%d)\n",
           m.TTO, m.HTP_P95, m.HTP_mean, m.HTP_median, m.SS_P95, m.SS_mean, m.SS_median, m.BCI, m.triCount, ts.x, ts.y);
}
} // namespace

