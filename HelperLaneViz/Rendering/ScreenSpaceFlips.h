//
//  ScreenSpaceFlips.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 09..
//

#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/Euler_operations.h>

#include <functional>
#include <vector>
#include <algorithm>
#include <optional>
#include <cmath>
#include <unordered_set>

namespace ssflip {

using K    = CGAL::Exact_predicates_inexact_constructions_kernel;
using P3   = K::Point_3;
using V2   = std::array<double,2>;
using Mesh = CGAL::Surface_mesh<P3>;

// Return false if the vertex is fully clipped (caller may cull those edges early).
using ProjectToPixel = std::function<bool(const P3& p, V2& outPx)>;

struct Params {
    int framebufferWidth  = 1920;
    int framebufferHeight = 1080;
    int tileWidth         = 32;
    int tileHeight        = 32;
    double minTriAreaPx   = 1.0;      // reject degenerate projected triangles
    double maxDihedralDeg = 8.0;      // only flip near-planar quads
    double minGain        = 0.5;      // hysteresis: require at least this many "tile units" improvement
};

struct Stats {
    std::size_t edgesVisited    = 0;
    std::size_t candidatesKept  = 0;
    std::size_t flipsApplied    = 0;
};

inline double orient2d(const V2& a, const V2& b, const V2& c) {
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
}

inline bool triAreaOk(const V2& a, const V2& b, const V2& c, double minArea) {
    return std::abs(0.5 * orient2d(a,b,c)) >= minArea;
}

inline bool segmentIntersect(const V2& a, const V2& b, const V2& c, const V2& d) {
    auto sgn = [](double v){ return (v>0) - (v<0); };
    double o1 = orient2d(a,b,c);
    double o2 = orient2d(a,b,d);
    double o3 = orient2d(c,d,a);
    double o4 = orient2d(c,d,b);
    return (sgn(o1)*sgn(o2) < 0) && (sgn(o3)*sgn(o4) < 0);
}

inline double dihedralDeg(const P3& a, const P3& b, const P3& c, const P3& d) {
    auto nrm = [](const P3& p0, const P3& p1, const P3& p2){
        const K::Vector_3 u = p1 - p0;
        const K::Vector_3 v = p2 - p0;
        K::Vector_3 n = CGAL::cross_product(u, v);
        const double len = std::sqrt(n.squared_length());
        return len > 0 ? n / len : K::Vector_3(0,0,0);
    };
    const auto n1 = nrm(a,b,c);
    const auto n2 = nrm(a,b,d);
    double cang = std::max(-1.0, std::min(1.0, (n1*n2) ));
    return std::acos(cang) * 180.0 / M_PI;
}

struct TriPxAABB {
    double minx, miny, maxx, maxy;
};

inline TriPxAABB triAabb(const V2& p0, const V2& p1, const V2& p2) {
    const double minx = std::min({p0[0], p1[0], p2[0]});
    const double miny = std::min({p0[1], p1[1], p2[1]});
    const double maxx = std::max({p0[0], p1[0], p2[0]});
    const double maxy = std::max({p0[1], p1[1], p2[1]});
    return {minx,miny,maxx,maxy};
}

inline double tileCostFromAABB(const TriPxAABB& bb, const Params& p) {
    const double w = std::max(0.0, std::ceil((bb.maxx - bb.minx) / p.tileWidth));
    const double h = std::max(0.0, std::ceil((bb.maxy - bb.miny) / p.tileHeight));
    return w * h;
}

struct EdgeGain {
    Mesh::Edge_index e;
    double gain;
    Mesh::Vertex_index va, vb, vc, vd; // quad verts for conflict filtering
};

static inline bool is_triangle_face(Mesh::Face_index f, const Mesh& m) {
    if (f == Mesh::null_face()) return false;
    int n = 0;
    for (auto he : CGAL::halfedges_around_face(halfedge(f, m), m)) ++n;
    return n == 3;
}

void try_flip_edge(Mesh& m, Mesh::Edge_index e) {
    if (CGAL::is_border(e, m)) return;

    auto h  = halfedge(e, m);
    auto f0 = face(h, m);
    auto f1 = face(opposite(h, m), m);
    if (!is_triangle_face(f0, m) || !is_triangle_face(f1, m)) return;

    // your additional guards (convexity, dihedral, etc.) go here...

    CGAL::Euler::flip_edge(h, m);
}

inline Stats flipPass(Mesh& mesh, const ProjectToPixel& project, const Params& params) {
    Stats stats{};
    auto vprop = get(CGAL::vertex_point, mesh);

    std::vector<EdgeGain> candidates;
    candidates.reserve(num_edges(mesh));

    auto inFB = [&](const V2& p){
        return p[0] >= 0 && p[1] >= 0 && p[0] < params.framebufferWidth && p[1] < params.framebufferHeight;
    };

    for (auto e : mesh.edges()) {
        ++stats.edgesVisited;
        if (mesh.is_border(e)) continue;

        // Halfedge orientation: h = (a->b), triangles (a,b,c) and (b,a,d)
        auto h  = mesh.halfedge(e);
        auto ha = h;
        auto hb = mesh.opposite(h);

        const auto a = mesh.source(ha);
        const auto b = mesh.target(ha);
        const auto c = mesh.target(mesh.next(ha));
        const auto d = mesh.target(mesh.next(hb));

        const P3& A = vprop[a];
        const P3& B = vprop[b];
        const P3& C = vprop[c];
        const P3& D = vprop[d];

        // Planarity guard
        if (dihedralDeg(A,B,C,D) > params.maxDihedralDeg) continue;

        V2 aPx,bPx,cPx,dPx;
        if (!project(A,aPx) || !project(B,bPx) || !project(C,cPx) || !project(D,dPx))
            continue;
        if (!(inFB(aPx) || inFB(bPx) || inFB(cPx) || inFB(dPx)))
            continue;

        // Screen-space convex quad and non-degenerate tris
        const bool convex = segmentIntersect(aPx,cPx, bPx,dPx);
        if (!convex) continue;

        if (!triAreaOk(aPx,bPx,cPx, params.minTriAreaPx)) continue;
        if (!triAreaOk(aPx,bPx,dPx, params.minTriAreaPx)) continue;

        // Costs: current (abc + abd) vs flipped (cda + cdb)
        const double C_now =
            tileCostFromAABB(triAabb(aPx,bPx,cPx), params) +
            tileCostFromAABB(triAabb(aPx,bPx,dPx), params);

        const double C_flip =
            tileCostFromAABB(triAabb(cPx,dPx,aPx), params) +
            tileCostFromAABB(triAabb(cPx,dPx,bPx), params);

        const double gain = C_now - C_flip;
        if (gain > params.minGain) {
            candidates.push_back({e, gain, a, b, c, d});
            ++stats.candidatesKept;
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const EdgeGain& x, const EdgeGain& y){ return x.gain > y.gain; });

    std::unordered_set<Mesh::Vertex_index> used;
    used.reserve(candidates.size() * 2);

    for (const auto& cand : candidates) {
        if (used.count(cand.va) || used.count(cand.vb) ||
            used.count(cand.vc) || used.count(cand.vd))
            continue;

        auto h  = mesh.halfedge(cand.e);
        auto f0 = face(h, mesh);
        auto f1 = face(opposite(h, mesh), mesh);
        if (!is_triangle_face(f0, mesh) || !is_triangle_face(f1, mesh)) continue;

        // Flip
        CGAL::Euler::flip_edge(h, mesh);
        used.insert(cand.va);
        used.insert(cand.vb);
        used.insert(cand.vc);
        used.insert(cand.vd);
        ++stats.flipsApplied;
    }

    return stats;
}

} // namespace ssflip

