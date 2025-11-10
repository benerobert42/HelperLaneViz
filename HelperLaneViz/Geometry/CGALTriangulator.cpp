//
//  CGALTriangulator.cpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 26..
//

#include "CGALTriangulator.h"
#include "ScreenSpaceFlips.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/IO/polygon_mesh_io.h>

#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <simd/simd.h>

namespace PMP = CGAL::Polygon_mesh_processing;
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using P3 = K::Point_3;
using Mesh = CGAL::Surface_mesh<P3>;

namespace {
bool ReadVerticesFromObj(std::string_view path, std::vector<P3>& pointsOut) {
    std::ifstream in(std::string{path});
    if (!in) return false;

    pointsOut.clear();
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == 'v' && (line.size() == 1 || std::isspace(static_cast<unsigned char>(line[1])))) {
            std::istringstream ss(line);
            char tag; double x{}, y{}, z{};
            ss >> tag >> x >> y >> z;
            if (ss) pointsOut.emplace_back(x, y, z);
        }
    }
    return !pointsOut.empty();
}

std::optional<std::pair<int, int>> DeduceGridFromCount(size_t N) {
    const int kMinSlices = 8;
    const double root = std::sqrt(static_cast<double>(N));

    int bestSlices = -1;
    int bestStacks = -1;

    for (size_t s = kMinSlices; s <= N; ++s) {
        if (N % s) {
            continue;
        }
        const size_t rows = N / s;
        if (rows < 3) {
            continue;
        }
        const int stacks = static_cast<int>(rows) - 1;
        if (stacks < 2) {
            continue;
        }

        const int slices = static_cast<int>(s);
        const bool first = (bestSlices < 0);
        const bool closer =
            std::abs(static_cast<double>(slices) - root) <
            std::abs(static_cast<double>(bestSlices < 0 ? 0 : bestSlices) - root);

        if (first || closer) {
            bestSlices = slices;
            bestStacks = stacks;
        }
    }

    if (bestSlices < 0) return std::nullopt;
    return std::pair{bestStacks, bestSlices};
}

void BuildGridQuads(int stacks, int slices,
                    std::vector<std::vector<size_t>>& quadsOut,
                    bool wrapColumns)
{
    auto idx = [&](int i, int j) -> std::size_t {
        const int jj = wrapColumns ? (j % slices) : j;
        return static_cast<size_t>(i) * static_cast<size_t>(slices)
             + static_cast<size_t>(jj);
    };

    quadsOut.clear();
    quadsOut.reserve(static_cast<size_t>(stacks) * static_cast<size_t>(slices));

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices - (wrapColumns ? 0 : 1); ++j) {
            const size_t a = idx(i, j);
            const size_t b = idx(i, j+1);
            const size_t c = idx(i+1, j+1);
            const size_t d = idx(i+1, j);
            quadsOut.push_back({a, b, c, d});
        }
    }
}

void TriangulateQuadsMWT(const std::vector<P3>& P,
                         const std::vector<std::vector<std::size_t>>& quads,
                         std::vector<std::vector<std::size_t>>& trisOut) {
    auto len2 = [&](size_t u, size_t v) {
        const auto& A = P[u];
        const auto& B = P[v];
        const double dx = B.x() - A.x();
        const double dy = B.y() - A.y();
        const double dz = B.z() - A.z();
        return dx*dx + dy*dy + dz*dz;
    };

    trisOut.clear();
    trisOut.reserve(quads.size() * 2);

    for (const auto& f : quads) {
        if (f.size() == 4) {
            const auto [a, b, c, d] = std::array<std::size_t,4>{ f[0], f[1], f[2], f[3] };
            if (len2(a, c) <= len2(b, d)) {
                trisOut.push_back({a, b, c});
                trisOut.push_back({a, c, d});
            } else {
                trisOut.push_back({b, c, d});
                trisOut.push_back({b, d, a});
            }
        } else if (f.size() == 3) {
            trisOut.push_back(f); // already a triangle (e.g., polar degenerate)
        } else if (f.size() > 4) {
            // Fallback fan (not expected for lat–long, but safe).
            for (std::size_t k = 1; k + 1 < f.size(); ++k)
                trisOut.push_back({ f[0], f[k], f[k+1] });
        }
    }
}

inline bool projectToPixel(const P3& p, ssflip::V2& outPx, const simd_float4x4 & viewProjMat, simd_uint2 frameBuffer) {
    const simd_float4 point{static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z()), 1.0};
    const simd_float4 transformedPoint = simd_mul(viewProjMat, point);
    if (transformedPoint.w <= 0.0) return false;                 // simple clip guard
    const double invW = 1.0 / transformedPoint.w;
    const double ndcX = transformedPoint.x * invW;
    const double ndcY = transformedPoint.y * invW;
    if (std::abs(ndcX) > 1.1 || std::abs(ndcY) > 1.1) return false; // culled
    outPx[0] = (ndcX * 0.5 + 0.5) * frameBuffer.x;
    outPx[1] = (ndcY * 0.5 + 0.5) * frameBuffer.y;
    return true;
}

} // namespace


bool CGALTriangulator::TriangulateVertexOnlyEllipsoidOBJ(const std::string& inPath,
                                       const std::string& outPath,
                                       TriangulationMode mode,
                                       int stacks,
                                       int slices,
                                       bool wrapColumns,
                                       bool applyScreenFlips,
                                       simd_uint2 frameBuffer,
                                       simd_float4x4 viewProjMatrix) {
    std::vector<P3> points;
    if (!ReadVerticesFromObj(inPath, points)) {
        std::cerr << "[Tri] ERROR: failed to read vertices from " << inPath << '\n';
        return false;
    }

    const std::size_t N = points.size();
    if (stacks < 0 || slices < 0) {
        auto guess = DeduceGridFromCount(N);
        if (!guess) {
            std::cerr << "[Tri] ERROR: cannot infer grid (N=" << N << "); pass stacks/slices explicitly.\n";
            return false;
        }
        stacks = guess->first;
        slices = guess->second;
    }

    if (N != static_cast<std::size_t>((stacks + 1) * slices)) {
        std::cerr << "[Tri] ERROR: N=" << N
                  << " != (stacks+1)*slices = " << ((stacks + 1) * slices) << '\n';
        return false;
    }

    // 1) Build quads from the grid
    std::vector<std::vector<std::size_t>> quads;
    BuildGridQuads(stacks, slices, quads, wrapColumns);

    // 2) Produce a CGAL mesh
    Mesh mesh;

    if (mode == TriangulationMode::Delaunay) {
        PMP::polygon_soup_to_polygon_mesh(points, quads, mesh);
        if (CGAL::is_empty(mesh)) {
            std::cerr << "[Tri] ERROR: empty mesh after soup conversion.\n";
            return false;
        }
        PMP::triangulate_faces(mesh, CGAL::parameters::use_delaunay_triangulation(true));
    } else {
        std::vector<std::vector<std::size_t>> tris;
        TriangulateQuadsMWT(points, quads, tris);

        // Ensure consistent orientation and build the mesh directly from triangles
        PMP::orient_polygon_soup(points, tris);
        PMP::polygon_soup_to_polygon_mesh(points, tris, mesh);
        if (CGAL::is_empty(mesh)) {
            std::cerr << "[Tri] ERROR: empty mesh after MWT triangulation.\n";
            return false;
        }
        // Already triangles; no need to call triangulate_faces again.
    }

    if (applyScreenFlips) {
        ssflip::Params p;
        p.framebufferWidth  = frameBuffer.x;
        p.framebufferHeight = frameBuffer.y;
        p.tileWidth         = 32;
        p.tileHeight        = 32;
        p.minTriAreaPx      = 0.5;
        p.maxDihedralDeg    = 6.0;
        p.minGain           = 1.0;

        auto projector = [&](const P3& pt, ssflip::V2& px){
            return projectToPixel(pt, px, viewProjMatrix, frameBuffer); // the projector shown earlier
        };

        constexpr int maxFlipIters = 1000;
        for (int iter = 0; iter < std::max(1, maxFlipIters); ++iter) {
            ssflip::Stats s = ssflip::flipPass(mesh, projector, p);
            // printf("[FlipPass] iter=%d visited=%zu kept=%zu applied=%zu\n",
            //        iter, s.edgesVisited, s.candidatesKept, s.flipsApplied);
            if (s.flipsApplied == 0) break; // converged
        }

        // Optional: re-validate
        // if (!CGAL::is_valid_polygon_mesh(mesh)) { /* handle */ }
    }

    if (!CGAL::is_valid_polygon_mesh(mesh)) {
        std::cerr << "[Tri] ERROR: invalid mesh after triangulation.\n";
        return false;
    }

    if (!CGAL::IO::write_polygon_mesh(std::string{outPath}, mesh,
                                      CGAL::parameters::stream_precision(17))) {
        std::cerr << "[Tri] ERROR: write failed: " << outPath << '\n';
        return false;
    }

    return true;
}
