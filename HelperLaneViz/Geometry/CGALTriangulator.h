//
//  CGALTriangulator.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 26..
//

#pragma once
#include <string>

#include <simd/simd.h>

enum class TriangulationMode { Delaunay, MWT };

namespace CGALTriangulator {

bool TriangulateVertexOnlyEllipsoidOBJ(const std::string& in_path,
                                       const std::string& out_path,
                                       TriangulationMode mode,
                                       int stacks = -1,
                                       int slices = -1,
                                       bool wrapColumns = true,
                                       bool applyScreenFlips = false,
                                       simd_uint2 frameBuffer = {1920, 1080},
                                       simd_float4x4 viewProjMatrix = matrix_identity_float4x4);

} // namespace CGALTriangulator
