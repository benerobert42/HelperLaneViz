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
                                       bool wrapColumns = true);

} // namespace CGALTriangulator
