//
//  CGALTriangulator.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 10. 26..
//

#pragma once
#include <string>

enum class TriMode { Delaunay, MWT };

namespace CGALTriangulator {
bool TriangulateVertexOnlyEllipsoidOBJ(const std::string& in_path,
                                       const std::string& out_path,
                                       TriMode mode,
                                       int stacks = -1,
                                       int slices = -1);
}
