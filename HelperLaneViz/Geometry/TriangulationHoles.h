//
//  TriangulationHoles.hpp
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 12. 14..
//

#pragma once

#include "ShaderTypes.h"

#include <vector>

namespace Triangulation::Helpers {

std::vector<Vertex> BuildBridgedPolygonImpl(const std::vector<Vertex>& outerVertices,
                                            const std::vector<Vertex>& holeVertices);

bool IsTriangleValidWithHoles(const std::vector<Vertex>& vertices,
                              int i,
                              int j,
                              int k,
                              const std::vector<Vertex>& outerVertices,
                              const std::vector<std::vector<Vertex>>& holes);
} // namespace Triangulation::Helpers
