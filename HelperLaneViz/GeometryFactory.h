//
//  GeometryFactory.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 09. 23..
//

#pragma once

#include "ShaderTypes.h"

#include <vector>

namespace GeometryFactory {
    std::vector<Vertex> CreateVerticesForCircle(int numVertices, float radius);
}
