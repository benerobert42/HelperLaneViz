//
//  Triangulation.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 21..
//

#pragma once

#include "ShaderTypes.h"

#include <simd/simd.h>
#include <vector>
#include <cstdint>

using Indices = std::vector<uint32_t>;

namespace Triangulation {
Indices EarClipping(const std::vector<Vertex>& vertices);
Indices EarClippingMapbox(const std::vector<Vertex>& vertices);
Indices EarClippingMapboxWithEdgeFlips(const std::vector<Vertex>& vertices);

Indices CentroidFan(std::vector<Vertex>& vertices);
Indices Strip(const std::vector<Vertex>& vertices);

Indices GreedyMaxArea(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);

Indices MinimumWeight(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);
Indices MaxMinArea(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);
Indices MinMaxArea(const std::vector<Vertex>& vertices, bool shouldHandleConcave = false);

Indices ConstrainedDelaunay(const std::vector<Vertex>& vertices);
Indices ConstrainedDelaunayWithEdgeFlips(const std::vector<Vertex>& vertices);

} // namespace Triangulation
