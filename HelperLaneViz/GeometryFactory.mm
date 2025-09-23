//
//  GeometryFactory.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 09. 23..
//

#include "GeometryFactory.h"

#include <simd/simd.h>

std::vector<Vertex> GeometryFactory::CreateVerticesForCircle(int numVertices, float radius) {
    constexpr float zCoord = 1.0;
    constexpr simd_float2 center = {0, 0};

    std::vector<Vertex> vertices(numVertices);
    for (uint32_t i = 0; i < numVertices; ++i) {
        const float t = (2.0f * M_PI * i) / numVertices;
        const float cx = center.x + radius * cosf(t);
        const float cy = center.y + radius * sinf(t);
        vertices[i] = Vertex({cx, cy, zCoord});
    }
    return vertices;
}
