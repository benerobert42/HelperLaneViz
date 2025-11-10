//
//  SceneShaders.metal
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 09..

#include <metal_stdlib>
using namespace metal;

#include "../ShaderTypes.h"

struct MainVSOut {
    float4 position [[position]];
};

vertex MainVSOut mainVS(uint vertexId [[vertex_id]],
                        uint instanceId [[instance_id]],
                        const device Vertex* vertices [[buffer(VertexInputIndexVertices)]],
                        constant FrameConstants& frame [[buffer(VertexInputIndexFrameConstants)]],
                        constant GridParams& grid [[buffer(VertexInputGridParams)]]) {
    const uint col = instanceId % grid.cols;
    const uint row = instanceId / grid.cols;

    const float2 cellOrigin = float2(col, row) * grid.cellSize + grid.origin;
    const float2 local = vertices[vertexId].position.xy * grid.scale;
    const float3 position = float3(cellOrigin + local, vertices[vertexId].position.z);

    MainVSOut out {.position = frame.viewProjectionMatrix * float4(position, 1.0)};
    return out;
}

fragment float4 mainFS(MainVSOut in [[stage_in]]) {
    const int isHelperThread = simd_is_helper_thread() ? 1 : 0;

    const int sum =
        quad_shuffle(isHelperThread, 0) +
        quad_shuffle(isHelperThread, 1) +
        quad_shuffle(isHelperThread, 2) +
        quad_shuffle(isHelperThread, 3);

    constexpr float4 kColors[4] = {
        float4(0.0, 1.0, 0.0, 1.0),  // 0 helpers
        float4(1.0, 1.0, 0.0, 1.0),  // 1
        float4(1.0, 0.5, 0.1, 1.0),  // 2
        float4(1.0, 0.0, 0.0, 1.0)   // 3
    };

    return kColors[clamp(sum, 0, 3)];
}

fragment float4 overdrawFS(MainVSOut in [[stage_in]]) {
    // Additive white; final intensity depends on the blend state
    return float4(1.0);
}
