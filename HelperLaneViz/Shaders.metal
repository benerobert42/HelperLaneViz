//
//  Shaders.metal
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#include <metal_stdlib>
using namespace metal;

#include "ShaderTypes.h"

struct RasterizerData {
    float4 position [[position]];
    float3 normal;
};

struct HelperCounter {
    atomic_uint helpers;
};

vertex RasterizerData
vertexShader(uint vertexID [[ vertex_id ]],
             uint instanceID [[instance_id]],
             const device Vertex* vertices [[ buffer(VertexInputIndexVertices) ]],
             constant FrameConstants &frameConstants [[ buffer(VertexInputIndexFrameConstants) ]],
             constant GridParams& gridParams [[ buffer(2) ]])
{
    RasterizerData out;

    uint col = instanceID % gridParams.cols;
    uint row = instanceID / gridParams.cols;

    float2 base = float2(col, row) * gridParams.cellSize + gridParams.origin;
    float2 p2 = base + vertices[vertexID].position.xy * gridParams.scale;

    // assume positions are in NDC already or pre-mapped; otherwise apply a proj*view.
    float4 position = float4(p2, vertices[vertexID].position.z, 1.0);;
    out.position = frameConstants.viewProjectionMatrix * position;
    return out;
}

fragment float4 fragmentShader(RasterizerData in [[stage_in]],
                               device HelperCounter* counter [[buffer(0)]])
{
    int h = simd_is_helper_thread() ? 1 : 0;

    // Count helper threads in this 2x2 quad
    int sum =
        quad_shuffle(h, 0) +
        quad_shuffle(h, 1) +
        quad_shuffle(h, 2) +
        quad_shuffle(h, 3);

    float4 colors[4] = {
        float4(0.0, 1.0, 0.0, 1.0),  // 0 helpers → green
        float4(1.0, 1.0, 0.0, 1.0),  // 1 helper → yellow
        float4(1.0, 0.5, 0.1, 1.0),  // 2 helpers → orange
        float4(1.0, 0.0, 0.0, 1.0)   // 3 helpers → red
    };

    atomic_fetch_add_explicit(&counter->helpers, sum, memory_order_relaxed);

    return colors[sum];
}

// Overdraw fragment: adds a small gray value each time a fragment lands
fragment float4 overdrawFragment(RasterizerData in [[stage_in]])
{
    constexpr float kIncrement = 1.0;   // visualize up to ~8 layers before clamping to white
    return float4(kIncrement, kIncrement, kIncrement, kIncrement);
}
