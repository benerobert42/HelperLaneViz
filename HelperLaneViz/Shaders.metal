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

vertex RasterizerData
vertexShader(uint vertexID [[ vertex_id ]],
             const device Vertex* vertices [[ buffer(VertexInputIndexVertices) ]],
             constant FrameConstants &frameConstants [[ buffer(VertexInputIndexFrameConstants) ]])
{
    RasterizerData out;

//    float2 pixelPosition = float2(vertices[vertexID].position.xy);
//    const vector_float2 floatViewport = vector_float2(viewportSize);
//
//    const vector_float2 topDownClipSpacePosition =
//        (pixelPosition / (floatViewport / 2.0)) - 1.0;

    float3 position = vertices[vertexID].position;
    out.position = frameConstants.viewProjectionMatrix * float4(position.x, position.y, position.z, 1.0);
    return out;
}

fragment float4 fragmentShader(RasterizerData in [[stage_in]])
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

    return colors[sum];
}
