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

    out.normal = vertices[vertexID].normal;
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

// Unused for now from here

fragment void accumulateFrag(RasterizerData in [[stage_in]],
                             constant FrameConstants &frameConstants  [[buffer(1)]],
                             device atomic_uint *C [[buffer(0)]])
{
    // Only real pixels perform increments
    uint2 vp = frameConstants.viewPortSize;
    if (!simd_is_helper_thread()) {
        int h0 = quad_shuffle(simd_is_helper_thread() ? 1:0, 0);
        int h1 = quad_shuffle(simd_is_helper_thread() ? 1:0, 1);
        int h2 = quad_shuffle(simd_is_helper_thread() ? 1:0, 2);
        int h3 = quad_shuffle(simd_is_helper_thread() ? 1:0, 3);

        float2 p0 = quad_shuffle(in.position.xy, 0);
        float2 p1 = quad_shuffle(in.position.xy, 1);
        float2 p2 = quad_shuffle(in.position.xy, 2);
        float2 p3 = quad_shuffle(in.position.xy, 3);

        auto add = [&](int h, float2 pf){
            if (h) {
                uint2 pi = uint2(pf);
                uint idx = pi.y * vp.x + pi.x;
                atomic_fetch_add_explicit(&C[idx], 1u, memory_order_relaxed);
            }
        };
        add(h0,p0); add(h1,p1); add(h2,p2); add(h3,p3);
    }
    // no color output; PSO has writeMask=0
}

vertex float4 visualizeVertex(uint vid [[vertex_id]]) {
    // 3 verts covering full screen in clip space
    float2 p = float2( (vid==0)? -1.0 : 3.0,
                       (vid==0)? -1.0 : (vid==1 ? -1.0 : 3.0) );
    return float4(p, 0.0, 1.0);
}

fragment float4 visualizeFrag(float4 pos [[position]],
                              constant FrameConstants &frameConstants [[buffer(1)]],
                              device atomic_uint *C [[buffer(0)]])
{
    uint2 vp = frameConstants.viewPortSize;
    uint2 pi = uint2(clamp(pos.xy, 0.0, float2(vp)-1.0));
    uint idx = pi.y * vp.x + pi.x;
    uint count = atomic_load_explicit(&C[idx], memory_order_relaxed);

    float t = clamp((float)count / 3.0, 0.0, 1.0);
    return float4(mix(float3(0.1,0.8,0.2), float3(0.9,0.1,0.1), t), 1.0);
}
