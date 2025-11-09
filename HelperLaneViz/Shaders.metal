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
    float4 position = float4(p2.xy, vertices[vertexID].position.z, 1.0);
    out.position = frameConstants.viewProjectionMatrix * position;
    return out;
}

fragment float4 fragmentShader(RasterizerData in [[stage_in]]
                               /*device HelperCounter* counter [[buffer(0)]]*/)
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

//    atomic_fetch_add_explicit(&counter->helpers, sum, memory_order_relaxed);

    return colors[sum];
}

// Overdraw fragment: adds a small gray value each time a fragment lands
fragment float4 overdrawFragment(RasterizerData in [[stage_in]])
{
    constexpr float kIncrement = 1.0;   // visualize up to ~8 layers before clamping to white
    return float4(kIncrement, kIncrement, kIncrement, kIncrement);
}

// Fullscreen triangle
vertex float4 probeVS(uint vid [[vertex_id]])
{
    float2 p = float2((vid == 2) ?  3.0 : -1.0,
                      (vid == 1) ? -3.0 :  1.0);
    return float4(p, 0.0, 1.0);
}

// Trivial fragment (keep it super cheap)
fragment float4 probeFS()
{
    return float4(0.0, 0.0, 0.0, 1.0);
}

struct GridUniforms {
    uint2 tileSize;       // (Tw, Th) from encoder.tileWidth/Height
    uint2 framebuffer;    // (W, H) drawable size in pixels
    float lineWidth;      // thickness in pixels (>= 1.0)
    float4 lineColor;     // RGBA for the grid lines
};

struct VSOut {
    float4 position [[position]];
    float2 uv;
};

vertex VSOut tilegrid_vs(uint vid [[vertex_id]])
{
    // Fullscreen triangle (no vertex buffer)
    float2 pos = float2((vid == 2) ? 3.0 : -1.0,
                        (vid == 1) ? 3.0 : -1.0);
    VSOut o;
    o.position = float4(pos, 0, 1);
    // Map NDC to [0,1] UV; used only to reconstruct pixel coords
    o.uv = float2((pos.x * 0.5f) + 0.5f, (pos.y * 0.5f) + 0.5f);
    return o;
}

fragment float4 tilegrid_fs(VSOut in                 [[stage_in]],
                            constant GridUniforms& u [[buffer(0)]])
{
    // Compute integer pixel coords from UV
    float2 fragPxF = in.uv * float2(u.framebuffer);
    uint2 fragPx   = uint2(floor(fragPxF));

    // Guard against out-of-bounds (rasterization may cover a hair more)
    if (fragPx.x >= u.framebuffer.x || fragPx.y >= u.framebuffer.y)
        return float4(0);

    // Distance to the nearest vertical tile boundary
    uint Tw = max(u.tileSize.x, 1u);
    uint Th = max(u.tileSize.y, 1u);

    uint modX = fragPx.x % Tw;
    uint modY = fragPx.y % Th;

    // Distance in pixels to the *closest* boundary on each axis
    uint distX = min(modX, Tw - modX);
    uint distY = min(modY, Th - modY);

    // Render a crosshair-style grid: line when close to a boundary
    float threshold = max(u.lineWidth, 1.0);
    bool onVertical   = (float(distX) < threshold);
    bool onHorizontal = (float(distY) < threshold);

    // Optional: make tile corners slightly brighter
    bool onCorner = onVertical && onHorizontal;

    float4 col = float4(0.0);
    if (onVertical || onHorizontal)
        col = u.lineColor;
    if (onCorner)
        col.rgb = min(col.rgb + float3(0.2), float3(1.0));

    return col;
}
