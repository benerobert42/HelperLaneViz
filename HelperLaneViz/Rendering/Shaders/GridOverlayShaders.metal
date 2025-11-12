//
//  GridOverlayShaders.metal
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 09..

#include <metal_stdlib>
using namespace metal;

#include "../ShaderTypes.h"

struct GridVSOut {
    float4 position [[position]];
    float2 uv;
};

vertex GridVSOut gridVS(uint vid [[vertex_id]]) {
    const float2 pos = float2( (vid == 2) ?  3.0 : -1.0,
                               (vid == 1) ?  3.0 : -1.0 );

    GridVSOut out {.position = float4(pos, 0.0, 1.0), .uv = pos * 0.5 + 0.5};
    return out;
}

//fragment float4 gridFS(GridVSOut in [[stage_in]],
//                       constant GridUniforms& uniforms [[buffer(0)]]) {
//    const float2 fragPxF = in.uv * float2(uniforms.framebuffer);
//    const uint2  fragPx = uint2(floor(fragPxF));
//
//    if (fragPx.x >= uniforms.framebuffer.x || fragPx.y >= uniforms.framebuffer.y)
//        return float4(0.0);
//
//    const uint Tw = max(uniforms.tileSize.x, 1u);
//    const uint Th = max(uniforms.tileSize.y, 1u);
//
//    const uint modX = fragPx.x % Tw;
//    const uint modY = fragPx.y % Th;
//
//    const uint distX = min(modX, Tw - modX);
//    const uint distY = min(modY, Th - modY);
//
//    const float thresh = max(uniforms.lineWidth, 1.0);
//    const bool  vLine  = (float(distX) < thresh);
//    const bool  hLine  = (float(distY) < thresh);
//    const bool  corner = vLine && hLine;
//
//    float4 color = (vLine || hLine) ? uniforms.lineColor : float4(0.0);
//    if (corner) color.rgb = min(color.rgb + float3(0.2), float3(1.0));
//
//    return color;
//}

static float3 heat(float t) {
    // simple magma-ish palette
    t = clamp(t, 0.0f, 1.0f);
    float3 a = float3(0.2, 0.0, 0.3);
    float3 b = float3(0.8, 0.2, 0.0);
    return mix(a, b, t) + float3(0.0, 0.3, 0.0) * smoothstep(0.5f, 1.0f, t);
}

fragment float4 gridFS(
    GridVSOut in [[stage_in]],
    constant GridUniforms& uniforms [[buffer(0)]],
    texture2d<float> tileCountsTex  [[texture(0)]])      // NEW
{
    const float2 fragPxF = in.uv * float2(uniforms.framebuffer);
    const uint2  fragPx = uint2(floor(fragPxF));
    if (fragPx.x >= uniforms.framebuffer.x || fragPx.y >= uniforms.framebuffer.y)
        return float4(0.0);

    const uint2 tile = max(uniforms.tileSize, uint2(1,1));
    const uint2 tileId = uint2(fragPx.x / tile.x, fragPx.y / tile.y);

    // sample normalized count (0..1) from the per-tile texture
    constexpr sampler s(coord::pixel);
    float tNorm = tileCountsTex.read(tileId).x; // grayscale stored in .x

    // fill color by heatmap + alpha
    float4 fill = float4(heat(tNorm), uniforms.fillAlpha);

    // Grid lines (your existing logic)
    const uint modX = fragPx.x % tile.x;
    const uint modY = fragPx.y % tile.y;
    const uint distX = min(modX, tile.x - modX);
    const uint distY = min(modY, tile.y - modY);
    const float thresh = max(uniforms.lineWidth, 1.0);
    const bool  vLine  = (float(distX) < thresh);
    const bool  hLine  = (float(distY) < thresh);
    float4 line = (vLine || hLine) ? uniforms.lineColor : float4(0.0);

    // Composite: heat fill under grid lines
    float4 out = fill;
    out = mix(out, line, line.a); // assume lineColor.a carries opacity
    return out;
}
