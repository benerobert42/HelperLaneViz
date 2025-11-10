//
//  GridOverlayShaders.metal
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 09..

#include <metal_stdlib>
using namespace metal;

// Matches GridOverlay CPU-side struct
struct GridUniforms {
    uint2  tileSize;
    uint2  framebuffer;
    float  lineWidth;
    float4 lineColor;
};

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

fragment float4 gridFS(GridVSOut in [[stage_in]],
                       constant GridUniforms& uniforms [[buffer(0)]]) {
    const float2 fragPxF = in.uv * float2(uniforms.framebuffer);
    const uint2  fragPx = uint2(floor(fragPxF));

    if (fragPx.x >= uniforms.framebuffer.x || fragPx.y >= uniforms.framebuffer.y)
        return float4(0.0);

    const uint Tw = max(uniforms.tileSize.x, 1u);
    const uint Th = max(uniforms.tileSize.y, 1u);

    const uint modX = fragPx.x % Tw;
    const uint modY = fragPx.y % Th;

    const uint distX = min(modX, Tw - modX);
    const uint distY = min(modY, Th - modY);

    const float thresh = max(uniforms.lineWidth, 1.0);
    const bool  vLine  = (float(distX) < thresh);
    const bool  hLine  = (float(distY) < thresh);
    const bool  corner = vLine && hLine;

    float4 color = (vLine || hLine) ? uniforms.lineColor : float4(0.0);
    if (corner) color.rgb = min(color.rgb + float3(0.2), float3(1.0));

    return color;
}

