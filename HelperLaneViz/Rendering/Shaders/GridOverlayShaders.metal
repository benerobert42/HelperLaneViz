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

fragment float4 gridFS(GridVSOut in [[stage_in]],
                       constant GridUniforms& uniforms [[buffer(0)]]) {
    const float2 fragPxF = in.uv * float2(uniforms.framebuffer);
    const uint2  fragPx = uint2(floor(fragPxF));
    if (fragPx.x >= uniforms.framebuffer.x || fragPx.y >= uniforms.framebuffer.y)
        return float4(0.0);

    const uint2 tile = max(uniforms.tileSize, uint2(1,1));

    // Grid lines
    const uint modX = fragPx.x % tile.x;
    const uint modY = fragPx.y % tile.y;
    const uint distX = min(modX, tile.x - modX);
    const uint distY = min(modY, tile.y - modY);
    const float thresh = max(uniforms.lineWidth, 1.0);
    const bool  vLine  = (float(distX) < thresh);
    const bool  hLine  = (float(distY) < thresh);
    return (vLine || hLine) ? uniforms.lineColor : float4(0.0);
}
