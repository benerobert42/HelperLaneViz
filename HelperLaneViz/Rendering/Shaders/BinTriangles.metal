//
//  BinTriangles.metal
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 12..
//

#include <metal_stdlib>
using namespace metal;

struct Vertex { float3 position; };               // matches your CPU Vertex
struct FrameConstants { float4x4 viewProjectionMatrix; };
struct BinUniforms {
    uint2 framebufferPx;   // drawable size
    uint2 tileSizePx;      // e.g., 32 x 32
    uint  indexCount;      // total indices
};

kernel void binTrianglesToTiles(
    constant Vertex*      verts      [[ buffer(0) ]],
    constant uint*        indices    [[ buffer(1) ]],
    constant FrameConstants& frame   [[ buffer(2) ]],
    constant BinUniforms& uni        [[ buffer(3) ]],
    device atomic_uint*   tileCounts [[ buffer(4) ]],
    uint triId                        [[ thread_position_in_grid ]])
{
    // one thread per triangle
    uint triBase = triId * 3u;
    if (triBase + 2u >= uni.indexCount) return;

    uint i0 = indices[triBase + 0];
    uint i1 = indices[triBase + 1];
    uint i2 = indices[triBase + 2];

    float4 p0 = float4(verts[i0].position, 1.0);
    float4 p1 = float4(verts[i1].position, 1.0);
    float4 p2 = float4(verts[i2].position, 1.0);

    // clip space → NDC
    float4 q0 = frame.viewProjectionMatrix * p0;
    float4 q1 = frame.viewProjectionMatrix * p1;
    float4 q2 = frame.viewProjectionMatrix * p2;

    // Drop triangles entirely behind the camera
    if (q0.w <= 0.0 && q1.w <= 0.0 && q2.w <= 0.0) return;

    float2 ndc0 = q0.xy / max(q0.w, 1e-6);
    float2 ndc1 = q1.xy / max(q1.w, 1e-6);
    float2 ndc2 = q2.xy / max(q2.w, 1e-6);

    // NDC [-1,1] → pixel
    float2 fb = float2(uni.framebufferPx);
    float2 px0 = (ndc0 * 0.5f + 0.5f) * fb;
    float2 px1 = (ndc1 * 0.5f + 0.5f) * fb;
    float2 px2 = (ndc2 * 0.5f + 0.5f) * fb;

    // Triangle AABB in pixels
    float2 pmin = floor(min(px0, min(px1, px2)));
    float2 pmax = ceil (max(px0, max(px1, px2)));

    // Convert to tile coords
    uint2 tile = uint2(max(uni.tileSizePx, uint2(1,1)));
    uint tilesX = (uni.framebufferPx.x + tile.x - 1u) / tile.x;
    uint tilesY = (uni.framebufferPx.y + tile.y - 1u) / tile.y;

    int tx0 = clamp(int(pmin.x) / int(tile.x), 0, int(tilesX) - 1);
    int ty0 = clamp(int(pmin.y) / int(tile.y), 0, int(tilesY) - 1);
    int tx1 = clamp(int(pmax.x) / int(tile.x), 0, int(tilesX) - 1);
    int ty1 = clamp(int(pmax.y) / int(tile.y), 0, int(tilesY) - 1);

    // Conservative: mark every tile touched by the triangle’s AABB
    for (int ty = ty0; ty <= ty1; ++ty) {
        uint row = uint(ty) * tilesX;
        for (int tx = tx0; tx <= tx1; ++tx) {
            atomic_fetch_add_explicit(&tileCounts[row + uint(tx)], 1u, memory_order_relaxed);
        }
    }
}

kernel void countsToTexture(
    device const uint*  tileCounts   [[ buffer(0) ]],
    constant uint2&     tilesWH      [[ buffer(1) ]], // {tilesX, tilesY}
    constant uint&      maxCount     [[ buffer(2) ]], // precomputed or scanned
    texture2d<float, access::write> outTex [[ texture(0) ]],
    uint2 tid [[ thread_position_in_grid ]])
{
    if (tid.x >= tilesWH.x || tid.y >= tilesWH.y) return;
    uint idx = tid.y * tilesWH.x + tid.x;
    float v = (maxCount > 0) ? (float(tileCounts[idx]) / float(maxCount)) : 0.0f;
    outTex.write(float4(v, v, v, 1.0), tid);
}

