//
//  GridOverlay.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import "GridOverlay.h"

@implementation GridOverlay {
    id<MTLRenderPipelineState> _pso;
    id<MTLDepthStencilState>   _noDepth;
}

- (instancetype)initWithPSO:(id<MTLRenderPipelineState>)pso noDepth:(id<MTLDepthStencilState>)noDepth {
    if ((self = [super init])) {
        _pso = pso;
        _noDepth = noDepth;
    }
    return self;
}

- (GridParams)createGridParamsForSegmentCount:(uint32_t)segmentCount
                                    andOrigin:(simd_float2)origin
                                     andScale:(float)scale {
    GridParams params{
        .cols = segmentCount,
        .rows = segmentCount,
        .cellSize = {2.f / segmentCount, 2.f / segmentCount},
        .origin = origin,
        .scale = scale
    };
    return params;
}

- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)enc drawableSize:(CGSize)size {
    // Build uniforms
    struct U { vector_uint2 tile, fb; float lineWidth; simd_float4 color; } u;
    u.tile = (vector_uint2){ MAX(uint32_t(enc.tileWidth),1), MAX(uint32_t(enc.tileHeight),1) };
    u.fb   = (vector_uint2){ (uint32_t)size.width, (uint32_t)size.height };
    u.lineWidth = 1.0f;
    u.color = (simd_float4){1,1,1,0.75f};

    [enc pushDebugGroup:@"GridOverlay"];
    [enc setRenderPipelineState:_pso];           // _pso == _pipes.gridPSO
    [enc setDepthStencilState:_noDepth];         // compare=Always, writes off
    [enc setTriangleFillMode:MTLTriangleFillModeFill];
    [enc setCullMode:MTLCullModeNone];
    [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [enc popDebugGroup];
}
@end
