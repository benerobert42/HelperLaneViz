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

    id<MTLTexture> _countsTex;
    uint32_t _tilesX;
    uint32_t _tilesY;
    uint32_t _tileW;
    uint32_t _tileH;
}

- (instancetype)initWithPSO:(id<MTLRenderPipelineState>)pso noDepth:(id<MTLDepthStencilState>)noDepth {
    if ((self = [super init])) {
        _pso = pso;
        _noDepth = noDepth;
    }
    return self;
}

- (void)setCountsTexture:(id<MTLTexture>)tex
                  tilesX:(uint32_t)tilesX
                  tilesY:(uint32_t)tilesY
                   tileW:(uint32_t)tileW
                   tileH:(uint32_t)tileH {
    _countsTex = tex;
    _tilesX = tilesX;
    _tilesY = tilesY;
    _tileW  = tileW;
    _tileH  = tileH;
}

- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)enc drawableSize:(CGSize)size {
    GridUniforms u = {0}; // zero pads too
    u.tileSize    = (simd_uint2){ _tileW, _tileH };  // <-- use the values you set via setCountsTexture
    u.framebuffer = (simd_uint2){ (uint32_t)size.width, (uint32_t)size.height };
    u.lineWidth   = 1.0f;
    u.lineColor   = (simd_float4){ 1, 1, 1, 0.75f };
    u.fillAlpha   = 0.85f;

    [enc pushDebugGroup:@"GridOverlay"];
    [enc setRenderPipelineState:_pso];
    [enc setDepthStencilState:_noDepth];
    [enc setTriangleFillMode:MTLTriangleFillModeFill];
    [enc setCullMode:MTLCullModeNone];

    // Bind the per-tile counts texture HERE to be explicit
    if (_countsTex) [enc setFragmentTexture:_countsTex atIndex:0];

    [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [enc popDebugGroup];
}
@end
