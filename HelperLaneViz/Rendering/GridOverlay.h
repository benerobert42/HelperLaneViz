//
//  GridOverlay.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import <MetalKit/MetalKit.h>
#import <simd/simd.h>

#include "ShaderTypes.h"

@interface GridOverlay : NSObject
- (instancetype)initWithPSO:(id<MTLRenderPipelineState>)pso noDepth:(id<MTLDepthStencilState>)noDepth;
- (void)setCountsTexture:(id<MTLTexture>)tex
                  tilesX:(uint32_t)tilesX
                  tilesY:(uint32_t)tilesY
                   tileW:(uint32_t)tileW
                   tileH:(uint32_t)tileH;
- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)enc drawableSize:(CGSize)drawableSize;
@end
