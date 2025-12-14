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

- (instancetype)initWithPipelineState:(id<MTLRenderPipelineState>)pipelineState
                      depthStencilState:(id<MTLDepthStencilState>)depthState;

/// Draws the tile grid overlay.
/// @param encoder The render command encoder
/// @param tileSize Size of each tile in pixels
/// @param drawableSize Current drawable size
- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize;

@end
