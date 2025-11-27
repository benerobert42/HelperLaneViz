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

/// Draws the tile heatmap overlay using the provided texture.
/// @param encoder The render command encoder
/// @param heatmapTexture Texture containing normalized tile counts (0-1 grayscale)
/// @param tileSize Size of each tile in pixels
/// @param drawableSize Current drawable size
- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
         heatmapTexture:(id<MTLTexture>)heatmapTexture
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize;

@end
