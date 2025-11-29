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

/// Draws the tile grid overlay, optionally with heatmap coloring.
/// @param encoder The render command encoder
/// @param heatmapTexture Texture containing normalized tile counts (0-1 grayscale), can be nil if showHeatmap is NO
/// @param tileSize Size of each tile in pixels
/// @param drawableSize Current drawable size
/// @param showHeatmap If YES, shows heatmap coloring; if NO, shows only grid lines
- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
         heatmapTexture:(id<MTLTexture>)heatmapTexture
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize
            showHeatmap:(BOOL)showHeatmap;

@end
