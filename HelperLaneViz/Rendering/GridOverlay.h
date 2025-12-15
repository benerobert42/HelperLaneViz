//
//  GridOverlay.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import <MetalKit/MetalKit.h>

@interface GridOverlay : NSObject

- (instancetype)initWithPipelineState:(id<MTLRenderPipelineState>)pipelineState
                      depthStencilState:(id<MTLDepthStencilState>)depthState;

- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize;

@end
