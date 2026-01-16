#import <MetalKit/MetalKit.h>

@interface GridOverlay : NSObject

- (instancetype)initWithPipelineState:(id<MTLRenderPipelineState>)pipelineState;

- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize;

@end
