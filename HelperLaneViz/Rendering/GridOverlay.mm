//
//  GridOverlay.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import "GridOverlay.h"

#import "ShaderTypes.h"

@implementation GridOverlay {
    id<MTLRenderPipelineState> _pipelineState;
}

- (instancetype)initWithPipelineState:(id<MTLRenderPipelineState>)pipelineState {
    if ((self = [super init])) {
        _pipelineState = pipelineState;
    }
    return self;
}

- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize {
    GridUniforms uniforms = {
        .tileSize = {tileSize, tileSize},
        .framebuffer = {(uint32_t)drawableSize.width, (uint32_t)drawableSize.height},
        .lineWidth = 1.0f,
        .lineColor = {1.0f, 1.0f, 1.0f, 0.75f}
    };
    
    [encoder pushDebugGroup:@"GridOverlay"];
    [encoder setRenderPipelineState:_pipelineState];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill];
    [encoder setCullMode:MTLCullModeNone];
    [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:0];
    [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [encoder popDebugGroup];
}

@end
