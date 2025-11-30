//
//  GridOverlay.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import "GridOverlay.h"

@implementation GridOverlay {
    id<MTLRenderPipelineState> _pipelineState;
    id<MTLDepthStencilState> _depthStencilState;
}

- (instancetype)initWithPipelineState:(id<MTLRenderPipelineState>)pipelineState
                    depthStencilState:(id<MTLDepthStencilState>)depthState {
    if ((self = [super init])) {
        _pipelineState = pipelineState;
        _depthStencilState = depthState;
    }
    return self;
}

- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)encoder
         heatmapTexture:(id<MTLTexture>)heatmapTexture
               tileSize:(uint32_t)tileSize
           drawableSize:(CGSize)drawableSize
            showHeatmap:(BOOL)showHeatmap {
    
    // Heatmap texture is required only when showing heatmap
    if (showHeatmap && !heatmapTexture) return;
    
    GridUniforms uniforms = {
        .tileSize = {tileSize, tileSize},
        .framebuffer = {(uint32_t)drawableSize.width, (uint32_t)drawableSize.height},
        .lineWidth = 1.0f,
        .lineColor = {1.0f, 1.0f, 1.0f, 0.75f},
        .fillAlpha = 0.85f,
        .showHeatmap = showHeatmap ? 1u : 0u
    };
    
    [encoder pushDebugGroup:@"GridOverlay"];
    [encoder setRenderPipelineState:_pipelineState];
    [encoder setDepthStencilState:_depthStencilState];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill];
    [encoder setCullMode:MTLCullModeNone];
    if (heatmapTexture) {
    [encoder setFragmentTexture:heatmapTexture atIndex:0];
    }
    [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:0];
    [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [encoder popDebugGroup];
}

@end
