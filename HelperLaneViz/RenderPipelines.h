//
//  RenderPipelines.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import <MetalKit/MetalKit.h>

@interface RenderPipelines : NSObject
@property (nonatomic, readonly) id<MTLRenderPipelineState> mainPSO;
@property (nonatomic, readonly) id<MTLRenderPipelineState> overdrawPSO;
@property (nonatomic, readonly) id<MTLRenderPipelineState> gridPSO;

@property (nonatomic, readonly) id<MTLDepthStencilState> depthState;
@property (nonatomic, readonly) id<MTLDepthStencilState> noDepthState;

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView*)view;
@end
