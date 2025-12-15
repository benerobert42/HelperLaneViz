//
//  PipelineFactory.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import "PipelineFactory.h"

// Common pipeline descriptor for view-based pipelines (no depth)
static inline MTLRenderPipelineDescriptor *CommonPipelineDescriptor(MTKView *view) {
    MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
    d.colorAttachments[0].pixelFormat = view.colorPixelFormat;
    d.depthAttachmentPixelFormat = MTLPixelFormatInvalid;
    d.stencilAttachmentPixelFormat = MTLPixelFormatInvalid;
    return d;
}

// Helper to configure additive blending
static inline void ConfigureAdditiveBlending(MTLRenderPipelineColorAttachmentDescriptor *ca) {
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation = MTLBlendOperationAdd;
    ca.alphaBlendOperation = MTLBlendOperationAdd;
    ca.sourceRGBBlendFactor = MTLBlendFactorOne;
    ca.destinationRGBBlendFactor = MTLBlendFactorOne;
    ca.sourceAlphaBlendFactor = MTLBlendFactorOne;
    ca.destinationAlphaBlendFactor = MTLBlendFactorOne;
}

// Helper to configure alpha blending
static inline void ConfigureAlphaBlending(MTLRenderPipelineColorAttachmentDescriptor *ca) {
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation = MTLBlendOperationAdd;
    ca.alphaBlendOperation = MTLBlendOperationAdd;
    ca.sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    ca.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    ca.sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
    ca.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
}

id<MTLRenderPipelineState> MakeMainPipelineState(id<MTLDevice> device,
                                                 MTKView *view,
                                                 id<MTLLibrary> library,
                                                 NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"MainPipeline";
    d.vertexFunction = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"mainFS"];
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeOverdrawPipelineState(id<MTLDevice> device,
                                                     MTKView *view,
                                                     id<MTLLibrary> library,
                                                     NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"OverdrawPipeline";
    d.vertexFunction = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"overdrawFS"];
    ConfigureAdditiveBlending(d.colorAttachments[0]);
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeWireframePipelineState(id<MTLDevice> device,
                                                     MTKView *view,
                                                     id<MTLLibrary> library,
                                                     NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"WireframePipeline";
    d.vertexFunction = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"wireframeFS"];
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeGridOverlayPipelineState(id<MTLDevice> device,
                                                        MTKView *view,
                                                        id<MTLLibrary> library,
                                                        NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"GridOverlayPipeline";
    d.vertexFunction = [library newFunctionWithName:@"gridVS"];
    d.fragmentFunction = [library newFunctionWithName:@"gridFS"];
    ConfigureAlphaBlending(d.colorAttachments[0]);
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeOverdrawCountPipelineState(id<MTLDevice> device,
                                                          id<MTLLibrary> library,
                                                          NSError **error) {
    MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
    d.label = @"OverdrawCountPipeline";
    d.vertexFunction = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"overdrawCountFS"];
    d.colorAttachments[0].pixelFormat = MTLPixelFormatR32Float;
    d.depthAttachmentPixelFormat = MTLPixelFormatInvalid;
    ConfigureAdditiveBlending(d.colorAttachments[0]);
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeHelperLaneCountPipelineState(id<MTLDevice> device,
                                                           id<MTLLibrary> library,
                                                           NSError **error) {
    MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
    d.label = @"HelperLaneCountPipeline";
    d.vertexFunction = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"helperLaneCountFS"];
    d.colorAttachments[0].pixelFormat = MTLPixelFormatR32Float;  // Same as overdraw
    d.depthAttachmentPixelFormat = MTLPixelFormatInvalid;
    // No blending - we want the last value written (helper count 0-3)
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}
