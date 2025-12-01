//
//  RenderPipelines.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import "PipelineFactory.h"

static inline MTLRenderPipelineDescriptor *CommonPipelineDescriptor(MTKView *view) {
    MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
    d.colorAttachments[0].pixelFormat = view.colorPixelFormat;
    d.depthAttachmentPixelFormat = view.depthStencilPixelFormat;
    d.stencilAttachmentPixelFormat = MTLPixelFormatInvalid;
    return d;
}

id<MTLRenderPipelineState> MakeMainPipelineState(id<MTLDevice> device,
                                                 MTKView *view,
                                                 id<MTLLibrary> library,
                                                 NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"MainPipeline";
    d.vertexFunction   = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"mainFS"];
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeOverdrawPipelineState(id<MTLDevice> device,
                                                     MTKView *view,
                                                     id<MTLLibrary> library,
                                                     NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"OverdrawPipeline";
    d.vertexFunction   = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"overdrawFS"];
    MTLRenderPipelineColorAttachmentDescriptor *ca = d.colorAttachments[0];
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation   = MTLBlendOperationAdd;
    ca.alphaBlendOperation = MTLBlendOperationAdd;
    ca.sourceRGBBlendFactor   = MTLBlendFactorOne;
    ca.destinationRGBBlendFactor = MTLBlendFactorOne;
    ca.sourceAlphaBlendFactor   = MTLBlendFactorOne;
    ca.destinationAlphaBlendFactor = MTLBlendFactorOne;
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeGridOverlayPipelineState(id<MTLDevice> device,
                                                        MTKView *view,
                                                        id<MTLLibrary> library,
                                                        NSError **error) {
    MTLRenderPipelineDescriptor *d = CommonPipelineDescriptor(view);
    d.label = @"GridOverlayPipeline";
    d.vertexFunction   = [library newFunctionWithName:@"gridVS"];
    d.fragmentFunction = [library newFunctionWithName:@"gridFS"];
    MTLRenderPipelineColorAttachmentDescriptor *ca = d.colorAttachments[0];
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation   = MTLBlendOperationAdd;
    ca.alphaBlendOperation = MTLBlendOperationAdd;
    ca.sourceRGBBlendFactor   = MTLBlendFactorSourceAlpha;
    ca.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    ca.sourceAlphaBlendFactor   = MTLBlendFactorSourceAlpha;
    ca.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLRenderPipelineState> MakeOverdrawCountPipelineState(id<MTLDevice> device,
                                                          id<MTLLibrary> library,
                                                          NSError **error) {
    MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
    d.label = @"OverdrawCountPipeline";
    d.vertexFunction   = [library newFunctionWithName:@"mainVS"];
    d.fragmentFunction = [library newFunctionWithName:@"overdrawCountFS"];
    
    // R32Float color attachment for accurate counting
    d.colorAttachments[0].pixelFormat = MTLPixelFormatR32Float;
    
    // Additive blending: each fragment adds 1.0
    MTLRenderPipelineColorAttachmentDescriptor *ca = d.colorAttachments[0];
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation   = MTLBlendOperationAdd;
    ca.alphaBlendOperation = MTLBlendOperationAdd;
    ca.sourceRGBBlendFactor   = MTLBlendFactorOne;
    ca.destinationRGBBlendFactor = MTLBlendFactorOne;
    ca.sourceAlphaBlendFactor   = MTLBlendFactorOne;
    ca.destinationAlphaBlendFactor = MTLBlendFactorOne;
    
    // No depth attachment needed for overdraw counting
    d.depthAttachmentPixelFormat = MTLPixelFormatInvalid;
    
    return [device newRenderPipelineStateWithDescriptor:d error:error];
}

id<MTLDepthStencilState> MakeDepthState(id<MTLDevice> device) {
    MTLDepthStencilDescriptor *ds = [MTLDepthStencilDescriptor new];
    ds.depthWriteEnabled = YES;
    ds.depthCompareFunction = MTLCompareFunctionLessEqual;
    id<MTLDepthStencilState> state = [device newDepthStencilStateWithDescriptor:ds];
    return state;
}

id<MTLDepthStencilState> MakeNoDepthState(id<MTLDevice> device) {
    MTLDepthStencilDescriptor *ds = [MTLDepthStencilDescriptor new];
    ds.depthWriteEnabled = NO;
    ds.depthCompareFunction = MTLCompareFunctionAlways;
    id<MTLDepthStencilState> state = [device newDepthStencilStateWithDescriptor:ds];
    return state;
}
