//
//  RenderPipelines.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>

NS_ASSUME_NONNULL_BEGIN

id<MTLRenderPipelineState> MakeMainPipelineState(id<MTLDevice> device,
                                                 MTKView *view,
                                                 id<MTLLibrary> library,
                                                 NSError **error);

id<MTLRenderPipelineState> MakeOverdrawPipelineState(id<MTLDevice> device,
                                                     MTKView *view,
                                                     id<MTLLibrary> library,
                                                     NSError **error);

// Wireframe pipeline - renders with fill mode lines
id<MTLRenderPipelineState> MakeWireframePipelineState(id<MTLDevice> device,
                                                     MTKView *view,
                                                     id<MTLLibrary> library,
                                                     NSError **error);

// Overdraw counting pipeline - renders to R32Float texture for accurate pixel counting
id<MTLRenderPipelineState> MakeOverdrawCountPipelineState(id<MTLDevice> device,
                                                          id<MTLLibrary> library,
                                                          NSError **error);

// Helper invocation counting pipeline - renders to a dummy target and writes per-pixel helper counts into a buffer
id<MTLRenderPipelineState> MakeHelperInvocationCountPipelineState(id<MTLDevice> device,
                                                                 id<MTLLibrary> library,
                                                                 NSError **error);

id<MTLRenderPipelineState> MakeGridOverlayPipelineState(id<MTLDevice> device,
                                                        MTKView *view,
                                                        id<MTLLibrary> library,
                                                        NSError **error);

id<MTLDepthStencilState> MakeDepthState(id<MTLDevice> device);
id<MTLDepthStencilState> MakeNoDepthState(id<MTLDevice> device);

NS_ASSUME_NONNULL_END
