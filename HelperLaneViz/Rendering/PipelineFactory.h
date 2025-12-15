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

id<MTLRenderPipelineState> MakeWireframePipelineState(id<MTLDevice> device,
                                                     MTKView *view,
                                                     id<MTLLibrary> library,
                                                     NSError **error);

// Overdraw counting pipeline - renders to R32Float texture for accurate pixel counting
id<MTLRenderPipelineState> MakeOverdrawCountPipelineState(id<MTLDevice> device,
                                                          id<MTLLibrary> library,
                                                          NSError **error);

// Helper lane counting pipeline - renders helper count (0-3) per pixel to R32Float texture
id<MTLRenderPipelineState> MakeHelperLaneCountPipelineState(id<MTLDevice> device,
                                                           id<MTLLibrary> library,
                                                           NSError **error);

id<MTLRenderPipelineState> MakeGridOverlayPipelineState(id<MTLDevice> device,
                                                        MTKView *view,
                                                        id<MTLLibrary> library,
                                                        NSError **error);

NS_ASSUME_NONNULL_END
