//
//  MetricsComputer.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import "MetricsComputer.h"
#import "GeometryManager.h"
#import "PipelineFactory.h"
#import "ShaderTypes.h"

#import <Metal/Metal.h>

@implementation MetricsComputer {
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    
    // Overdraw measurement (GPU-based, texture approach)
    id<MTLRenderPipelineState> _overdrawCountPipeline;
    id<MTLComputePipelineState> _sumOverdrawPipeline;
    id<MTLTexture> _overdrawCountTexture;
    id<MTLBuffer> _overdrawResultsBuffer;
    
    // Helper lane measurement (GPU-based, texture approach - same as overdraw)
    id<MTLRenderPipelineState> _helperLaneCountPipeline;
    id<MTLComputePipelineState> _sumHelperLanePipeline;
    id<MTLTexture> _helperLaneCountTexture;
    id<MTLBuffer> _helperLaneResultsBuffer;
    
    // Shared resources for helper lane engagement
    id<MTLTexture> _helperLaneTexture;  // 1x1 dummy texture for forcing derivatives
    id<MTLSamplerState> _pointSampler;
}

- (instancetype)initWithDevice:(id<MTLDevice>)device commandQueue:(id<MTLCommandQueue>)commandQueue {
    if (!(self = [super init])) return nil;
    
    _device = device;
    _commandQueue = commandQueue;
    
    [self setupPipelines];
    [self setupResources];
    
    return self;
}

- (void)setupPipelines {
    NSError *error = nil;
    id<MTLLibrary> library = [_device newDefaultLibrary];
    
    // Overdraw pipelines
    _overdrawCountPipeline = MakeOverdrawCountPipelineState(_device, library, &error);
    NSAssert(_overdrawCountPipeline, @"Failed to create overdraw count pipeline: %@", error);
    
    id<MTLFunction> sumOverdrawFunc = [library newFunctionWithName:@"sumOverdrawTexture"];
    _sumOverdrawPipeline = [_device newComputePipelineStateWithFunction:sumOverdrawFunc error:&error];
    NSAssert(_sumOverdrawPipeline, @"Failed to create sumOverdraw pipeline: %@", error);
    
    // Helper lane pipelines (texture-based, like overdraw)
    _helperLaneCountPipeline = MakeHelperLaneCountPipelineState(_device, library, &error);
    NSAssert(_helperLaneCountPipeline, @"Failed to create helper lane count pipeline: %@", error);
    
    id<MTLFunction> sumHelperLaneFunc = [library newFunctionWithName:@"sumHelperLaneTexture"];
    _sumHelperLanePipeline = [_device newComputePipelineStateWithFunction:sumHelperLaneFunc error:&error];
    NSAssert(_sumHelperLanePipeline, @"Failed to create sumHelperLane pipeline: %@", error);
    
    // Results buffers (2 uints each: total count, unique pixels)
    _overdrawResultsBuffer = [_device newBufferWithLength:sizeof(uint32_t) * 2
                                                  options:MTLResourceStorageModeShared];
    _helperLaneResultsBuffer = [_device newBufferWithLength:sizeof(uint32_t) * 2
                                                    options:MTLResourceStorageModeShared];
}

- (void)setupResources {
    // Create 1x1 dummy texture for helper lane engagement (used by helperLaneCountFS)
    MTLTextureDescriptor *helperTexDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:1 height:1 mipmapped:NO];
    _helperLaneTexture = [_device newTextureWithDescriptor:helperTexDesc];
    uint8_t white = 255;
    [_helperLaneTexture replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&white bytesPerRow:1];
    
    MTLSamplerDescriptor *samplerDesc = [MTLSamplerDescriptor new];
    samplerDesc.minFilter = MTLSamplerMinMagFilterNearest;
    samplerDesc.magFilter = MTLSamplerMinMagFilterNearest;
    _pointSampler = [_device newSamplerStateWithDescriptor:samplerDesc];
}

- (id<MTLTexture>)reallocateTextureForSize:(vector_uint2)viewportSize {
    const uint32_t width = viewportSize.x;
    const uint32_t height = viewportSize.y;
    if (width == 0 || height == 0) {
        assert("Invalid viewport size");
        return nullptr;
    }
    
    MTLTextureDescriptor *desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    desc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModePrivate;
    return [_device newTextureWithDescriptor:desc];
}

- (void)computeOverdrawMetricsWithGeometry:(GeometryManager *)geometry
                                overdrawSum:(uint64_t*)outSum
                               overdrawRatio:(double*)outRatio {
    if (!geometry.vertexBuffer || !geometry.indexBuffer) {
        if (outSum) *outSum = 0;
        if (outRatio) *outRatio = 0.0;
        return;
    }
    
    const vector_uint2 viewportSize = geometry.viewportSize;
    if (!_overdrawCountTexture ||
        viewportSize.x != _overdrawCountTexture.width ||
        viewportSize.y != _overdrawCountTexture.height) {
        _overdrawCountTexture = [self reallocateTextureForSize:viewportSize];
    }

    if (!_overdrawCountTexture) {
        if (outSum) {
            *outSum = 0;
        }
        if (outRatio) {
            *outRatio = 0.0;
        }
        return;
    }
    
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    MTLRenderPassDescriptor *renderPass = [MTLRenderPassDescriptor new];
    renderPass.colorAttachments[0].texture = _overdrawCountTexture;
    renderPass.colorAttachments[0].loadAction = MTLLoadActionClear;
    renderPass.colorAttachments[0].storeAction = MTLStoreActionStore;
    renderPass.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 0);
    renderPass.depthAttachment.texture = nil;
    renderPass.depthAttachment.loadAction = MTLLoadActionDontCare;
    
    id<MTLRenderCommandEncoder> renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPass];
    [renderEncoder setRenderPipelineState:_overdrawCountPipeline];
    [renderEncoder setCullMode:MTLCullModeNone];
    [renderEncoder setTriangleFillMode:MTLTriangleFillModeFill];
    
    FrameConstants frameConstants = {
        .viewProjectionMatrix = geometry.viewProjectionMatrix,
        .viewPortSize = geometry.viewportSize
    };
    GridParams gridParams = geometry.gridParams;
    
    [renderEncoder setVertexBuffer:geometry.vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [renderEncoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [renderEncoder setVertexBytes:&gridParams length:sizeof(gridParams) atIndex:VertexInputGridParams];
    
    [renderEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                              indexCount:geometry.indexCount
                               indexType:MTLIndexTypeUInt32
                             indexBuffer:geometry.indexBuffer
                       indexBufferOffset:0
                           instanceCount:geometry.instanceCount];
    [renderEncoder endEncoding];
    
    // Clear results buffer
    uint32_t* resultsPtr = (uint32_t*)_overdrawResultsBuffer.contents;
    resultsPtr[0] = 0;
    resultsPtr[1] = 0;
    
    // Sum texture with compute shader
    id<MTLComputeCommandEncoder> computeEncoder = [commandBuffer computeCommandEncoder];
    [computeEncoder setComputePipelineState:_sumOverdrawPipeline];
    [computeEncoder setTexture:_overdrawCountTexture atIndex:0];
    [computeEncoder setBuffer:_overdrawResultsBuffer offset:0 atIndex:0];
    
    const uint32_t width = (uint32_t)_overdrawCountTexture.width;
    const uint32_t height = (uint32_t)_overdrawCountTexture.height;
    [computeEncoder dispatchThreads:MTLSizeMake(width, height, 1)
              threadsPerThreadgroup:MTLSizeMake(16, 16, 1)];
    [computeEncoder endEncoding];
    
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    
    uint64_t totalDraws = resultsPtr[0];
    uint64_t uniquePixels = resultsPtr[1];
    
    if (outSum) {
        *outSum = totalDraws;
    }
    if (outRatio) {
        *outRatio = (uniquePixels > 0) ? ((double)totalDraws / uniquePixels) : 0.0;
    }
}

- (void)computeHelperInvocationMetricsWithGeometry:(GeometryManager *)geometry
                                         helperSum:(uint64_t*)outSum
                                        helperRatio:(double*)outRatio {
    if (!geometry.vertexBuffer || !geometry.indexBuffer) {
        if (outSum) *outSum = 0;
        if (outRatio) *outRatio = 0.0;
        return;
    }
    
    const vector_uint2 viewportSize = geometry.viewportSize;
    if (!_helperLaneCountTexture ||
        viewportSize.x != _helperLaneCountTexture.width ||
        viewportSize.y != _helperLaneCountTexture.height) {
        _helperLaneCountTexture = [self reallocateTextureForSize:viewportSize];
    }

    if (!_helperLaneCountTexture) {
        if (outSum) *outSum = 0;
        if (outRatio) *outRatio = 0.0;
        return;
    }
    
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Render triangles to helper lane count texture
    // Each pixel gets the helper count (0-3) for its 2x2 quad
    MTLRenderPassDescriptor *renderPass = [MTLRenderPassDescriptor new];
    renderPass.colorAttachments[0].texture = _helperLaneCountTexture;
    renderPass.colorAttachments[0].loadAction = MTLLoadActionClear;
    renderPass.colorAttachments[0].storeAction = MTLStoreActionStore;
    renderPass.colorAttachments[0].clearColor = MTLClearColorMake(-1, 0, 0, 0); // -1 means "not rendered"
    renderPass.depthAttachment.texture = nil;
    renderPass.depthAttachment.loadAction = MTLLoadActionDontCare;
    
    id<MTLRenderCommandEncoder> renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPass];
    [renderEncoder setRenderPipelineState:_helperLaneCountPipeline];
    [renderEncoder setCullMode:MTLCullModeNone];
    [renderEncoder setTriangleFillMode:MTLTriangleFillModeFill];
    
    FrameConstants frameConstants = {
        .viewProjectionMatrix = geometry.viewProjectionMatrix,
        .viewPortSize = geometry.viewportSize
    };
    GridParams gridParams = geometry.gridParams;
    
    [renderEncoder setVertexBuffer:geometry.vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [renderEncoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [renderEncoder setVertexBytes:&gridParams length:sizeof(gridParams) atIndex:VertexInputGridParams];
    [renderEncoder setFragmentTexture:_helperLaneTexture atIndex:0];
    [renderEncoder setFragmentSamplerState:_pointSampler atIndex:0];
    
    [renderEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                              indexCount:geometry.indexCount
                               indexType:MTLIndexTypeUInt32
                             indexBuffer:geometry.indexBuffer
                       indexBufferOffset:0
                           instanceCount:geometry.instanceCount];
    [renderEncoder endEncoding];
    
    // Clear results buffer
    uint32_t* resultsPtr = (uint32_t*)_helperLaneResultsBuffer.contents;
    resultsPtr[0] = 0;
    resultsPtr[1] = 0;
    
    // Sum texture with compute shader
    id<MTLComputeCommandEncoder> computeEncoder = [commandBuffer computeCommandEncoder];
    [computeEncoder setComputePipelineState:_sumHelperLanePipeline];
    [computeEncoder setTexture:_helperLaneCountTexture atIndex:0];
    [computeEncoder setBuffer:_helperLaneResultsBuffer offset:0 atIndex:0];
    
    const uint32_t width = (uint32_t)_helperLaneCountTexture.width;
    const uint32_t height = (uint32_t)_helperLaneCountTexture.height;
    [computeEncoder dispatchThreads:MTLSizeMake(width, height, 1)
              threadsPerThreadgroup:MTLSizeMake(16, 16, 1)];
    [computeEncoder endEncoding];
    
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    
    uint64_t totalHelpers = resultsPtr[0];
    uint64_t renderedPixels = resultsPtr[1];
    
    if (outSum) *outSum = totalHelpers;
    // Ratio = average helper count per rendered pixel
    if (outRatio) *outRatio = (renderedPixels > 0) ? ((double)totalHelpers / renderedPixels) : 0.0;
}

@end
