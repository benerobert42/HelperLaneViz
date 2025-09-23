//
//  Renderer.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#include <MetalKit/MetalKit.h>

#import "AssetLoader.h"
#import "MathUtilities.h"
#import "Renderer.h"
#import "ShaderTypes.h"
#import "Triangulation.h"

#import <iostream>

@implementation Renderer
{
    id<MTLDevice>              _device;
    id<MTLCommandQueue>        _commandQueue;
    id<MTLRenderPipelineState> _pipelineState;
    id<MTLRenderPipelineState> _accumulatePSO;
    id<MTLRenderPipelineState> _visualizePSO;

    id<MTLBuffer> vbuf;
    id<MTLBuffer> ibuf;

    id<MTLDepthStencilState> _depthState;
    vector_uint2             _viewportSize;

    uint32_t _ibuflen;
}

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView
{
    self = [super init];
    if(self)
    {
        _device = mtkView.device;
        mtkView.clearColor = MTLClearColorMake(0, 0, 0, 1);
        mtkView.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
        mtkView.clearDepth = 1.0;

        id<MTLLibrary> defaultLibrary = [_device newDefaultLibrary];
        id<MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
        id<MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"fragmentShader"];

        MTLRenderPipelineDescriptor *pipelineStateDescriptor = [[MTLRenderPipelineDescriptor alloc] init];
        pipelineStateDescriptor.vertexFunction = vertexFunction;
        pipelineStateDescriptor.fragmentFunction = fragmentFunction;
        pipelineStateDescriptor.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat;
        pipelineStateDescriptor.depthAttachmentPixelFormat = mtkView.depthStencilPixelFormat;
        pipelineStateDescriptor.vertexBuffers[VertexInputIndexVertices].mutability = MTLMutabilityImmutable;

        NSError *error;
        _pipelineState = [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor error:&error];

        id<MTLFunction> accumulateFragmentFunction = [defaultLibrary newFunctionWithName:@"accumulateFrag"];
//        pipelineStateDescriptor.colorAttachments[0].writeMask = 0;
        pipelineStateDescriptor.fragmentFunction = accumulateFragmentFunction;
        _accumulatePSO = [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor error:&error];

        id<MTLFunction> visualizeFragmentFunction = [defaultLibrary newFunctionWithName:@"visualizeFrag"];
        id<MTLFunction> visualizeVertexFunction = [defaultLibrary newFunctionWithName:@"visualizeVertex"];
        pipelineStateDescriptor.vertexFunction = visualizeVertexFunction;
        pipelineStateDescriptor.fragmentFunction = visualizeFragmentFunction;
        _visualizePSO = [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor error:&error];

        NSAssert(_pipelineState, @"Failed to create pipeline state: %@", error);
        NSAssert(_accumulatePSO, @"Failed to create acc. pipeline state: %@", error);
        NSAssert(_visualizePSO, @"Failed to create vis. pipeline state: %@", error);

        MTLDepthStencilDescriptor *depthDescriptor = [MTLDepthStencilDescriptor new];
        depthDescriptor.depthCompareFunction = MTLCompareFunctionLessEqual;
        depthDescriptor.depthWriteEnabled = YES;
        _depthState = [_device newDepthStencilStateWithDescriptor:depthDescriptor];

        _commandQueue = [_device newCommandQueue];

        // ----- circle (decagon) geometry -----
        constexpr uint32_t kSides = 10;
        const float zCoord = 1.0;

        const simd_float2 center = {0, 0};
        const float  radius = 1.0;

        Vertex circleVertices[kSides];
        for (uint32_t i = 0; i < kSides; ++i) {
            const float t = (2.0f * M_PI * i) / kSides;
            const float cx = center.x + radius * cosf(t);
            const float cy = center.y + radius * sinf(t);

            circleVertices[i] = {{cx, cy, zCoord}, {0, 0, 1}};
        }

        // ----- index buffer (triangle fan pivoted at vertex 0) -----
//        constexpr uint32_t kTriCount = kSides - 2;         // 8
//        uint16_t circleIndices[kTriCount * 3];             // 24 indices
//
//        for (uint32_t i = 0; i < kTriCount; ++i) {
//            circleIndices[i*3 + 0] = 0;     // pivot
//            circleIndices[i*3 + 1] = i + 1; // next
//            circleIndices[i*3 + 2] = i + 2; // next-next
//        }

        std::vector<simd_float3> positions;
        for (auto& vertex: circleVertices) {
            positions.push_back(vertex.position);
        }
        std::vector<uint32_t> circleIndicesMWT = TriangleFactory::minimumWeightTriangulationConvex(positions).indices;


        vbuf = [_device newBufferWithBytes:circleVertices
                                    length:sizeof(circleVertices)
                                   options:MTLResourceStorageModeShared];

        std::cout << circleIndicesMWT.size() << std::endl;
        ibuf = [_device newBufferWithBytes:circleIndicesMWT.data()
                                    length:circleIndicesMWT.size() * 4/*sizeof(circleIndicesMWT) * sizeof(uint32_t)*/
                                   options:MTLResourceStorageModeShared];
        _ibuflen = (uint32_t)circleIndicesMWT.size();
    }
    return self;
}

#pragma mark - MTKView Delegate Methods

- (void)mtkView:(nonnull MTKView *)view drawableSizeWillChange:(CGSize)size
{
    _viewportSize.x = size.width;
    _viewportSize.y = size.height;
}

- (void)drawInMTKView:(nonnull MTKView *)view
{
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    commandBuffer.label = @"Command Buffer";

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if(renderPassDescriptor != nil)
    {
        id<MTLRenderCommandEncoder> renderEncoder =
        [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
        renderEncoder.label = @"Render Encoder";

        [renderEncoder setRenderPipelineState:_pipelineState];
        [renderEncoder setDepthStencilState:_depthState];

//        MTKMesh* mesh = AssetLoader::LoadModelFromObj(_device, @"Challenger");

//        for (size_t index = 0; index < mesh.vertexBuffers.count; ++index) {
//            [renderEncoder setVertexBuffer:mesh.vertexBuffers[index].buffer offset:0 atIndex:index];
//        }

//        simd_float4x4 projectionMatrix = makePerspective(M_PI_4, 1, 0.1, 30);
//        simd_float4x4 viewMatrix = createLookAtRhs(simd_float3{8, 8, 15},
//                                             simd_float3{-8, -8, -15},
//                                             simd_float3{0, 1, 0});
//        simd_float4x4 viewProjectionMatrix = simd_mul(projectionMatrix, viewMatrix);
        FrameConstants frameConstants{.viewProjectionMatrix = matrix_identity_float4x4, .viewPortSize = _viewportSize};
        [renderEncoder setVertexBytes:&frameConstants
                               length:sizeof(frameConstants)
                              atIndex:VertexInputIndexFrameConstants];

        [renderEncoder setVertexBuffer:vbuf offset:0 atIndex:0];

        [renderEncoder setRenderPipelineState:_pipelineState];

        [renderEncoder drawIndexedPrimitives:MTLPrimitiveTypeLineStrip
                                  indexCount:_ibuflen
                               indexType:MTLIndexTypeUInt32
                             indexBuffer:ibuf
                       indexBufferOffset:0];

        [renderEncoder endEncoding];
        [commandBuffer presentDrawable:view.currentDrawable];
    }
    [commandBuffer commit];
}

@end

