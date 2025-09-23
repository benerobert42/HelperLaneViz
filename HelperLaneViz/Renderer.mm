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
#import "GeometryFactory.h"

#import <iostream>

@implementation Renderer
{
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLRenderPipelineState> _pipelineState;
    id<MTLDepthStencilState> _depthState;

    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;

    vector_uint2 _viewportSize;
}

- (id<MTLRenderPipelineState>) createPSOWith:(nonnull MTKView*)mtkView {
    id<MTLLibrary> defaultLibrary = [_device newDefaultLibrary];
    id<MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
    id<MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"fragmentShader"];

    MTLRenderPipelineDescriptor *psDesc = [[MTLRenderPipelineDescriptor alloc] init];
    psDesc.vertexFunction = vertexFunction;
    psDesc.fragmentFunction = fragmentFunction;
    psDesc.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat;
    psDesc.depthAttachmentPixelFormat = mtkView.depthStencilPixelFormat;
    psDesc.vertexBuffers[VertexInputIndexVertices].mutability = MTLMutabilityImmutable;

    NSError *error;
    id<MTLRenderPipelineState> psObject = [_device newRenderPipelineStateWithDescriptor:psDesc error:&error];
    NSAssert(psObject, @"Failed to create pipeline state: %@", error);
    return psObject;
}

- (id<MTLDepthStencilState>) createDSS {
    MTLDepthStencilDescriptor *dsDesc = [MTLDepthStencilDescriptor new];
    dsDesc.depthCompareFunction = MTLCompareFunctionLessEqual;
    dsDesc.depthWriteEnabled = YES;

    id<MTLDepthStencilState> dsState = [_device newDepthStencilStateWithDescriptor:dsDesc];
    return dsState;
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

        _pipelineState = [self createPSOWith:mtkView];
        _depthState = [self createDSS];
        _commandQueue = [_device newCommandQueue];

        auto vertices = GeometryFactory::CreateVerticesForCircle(100, 1.0);
        std::vector<uint32_t> indicesMWT = TriangleFactory::CreateConvexMWT(vertices).indices;

        
        _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                    length:vertices.size() * sizeof(Vertex)
                                   options:MTLResourceStorageModeShared];

        _indexBuffer = [_device newBufferWithBytes:indicesMWT.data()
                                    length:indicesMWT.size() * 4
                                   options:MTLResourceStorageModeShared];
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

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if(renderPassDescriptor != nil)
    {
        id<MTLRenderCommandEncoder> encoder =
        [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

        [encoder setRenderPipelineState:_pipelineState];
        [encoder setDepthStencilState:_depthState];

        FrameConstants frameConstants{
            .viewProjectionMatrix = matrix_identity_float4x4,
            .viewPortSize = _viewportSize
        };
        [encoder setVertexBytes:&frameConstants
                         length:sizeof(frameConstants)
                        atIndex:VertexInputIndexFrameConstants];

        [encoder setVertexBuffer:_vertexBuffer offset:0 atIndex:0];

        [encoder drawIndexedPrimitives:MTLPrimitiveTypeLineStrip
                            indexCount:_indexBuffer.length / sizeof(uint32_t)
                             indexType:MTLIndexTypeUInt32
                           indexBuffer:_indexBuffer
                     indexBufferOffset:0];

        [encoder endEncoding];
        [commandBuffer presentDrawable:view.currentDrawable];
    }
    [commandBuffer commit];
}

@end

