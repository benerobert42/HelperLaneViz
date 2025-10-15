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
#import "SVGLoader.h"

#import <iostream>

@implementation Renderer
{
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLRenderPipelineState> _pipelineState;
    id<MTLDepthStencilState> _depthState;

    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;

    id<MTLBuffer> _counterBuffer;

    vector_uint2 _viewportSize;

    GridParams _gridParams;
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

        std::vector<Vertex> vertices;
        std::vector<uint32_t> indicesUnused;
        SVGLoader::TessellateSvgToMesh("/Users/robi/Downloads/drip-coffee-maker.svg", vertices, indicesUnused);

//        auto vertices = GeometryFactory::CreateVerticesForCircle(3000, 0.015);
        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMWT(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateCentralTriangulation(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateDelauneyTriangulation(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateMaxAreaTriangulation(vertices);

        _gridParams = {
            .cols = 1,
            .rows = 1,
            .cellSize = {2.0 / 1, 2.0 / 1},
            .origin = {-0.99, -0.99},
            .scale = 1.0
        };

        _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                    length:vertices.size() * sizeof(Vertex)
                                   options:MTLResourceStorageModeShared];

        _indexBuffer = [_device newBufferWithBytes:indices.data()
                                    length:indices.size() * 4
                                   options:MTLResourceStorageModeShared];

        MTLResourceOptions opts = MTLResourceStorageModeManaged;
        _counterBuffer = [_device newBufferWithLength:sizeof(uint32_t) options:opts];
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

    *reinterpret_cast<uint32_t *>(_counterBuffer.contents) = 0;
    if (_counterBuffer.storageMode == MTLStorageModeManaged) {
        id<MTLBlitCommandEncoder> blit = [commandBuffer blitCommandEncoder];
        [blit synchronizeResource:_counterBuffer]; // make CPU write visible to GPU
        [blit endEncoding];
    }

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if(renderPassDescriptor != nil) {
        id<MTLRenderCommandEncoder> encoder =
        [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

        [encoder setRenderPipelineState:_pipelineState];
        [encoder setDepthStencilState:_depthState];

        FrameConstants frameConstants{
            .viewProjectionMatrix = matrix_identity_float4x4,
            .viewPortSize = _viewportSize
        };

        [encoder setFragmentBuffer:_counterBuffer
                           offset:0
                          atIndex:0];

        [encoder setVertexBytes:&frameConstants
                         length:sizeof(frameConstants)
                        atIndex:VertexInputIndexFrameConstants];

        [encoder setVertexBuffer:_vertexBuffer offset:0 atIndex:0];

        [encoder setVertexBytes:&_gridParams
                         length:sizeof(_gridParams)
                        atIndex:2];

        [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                            indexCount:_indexBuffer.length / sizeof(uint32_t)
                             indexType:MTLIndexTypeUInt32
                           indexBuffer:_indexBuffer
                     indexBufferOffset:0
                         instanceCount:1];

        [encoder endEncoding];

        id<MTLBlitCommandEncoder> blit = [commandBuffer blitCommandEncoder];
        [blit synchronizeResource:_counterBuffer];
        [blit endEncoding];

        [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> _Nonnull) {
            const uint32_t total = *reinterpret_cast<const uint32_t *>(self->_counterBuffer.contents);
            printf("Total helper fragments this frame: %u\n", total);
        }];

        [commandBuffer presentDrawable:view.currentDrawable];
    }
    [commandBuffer commit];
}

@end

