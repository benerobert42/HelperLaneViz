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

    id<MTLRenderPipelineState> _overdrawPipelineState;
    id<MTLDepthStencilState> _overdrawDepthState;

    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;

    id<MTLBuffer> _counterBuffer;

    vector_uint2 _viewportSize;

    GridParams _gridParams;
    bool _useOverDrawPass;
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

- (id<MTLRenderPipelineState>) createOverdrawPSOWith:(nonnull MTKView*)mtkView {
    id<MTLLibrary> defaultLibrary = [_device newDefaultLibrary];
    id<MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
    id<MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"overdrawFragment"];

    MTLRenderPipelineDescriptor *psDesc = [MTLRenderPipelineDescriptor new];
    psDesc.vertexFunction = vertexFunction;
    psDesc.fragmentFunction = fragmentFunction;
    psDesc.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat;

    psDesc.depthAttachmentPixelFormat   = mtkView.depthStencilPixelFormat;
    psDesc.stencilAttachmentPixelFormat = MTLPixelFormatInvalid;

    // Enable additive blending: dst = src + dst
    auto *ca0 = psDesc.colorAttachments[0];
    ca0.blendingEnabled = YES;
    ca0.rgbBlendOperation = MTLBlendOperationAdd;
    ca0.alphaBlendOperation = MTLBlendOperationAdd;
    ca0.sourceRGBBlendFactor = MTLBlendFactorOne;
    ca0.destinationRGBBlendFactor = MTLBlendFactorOne;
    ca0.sourceAlphaBlendFactor = MTLBlendFactorOne;
    ca0.destinationAlphaBlendFactor = MTLBlendFactorOne;

    // (Depth format is irrelevant; we’ll disable depth test for this pass.)
    NSError *err = nil;
    id<MTLRenderPipelineState> pso = [_device newRenderPipelineStateWithDescriptor:psDesc error:&err];
    NSAssert(pso, @"Overdraw PSO failed: %@", err);
    return pso;
}

- (id<MTLDepthStencilState>) createNoDepthDSS {
    MTLDepthStencilDescriptor *ds = [MTLDepthStencilDescriptor new];
    ds.depthWriteEnabled   = NO;
    ds.depthCompareFunction = MTLCompareFunctionAlways; // accept every fragment
    return [_device newDepthStencilStateWithDescriptor:ds];
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

        _overdrawPipelineState = [self createOverdrawPSOWith:mtkView];
        _overdrawDepthState    = [self createNoDepthDSS];

        _commandQueue = [_device newCommandQueue];

//        std::vector<Vertex> vertices;
//        std::vector<uint32_t> indicesUnused;
//        SVGLoader::TessellateSvgToMesh("/Users/robi/Downloads/Tractor2.svg", vertices, indicesUnused);

        double cumulatedEdgeLength;
        auto vertices = GeometryFactory::CreateVerticesForEllipse(500, 1, 1);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMWT(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateCentralTriangulation(vertices);
        std::vector<uint32_t> indices = TriangleFactory::TriangulatePolygon_CDT(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateStripTriangulation(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMinMaxAreaTriangulation(vertices, cumulatedEdgeLength);

        std::cout << "Cumulated edge length: " << cumulatedEdgeLength << std::endl;

        uint32_t gridSegmentCount = 2;
        _gridParams = {
            .cols = gridSegmentCount,
            .rows = gridSegmentCount,
            .cellSize = {static_cast<float>(2.0 / gridSegmentCount), static_cast<float>(2.0 / gridSegmentCount)},
            .origin = {-0.99, -0.99},
            .scale = 0.5
        };

        _useOverDrawPass = false;

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

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if(renderPassDescriptor != nil) {
        if (_useOverDrawPass) {
            id<MTLRenderCommandEncoder> encoder =
            [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

            [encoder setRenderPipelineState:_overdrawPipelineState];
            [encoder setDepthStencilState:_overdrawDepthState];

            FrameConstants frameConstants{
                .viewProjectionMatrix = matrix_identity_float4x4,
                .viewPortSize = _viewportSize
            };

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

            [commandBuffer presentDrawable:view.currentDrawable];

        } else {
            *reinterpret_cast<uint32_t *>(_counterBuffer.contents) = 0;
            if (_counterBuffer.storageMode == MTLStorageModeManaged) {
                id<MTLBlitCommandEncoder> blit = [commandBuffer blitCommandEncoder];
                [blit synchronizeResource:_counterBuffer]; // make CPU write visible to GPU
                [blit endEncoding];
            }

            id<MTLRenderCommandEncoder> encoder =
            [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

            [encoder setRenderPipelineState:_pipelineState];
            [encoder setDepthStencilState:_depthState];
            [encoder setTriangleFillMode:MTLTriangleFillModeLines];
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
                             instanceCount:4];

            [encoder endEncoding];

            id<MTLBlitCommandEncoder> blit = [commandBuffer blitCommandEncoder];
            [blit synchronizeResource:_counterBuffer];
            [blit endEncoding];

            [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> _Nonnull) {
                const uint32_t total = *reinterpret_cast<const uint32_t *>(self->_counterBuffer.contents);
            }];

            [commandBuffer presentDrawable:view.currentDrawable];
        }
    }
    [commandBuffer commit];
}

@end
