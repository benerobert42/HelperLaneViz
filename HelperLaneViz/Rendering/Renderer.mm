//
//  Renderer.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//
#import "Renderer.h"
#import "ShaderTypes.h"
#import "AssetLoader.h"
#import "MathUtilities.h"
#import "GridOverlay.h"
#import "PipelineFactory.h"
#import "TriangulationMetrics.h"

#import "../Geometry/CGALTriangulator.h"

#import <MetalKit/MetalKit.h>

#import <iostream>
#import <filesystem>

@implementation Renderer {
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    MTKView *_view;

    id<MTLRenderPipelineState> _mainPipeline;
    id<MTLRenderPipelineState> _gridOverlayPipeline;

    id<MTLDepthStencilState> _depthState;
    id<MTLDepthStencilState> _noDepthState;

    GridOverlay *_gridOverlay;
    GridParams _gridParams;

    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;

    vector_uint2 _viewportSize;
    simd_float4x4 _viewProjection;
}

- (GridParams)createGridParamsForSegmentCount:(uint32_t)segmentCount
                                    andOrigin:(simd_float2)origin
                                     andScale:(float)scale {
    GridParams params{
        .cols = segmentCount,
        .rows = segmentCount,
        .cellSize = {2.f / segmentCount, 2.f / segmentCount},
        .origin = origin,
        .scale = scale
    };
    return params;
}

- (instancetype)initWithMetalKitView:(MTKView *)mtkView {
    if (!(self = [super init])) return nil;

    _device = mtkView.device;
    _view = mtkView;
    _view.clearColor = MTLClearColorMake(0, 0, 0, 1);
    _view.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
    _view.clearDepth = 1.0;

    NSError *error = nil;
    id<MTLLibrary> library = [_device newDefaultLibrary];

    _mainPipeline = MakeMainPipelineState(_device, _view, library, &error);
    NSAssert(_mainPipeline != nil, @"Failed to create MainPipeline: %@", error);

    _gridOverlayPipeline = MakeGridOverlayPipelineState(_device, _view, library, &error);
    NSAssert(_gridOverlayPipeline != nil, @"Failed to create GridOverlayPipeline: %@", error);

    _depthState = MakeDepthState(_device);
    _noDepthState = MakeNoDepthState(_device);

    _gridOverlay = [[GridOverlay alloc] initWithPSO:_gridOverlayPipeline noDepth:_noDepthState];
    _gridParams = [self createGridParamsForSegmentCount:3
                                              andOrigin:simd_make_float2(-0.7, -0.7)
                                               andScale:0.4f];

    _commandQueue = [_device newCommandQueue];

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    AssetLoader::LoadPositionsAndIndicesFromObj(_device,
                                                @"/Users/robi/Downloads/ellipsoid_oblate_capOutMWT.obj",
                                                vertices,
                                                indices);

    _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                         length:vertices.size() * sizeof(Vertex)
                                        options:MTLResourceStorageModeShared];

    _indexBuffer = [_device newBufferWithBytes:indices.data()
                                        length:indices.size() * sizeof(uint32_t)
                                       options:MTLResourceStorageModeShared];

    simd_float4x4 viewMatrix = createLookAtRhs((simd_float3){0, 0, 3},
                                               (simd_float3){0, 0, 0},
                                               (simd_float3){0, 1, 0});

    simd_float4x4 projMatrix = makePerspective(M_PI_4, 1, 0.1, 40);
    _viewProjection = simd_mul(projMatrix, viewMatrix);

    std::vector<Vertex> vertsMWT, vertsDel;
    std::vector<uint32_t> idxMWT, idxDel;

    const auto outPath = std::filesystem::temp_directory_path() / "ellipsoid_oblate_capOutMWTSS.obj";
    std::cerr << "[DBG] temp dir: " << std::filesystem::temp_directory_path() << "\n";

//    int stacks,
//    int slices,
//    bool wrapColumns,
//    bool applyScreenFlips,
//    simd_int2 frameBuffer,
//    simd_float4x4 viewProjMatrix

    CGALTriangulator::TriangulateVertexOnlyEllipsoidOBJ("/Users/robi/Downloads/ellipsoid_oblate_cap.obj",
                                                        outPath,
                                                        TriangulationMode::Delaunay,
                                                        161, 32, true, false, _viewportSize,
                                                        _viewProjection);

    AssetLoader::LoadPositionsAndIndicesFromObj(_device,
                                                @"/Users/robi/Downloads/ellipsoid_oblate_cap_lowOutMWT.obj",
                                                vertsMWT,
                                                idxMWT);

    AssetLoader::LoadPositionsAndIndicesFromObj(_device,
                                                @"/Users/robi/Downloads/ellipsoid_oblate_cap_lowOutDel.obj",
                                                vertsDel,
                                                idxDel);

    simd_int2 framebufferPx = { (int)_view.drawableSize.width, (int)_view.drawableSize.height };
    simd_int2 tileSizePx = { 32, 32 };

    TriMetrics::printEdgeAndTileMetrics(vertsMWT,
                                        idxMWT,
                                        vertsDel,
                                        idxDel,
                                        _viewProjection,
                                        framebufferPx,
                                        tileSizePx);

    return self;
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    _viewportSize = (vector_uint2){ (uint32_t)size.width, (uint32_t)size.height };
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    MTLRenderPassDescriptor *renderPassDesc = view.currentRenderPassDescriptor;
    if (!renderPassDesc) {
        [commandBuffer commit];
        return;
    }

    renderPassDesc.tileWidth = 32;
    renderPassDesc.tileHeight = 32;

    FrameConstants frameConstants = { .viewProjectionMatrix = _viewProjection, .viewPortSize = _viewportSize };

    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDesc];

    [encoder setRenderPipelineState:_mainPipeline];
    [encoder setDepthStencilState:_depthState];
    [encoder setCullMode:MTLCullModeBack];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill];

    [encoder setVertexBuffer:_vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [encoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [encoder setVertexBytes:&_gridParams length:sizeof(_gridParams) atIndex:VertexInputGridParams];

    [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                    indexCount:_indexBuffer.length / sizeof(uint32_t)
                     indexType:MTLIndexTypeUInt32
                   indexBuffer:_indexBuffer
             indexBufferOffset:0
                 instanceCount:_gridParams.cols * _gridParams.rows];

    MTLViewport viewPort = {.originX = 0, .originY = 0,
                            .width = view.drawableSize.width, .height = view.drawableSize.height,
                            .znear = 0, .zfar = 1};
    [encoder setViewport:viewPort];
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill];
    [encoder setDepthStencilState:_noDepthState];

    [_gridOverlay drawWithEncoder:encoder drawableSize:view.drawableSize];

    [encoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
    [commandBuffer commit];
}

@end
