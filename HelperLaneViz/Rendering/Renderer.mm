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
#import "../Geometry/GeometryFactory.h"
#import "GridOverlay.h"
#import "PipelineFactory.h"
#import "TriangulationMetrics.h"
#import "../Geometry/Triangulation.h"

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

    id<MTLComputePipelineState> _binTrianglesPSO;
    id<MTLComputePipelineState> _countsToTexturePSO;

    id<MTLBuffer>  _tileCounts;      // uint per tile
    id<MTLTexture> _countsTex;       // one pixel per tile (RGBA8 for simplicity)

    uint32_t _tilesX;
    uint32_t _tilesY;
    uint32_t _tileW;
    uint32_t _tileH;

    id<MTLBuffer> _maxCountBuf;

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

- (void)meshTriangulatorHelper {
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

    CGALTriangulator::TriangulateVertexOnlyEllipsoidOBJ("/Users/robi/Downloads/ellipsoid_oblate_cap.obj",
                                                        outPath,
                                                        TriangulationMode::MWT,
                                                        -1, -1, true);

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
}

- (void)polygonTriangulatorHelper {
    simd_float4x4 viewMatrix = createLookAtRhs((simd_float3){0, 0, -3},
                                               (simd_float3){0, 0, 0},
                                               (simd_float3){0, 1, 0});
    simd_float4x4 projMatrix = makeOrthoRhs(/*left*/-1, /*right*/1,
                                         /*bottom*/-1, /*top*/1,
                                         /*znear*/0.0f, /*zfar*/10.0f);
    // Place geometry at z in [0,1] (your ellipse z is 0, OK with near=0,f ar=10)
    _viewProjection = simd_mul(projMatrix, viewMatrix);

    auto vertices = GeometryFactory::CreateVerticesForEllipse(300, 1.0, 0.5);
    double edgeLength = 0;
    auto indices = TriangleFactory::CreateConvexMWT(vertices, edgeLength);

    _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                         length:vertices.size() * sizeof(Vertex)
                                        options:MTLResourceStorageModeShared];

    _indexBuffer = [_device newBufferWithBytes:indices.data()
                                        length:indices.size() * sizeof(uint32_t)
                                       options:MTLResourceStorageModeShared];

    simd_int2 framebufferPx = { (int)_view.drawableSize.width, (int)_view.drawableSize.height };
    TriMetrics::Print2DMeshMetrics(vertices, indices, framebufferPx, simd_int2{32, 32});
}

- (void)prepareTileHeatmapWithCommandBuffer:(id<MTLCommandBuffer>)cb
{
    // 0) Guard: need geometry
    if (!_vertexBuffer || !_indexBuffer) return;

    // 1) Derive tiling from current drawable size
    const uint32_t fbW = (uint32_t)_view.drawableSize.width;
    const uint32_t fbH = (uint32_t)_view.drawableSize.height;
    const uint32_t tilesX = (fbW + _tileW - 1) / _tileW;
    const uint32_t tilesY = (fbH + _tileH - 1) / _tileH;
    const size_t   tileCount = (size_t)tilesX * tilesY;

    // 2) (Re)allocate resources if needed (counts buffer, heat texture, max buffer)
    const bool needRealloc = (!_tileCounts || tilesX != _tilesX || tilesY != _tilesY);
    if (needRealloc) {
        _tilesX = tilesX;
        _tilesY = tilesY;

        _tileCounts = [_device newBufferWithLength:tileCount * sizeof(uint32_t)
                                           options:MTLResourceStorageModeShared];

        MTLTextureDescriptor *td =
            [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                               width:tilesX
                                                              height:tilesY
                                                           mipmapped:NO];
        td.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
        _countsTex = [_device newTextureWithDescriptor:td];

        if (!_maxCountBuf) {
            _maxCountBuf = [_device newBufferWithLength:sizeof(uint32_t)
                                                options:MTLResourceStorageModeShared];
        }
    }

    // 3) Clear counts + max (host-visible, tiny)
    memset(_tileCounts.contents, 0, tileCount * sizeof(uint32_t));
    *(uint32_t *)_maxCountBuf.contents = 0;

    // 4) Dispatch binTrianglesToTiles (one thread per triangle)
    const uint32_t indexCount = (uint32_t)(_indexBuffer.length / sizeof(uint32_t));
    const uint32_t triCount   = indexCount / 3u;

    if (triCount > 0) {
        id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
        [ce setComputePipelineState:_binTrianglesPSO];

        // buffers: 0=verts, 1=indices, 2=VPOnly, 3=BinUniforms, 4=tileCounts (atomic), 5=maxBuf (atomic)
        [ce setBuffer:_vertexBuffer offset:0 atIndex:0];
        [ce setBuffer:_indexBuffer  offset:0 atIndex:1];

        // VPOnly and BinUniforms must match your Metal structs (ShaderTypes.h)
        typedef struct { simd_float4x4 viewProjectionMatrix; } VPOnly;
        VPOnly vp = { _viewProjection };
        [ce setBytes:&vp length:sizeof(vp) atIndex:2];

        typedef struct { simd_uint2 framebufferPx; simd_uint2 tileSizePx; uint indexCount; uint _pad; } BinUniforms;
        BinUniforms bu = {
            .framebufferPx = { fbW, fbH },
            .tileSizePx    = { _tileW, _tileH },
            .indexCount    = indexCount,
            ._pad          = 0
        };
        [ce setBytes:&bu length:sizeof(bu) atIndex:3];

        [ce setBuffer:_tileCounts offset:0 atIndex:4];
        [ce setBuffer:_maxCountBuf offset:0 atIndex:5];

        // thread_position_in_grid expects dispatchThreads
        const MTLSize threads = MTLSizeMake(triCount, 1, 1);
        const MTLSize tpg     = MTLSizeMake(64,       1, 1);
        [ce dispatchThreads:threads threadsPerThreadgroup:tpg];
        [ce endEncoding];
    }

    // 5) Bake normalized counts to a texture (GPU reads max from buffer)
    {
        id<MTLComputeCommandEncoder> ce2 = [cb computeCommandEncoder];
        [ce2 setComputePipelineState:_countsToTexturePSO];

        // countsToTexture args: buffer(0)=tileCounts, buffer(1)=tilesWH, buffer(2)=maxBuf, texture(0)=outTex
        simd_uint2 tilesWH = { _tilesX, _tilesY };
        [ce2 setBuffer:_tileCounts  offset:0 atIndex:0];
        [ce2 setBytes:&tilesWH      length:sizeof(tilesWH) atIndex:1];
        [ce2 setBuffer:_maxCountBuf offset:0 atIndex:2];
        [ce2 setTexture:_countsTex atIndex:0];

        const MTLSize grid = MTLSizeMake(_tilesX, _tilesY, 1);
        const MTLSize tpg  = MTLSizeMake(8, 8, 1);
        [ce2 dispatchThreads:grid threadsPerThreadgroup:tpg];
        [ce2 endEncoding];
    }
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

    [self polygonTriangulatorHelper];

    _commandQueue = [_device newCommandQueue];

    NSError *e = nil;
    id<MTLLibrary> lib = [_device newDefaultLibrary];

    id<MTLFunction> f0 = [lib newFunctionWithName:@"binTrianglesToTiles"];
    _binTrianglesPSO   = [_device newComputePipelineStateWithFunction:f0 error:&e];
    NSAssert(_binTrianglesPSO, @"binTrianglesToTiles PSO error: %@", e);

    id<MTLFunction> f1 = [lib newFunctionWithName:@"countsToTexture"];
    _countsToTexturePSO = [_device newComputePipelineStateWithFunction:f1 error:&e];
    NSAssert(_countsToTexturePSO, @"countsToTexture PSO error: %@", e);

    _tilesX = 0;
    _tilesY = 0;
    _tileH = 32;
    _tileW = 32;

    return self;
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    _viewportSize = (vector_uint2){ (uint32_t)size.width, (uint32_t)size.height };
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];

    [self prepareTileHeatmapWithCommandBuffer:commandBuffer];

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
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeLines];

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

    [encoder setFragmentTexture:_countsTex atIndex:0];
    [_gridOverlay setCountsTexture:_countsTex tilesX:_tilesX tilesY:_tilesY tileW:_tileW tileH:_tileH];
    [_gridOverlay drawWithEncoder:encoder drawableSize:view.drawableSize];

    [encoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
    [commandBuffer commit];
}

@end
