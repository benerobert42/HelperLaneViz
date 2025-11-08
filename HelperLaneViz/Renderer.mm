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
#include "TileMetrics.h"
#include "CGALTriangulator.h"
#import <iostream>
#import <fstream>

#include <map>

@implementation Renderer
{
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    MTKView *_mtkView;

    id<MTLRenderPipelineState> _pipelineState;
    id<MTLRenderPipelineState> _overdrawPipelineState;
    id<MTLRenderPipelineState> _gridPipelineState;

    id<MTLDepthStencilState> _depthState;
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

- (void) createOverdrawPSOWith:(nonnull MTKView*)mtkView {
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
    _overdrawPipelineState = [_device newRenderPipelineStateWithDescriptor:psDesc error:&err];
    NSAssert(_overdrawPipelineState, @"Overdraw PSO failed: %@", err);
}

- (void) createGridPSOWith:(nonnull MTKView*)mtkView {
    id<MTLLibrary> defaultLibrary = [_device newDefaultLibrary];

    MTLRenderPipelineDescriptor *gdesc = [MTLRenderPipelineDescriptor new];
    gdesc.vertexFunction   = [defaultLibrary newFunctionWithName:@"tilegrid_vs"];
    gdesc.fragmentFunction = [defaultLibrary newFunctionWithName:@"tilegrid_fs"];
    gdesc.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat;

    gdesc.depthAttachmentPixelFormat = mtkView.depthStencilPixelFormat;
    gdesc.stencilAttachmentPixelFormat = MTLPixelFormatInvalid;

    // Enable alpha blending so grid overlays your frame
    auto *ca = gdesc.colorAttachments[0];
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation = MTLBlendOperationAdd;
    ca.alphaBlendOperation = MTLBlendOperationAdd;
    ca.sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    ca.sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
    ca.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    ca.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;

    NSError *err = nil;
    _gridPipelineState = [_device newRenderPipelineStateWithDescriptor:gdesc error:&err];
    NSAssert(_gridPipelineState, @"Grid overlay PSO failed: %@", err);
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


- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView
{
    self = [super init];
    if(self)
    {
        _device = mtkView.device;
        _mtkView = mtkView;     
        mtkView.clearColor = MTLClearColorMake(0, 0, 0, 1);
        mtkView.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
        mtkView.clearDepth = 1.0;

        _pipelineState = [self createPSOWith:mtkView];
        _depthState = [self createDSS];

        [self createOverdrawPSOWith:mtkView];
        _overdrawDepthState  = [self createNoDepthDSS];

        _commandQueue = [_device newCommandQueue];
//        const auto outPath = std::filesystem::temp_directory_path() / "ellipsoidAni80Del2.obj";
//        std::cerr << "[DBG] temp dir: " << std::filesystem::temp_directory_path() << "\n";

//        CGALTriangulateFile("/Users/robi/Downloads/ellipsoid_ani_80_vertices_only.obj", outPath);
//        CGALTriangulator::TriangulateVertexOnlyEllipsoidOBJ("/Users/robi/Downloads/ellipsoid_ani_80_vertices_only.obj",
//                                                            outPath,
//                                                            TriMode::Delaunay);
//        std::vector<Vertex> vertices;
//        std::vector<uint32_t> indicesUnused;
//        SVGLoader::TessellateSvgToMesh("/Users/robi/Downloads/Tractor2.svg", vertices, indicesUnused);

//        double cumulatedEdgeLength;
//        auto vertices = GeometryFactory::CreateVerticesForEllipse(1000, 1.5, 1.0);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMWT(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateCentralTriangulation(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::TriangulatePolygon_CDT(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateStripTriangulation(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMinMaxAreaTriangulation(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMaxMinAreaTriangulation(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateMaxAreaTriangulation(vertices, cumulatedEdgeLength);

//        std::cout << "Cumulated edge length: " << cumulatedEdgeLength << std::endl;

        _gridParams = [self createGridParamsForSegmentCount:30 andOrigin:simd_float2{0, 0} andScale:0.8];

        _useOverDrawPass = false;

        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
        bool success = AssetLoader::LoadPositionsAndIndicesFromObj(_device, @"/Users/robi/Downloads/ellipsoidAni80MWT2.obj", vertices, indices);

        EdgeMetrics metrics  = TriangleFactory::compute_edge_metrics(vertices, indices);
        printf("[MWT]  interior (diagonals) length = %.9f  | boundary = %.9f  | unique_all = %.9f  | nonmanifold=%zu\n",
               metrics.interior_sum, metrics.boundary_sum, metrics.unique_all, metrics.nonmanifold_count);

        _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                    length:vertices.size() * sizeof(Vertex)
                                   options:MTLResourceStorageModeShared];

        _indexBuffer = [_device newBufferWithBytes:indices.data()
                                    length:indices.size() * 4
                                   options:MTLResourceStorageModeShared];

        MTLResourceOptions opts = MTLResourceStorageModeManaged;
        _counterBuffer = [_device newBufferWithLength:sizeof(uint32_t) options:opts];

        // ... after you know framebuffer size and measured tile size:
        simd_int2 fb{ int(mtkView.drawableSize.width), int(mtkView.drawableSize.height) };
        simd_int2 ts{ int(32), int(32) }; // or your chosen (Tw,Th)

        // 1) Collect NDC positions from your vertex buffer (CPU mirror you already have):
        std::vector<simd_float2> ndc;
        ndc.reserve(vertices.size());
        for (const auto& v : vertices) {
            ndc.push_back(simd_make_float2(v.position.x, v.position.y));
        }

        // 2) Convert to pixel-space triangles (indexed)
        std::vector<ttm::TrianglePx> triPx = ttm::build_pixel_tris_from_ndc(ndc, indices, fb);

        // 3) Compute metrics
        ttm::Metrics m = ttm::compute_metrics(triPx, fb, ts);

        printf("[TTM] TTO=%.0f  HTP_P95=%.1f  HTP_mean=%.1f  HTP_median=%.1f  SS_P95=%.1f  SS_mean=%.1f  SS_median=%.1f  BCI=%.3f  (tris=%.0f tiles=%dx%d)\n",
               m.TTO, m.HTP_P95, m.HTP_mean, m.HTP_median, m.SS_P95, m.SS_mean, m.SS_median, m.BCI, m.triCount, ts.x, ts.y);

        [self createGridPSOWith:mtkView];
    }
    return self;
}

#pragma mark - MTKView Delegate Methods

typedef struct {
    vector_uint2 tileSize;
    vector_uint2 framebuffer;
    float        lineWidth;
    simd_float4  lineColor;
} GridUniformsCPU;

- (void)drawTileGridOverlayWithEncoder:(id<MTLRenderCommandEncoder>)enc
                          drawableSize:(CGSize)drawableSize
{
    if (!_gridPipelineState) return;

    // Query the actual per-pass tile size
    const uint32_t tw = MAX(enc.tileWidth, 1);
    const uint32_t th = MAX(enc.tileHeight, 1);

    GridUniformsCPU u;
    u.tileSize    = (vector_uint2){ tw, th };
    u.framebuffer = (vector_uint2){ (uint32_t)drawableSize.width, (uint32_t)drawableSize.height };
    u.lineWidth   = 1.0f;
    u.lineColor   = (simd_float4){ 1, 1, 1, 0.7f };

    [enc pushDebugGroup:@"TileGridOverlay"];

    // ✅ Ensure the overlay is rasterized as a FILLED triangle
    [enc setTriangleFillMode:MTLTriangleFillModeFill];

    // ✅ Ensure depth test doesn't cull the overlay (use your no-depth state)
    if (_overdrawDepthState) {
        [enc setDepthStencilState:_overdrawDepthState]; // compare=Always, writes off
    }

    // ✅ Optional: force a full-coverage viewport in case the app set a smaller one
    MTLViewport vp;
    vp.originX = 0; vp.originY = 0;
    vp.width   = drawableSize.width;
    vp.height  = drawableSize.height;
    vp.znear   = 0.0; vp.zfar = 1.0;
    [enc setViewport:vp];

    [enc setRenderPipelineState:_gridPipelineState];
    [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

    [enc popDebugGroup];
}



- (void)mtkView:(nonnull MTKView *)view drawableSizeWillChange:(CGSize)size
{
    _viewportSize.x = size.width;
    _viewportSize.y = size.height;
}

- (void)drawInMTKView:(nonnull MTKView *)view
{
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    renderPassDescriptor.tileWidth = 32;
    renderPassDescriptor.tileHeight = 32;
    if(renderPassDescriptor != nil) {
        simd_float4x4 viewMat = createLookAtRhs(simd_make_float3(0, 5, 0),
                                                simd_make_float3(0, 0, 0),
                                                simd_make_float3(0, 0, -1));
        simd_float4x4 projMat = makePerspective(M_PI_4, 1, 0.1, 40);
        simd_float4x4 viewProjMat = simd_mul(projMat, viewMat);

        FrameConstants frameConstants{
            .viewProjectionMatrix = viewProjMat,
            .viewPortSize = _viewportSize
        };

        if (_useOverDrawPass) {
            id<MTLRenderCommandEncoder> encoder =
            [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

            [encoder setRenderPipelineState:_overdrawPipelineState];
            [encoder setDepthStencilState:_overdrawDepthState];

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

            [self drawTileGridOverlayWithEncoder:encoder
                                                drawableSize:view.drawableSize];

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
//            [encoder setTriangleFillMode:MTLTriangleFillModeLines];
        

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
                             instanceCount:_gridParams.cols * _gridParams.rows];

            [self drawTileGridOverlayWithEncoder:encoder
                                                drawableSize:view.drawableSize];

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
