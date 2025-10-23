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

#include <map>

// ====== BEGIN: Tile Explorer ======
@interface TileExplorer : NSObject
@property (nonatomic, strong) id<MTLRenderPipelineState> pso;
@end

@implementation TileExplorer

static inline double median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
    return v[v.size()/2];
}

static std::pair<int,int> estimateTileSizeFromSeries(const std::vector<double>& series) {
    // 1st-order diff (discrete derivative)
    std::vector<double> d;
    d.reserve(series.size()-1);
    for (size_t i = 1; i < series.size(); ++i) {
        d.push_back(series[i] - series[i-1]);
    }

    // Threshold: mean + 2*stddev → mark peaks
    double mean=0;
    double var=0;
    for (double x: d) {
        mean += x; mean /= std::max<size_t>(1, d.size());
    }
    for (double x: d) {
        var += (x-mean)*(x-mean);
    }
    var /= std::max<size_t>(1, d.size());
    double stddev = std::sqrt(var);
    double th = mean + 2.0*stddev;

    std::vector<int> peaks;
    for (int i=1; i<(int)d.size()-1; ++i) {
        if (d[i] > th && d[i] >= d[i-1] && d[i] >= d[i+1]) peaks.push_back(i);
    }
    if (peaks.size() < 3) return {0,0};

    // Distance histogram between successive peaks → mode ≈ tile dimension
    std::map<int,int> hist;
    for (size_t i=1;i<peaks.size();++i) {
        int dist = peaks[i] - peaks[i-1];
        if (dist > 0) hist[dist]++;
    }
    int bestDist=0, bestCount=0;
    for (auto& kv : hist) if (kv.second > bestCount) { bestCount = kv.second; bestDist = kv.first; }

    // Confidence: how stable are distances? crude: fraction of most common distance
    int confidence = (int)std::lround(100.0 * double(bestCount) / std::max<size_t>(1, hist.size()));
    return {bestDist, confidence};
}

- (instancetype)initWithDevice:(id<MTLDevice>)device colorFormat:(MTLPixelFormat)fmt {
    if ((self = [super init])) {
        NSError *err = nil;
        id<MTLLibrary> lib = [device newDefaultLibrary];
        MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction   = [lib newFunctionWithName:@"probeVS"];
        d.fragmentFunction = [lib newFunctionWithName:@"probeFS"];
        d.colorAttachments[0].pixelFormat = fmt;
        // Depth/stencil off. No blending. Keep it bare.
        _pso = [device newRenderPipelineStateWithDescriptor:d error:&err];
        NSAssert(_pso, @"TileExplorer PSO failed: %@", err);
    }
    return self;
}

// Runs a sweep in one axis and returns per-step GPU time (ms).
- (std::vector<double>)runSweepOnView:(MTKView*)view
                        commandQueue:(id<MTLCommandQueue>)queue
                            axisIsX:(BOOL)axisIsX
                           fullSize:(MTLSize)full
                     samplesPerStep:(int)samplesPerStep
{
    const int steps = axisIsX ? (int)full.width : (int)full.height;
    std::vector<double> series;
    series.reserve(steps);

    for (int s = 1; s <= steps; ++s) {
        std::vector<double> reps;
        reps.reserve(samplesPerStep);

        for (int rep = 0; rep < samplesPerStep; ++rep) {
            @autoreleasepool {
                // Acquire a drawable; if it's not ready yet, retry this rep shortly.
                id<CAMetalDrawable> drawable = view.currentDrawable;
                if (!drawable) {
                    [NSThread sleepForTimeInterval:0.005];
                    --rep;
                    continue; }

                // Minimal render pass targeting the drawable texture
                MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
                rp.colorAttachments[0].texture     = drawable.texture;
                rp.colorAttachments[0].loadAction  = (rep == 0) ? MTLLoadActionClear : MTLLoadActionLoad;
                rp.colorAttachments[0].storeAction = MTLStoreActionStore;
                rp.colorAttachments[0].clearColor  = MTLClearColorMake(0, 0, 0, 1);

                id<MTLCommandBuffer> cb = [queue commandBuffer];

                // Scissor: sweep one axis, keep the other full
                MTLScissorRect sc;
                sc.x      = 0;
                sc.y      = 0;
                sc.width  = axisIsX ? (NSUInteger)s : (NSUInteger)full.width;
                sc.height = axisIsX ? (NSUInteger)full.height : (NSUInteger)s;

                id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
                [enc setRenderPipelineState:self.pso];
                [enc setScissorRect:sc];
                // Full-screen triangle (cheap shader)
                [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
                [enc endEncoding];

                [cb presentDrawable:drawable];
                [cb commit];
                [cb waitUntilCompleted];

                const double ms = (cb.GPUEndTime > cb.GPUStartTime)
                                  ? (cb.GPUEndTime - cb.GPUStartTime) * 1000.0
                                  : 0.0;
                reps.push_back(ms);
            } // autoreleasepool
        }

        // Median per step to suppress noise
        series.push_back(median(reps));
    }
    return series;
}


- (void)exploreOnView:(MTKView*)view commandQueue:(id<MTLCommandQueue>)queue
{
    const int samplesPerStep = 3; // tune 3..7
    const MTLSize full = MTLSizeMake((NSUInteger)view.drawableSize.width,
                                     (NSUInteger)view.drawableSize.height, 1);

    // Warm up a couple of frames
    for (int i=0;i<3;++i) {
        (void)[view currentDrawable];
        [NSThread sleepForTimeInterval:0.005]; }

    // Horizontal sweep
    auto sx = [self runSweepOnView:view
                      commandQueue:queue
                           axisIsX:YES
                          fullSize:full
                    samplesPerStep:samplesPerStep];
    auto [tw, confx] = estimateTileSizeFromSeries(sx);

    // Vertical sweep
    auto sy = [self runSweepOnView:view
                      commandQueue:queue
                           axisIsX:NO
                          fullSize:full
                    samplesPerStep:samplesPerStep];
    auto [th, confy] = estimateTileSizeFromSeries(sy);

    // Fallback if we didn’t detect peaks
    if (tw <= 0) tw = 32;
    if (th <= 0) th = 32;

    const int tilesX = (int)std::ceil(double(full.width)  / double(tw));
    const int tilesY = (int)std::ceil(double(full.height) / double(th));

    printf("[TileExplorer] drawableSize = %dx%d\n", (int)full.width, (int)full.height);
    printf("[TileExplorer] estimated tile ~ %dx%d (confidence X=%d%%, Y=%d%%)\n", tw, th, confx, confy);
    printf("[TileExplorer] estimated total tiles ~ %d x %d = %d\n",
           tilesX, tilesY, tilesX*tilesY);
}

@end
// ====== END: Tile Explorer ======

@implementation Renderer
{
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLRenderPipelineState> _pipelineState;
    id<MTLDepthStencilState> _depthState;

    id<MTLRenderPipelineState> _overdrawPipelineState;
    id<MTLDepthStencilState> _overdrawDepthState;

    id<MTLRenderPipelineState> _gridPipeline;

    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;

    id<MTLBuffer> _counterBuffer;

    vector_uint2 _viewportSize;

    GridParams _gridParams;
    bool _useOverDrawPass;
    TileExplorer *_tileExplorer;
    MTKView *_mtkView;
    BOOL _isMeasuringTiles;
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

- (GridParams)createGridParamsForSegmentCount:(uint32_t)segmentCount {
    GridParams params{
        .cols = segmentCount,
        .rows = segmentCount,
        .cellSize = {2.f / segmentCount, 2.f / segmentCount},
        .origin = {-0.99, -0.99},
        .scale = 0.5
    };
    return params;
}

- (void)measureTilesNow {
    if (_isMeasuringTiles || !_mtkView || !_commandQueue) return;
    _isMeasuringTiles = YES;

    // Run on main to keep drawable acquisition straightforward.
    dispatch_async(dispatch_get_main_queue(), ^{
        @autoreleasepool {
            // Optional: briefly clear once so the first step isn't dominated by clear cost.
            // (You can skip this if you want)
            id<CAMetalDrawable> drawable = _mtkView.currentDrawable;
            if (drawable) {
                MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
                rp.colorAttachments[0].texture     = drawable.texture;
                rp.colorAttachments[0].loadAction  = MTLLoadActionClear;
                rp.colorAttachments[0].storeAction = MTLStoreActionStore;
                rp.colorAttachments[0].clearColor  = MTLClearColorMake(0,0,0,1);
                id<MTLCommandBuffer> cb = [_commandQueue commandBuffer];
                id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
                [enc endEncoding];
                [cb presentDrawable:drawable];
                [cb commit];
                [cb waitUntilCompleted];
            }

            // Do the actual sweep (prints results to the console)
            [_tileExplorer exploreOnView:_mtkView commandQueue:_commandQueue];
        }
        _isMeasuringTiles = NO;
    });
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

        _overdrawPipelineState = [self createOverdrawPSOWith:mtkView];
        _overdrawDepthState    = [self createNoDepthDSS];

        _commandQueue = [_device newCommandQueue];

//        std::vector<Vertex> vertices;
//        std::vector<uint32_t> indicesUnused;
//        SVGLoader::TessellateSvgToMesh("/Users/robi/Downloads/Tractor2.svg", vertices, indicesUnused);

        double cumulatedEdgeLength;
        auto vertices = GeometryFactory::CreateVerticesForEllipse(100, 1, 1);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMWT(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateCentralTriangulation(vertices);
        std::vector<uint32_t> indices = TriangleFactory::TriangulatePolygon_CDT(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateStripTriangulation(vertices);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMinMaxAreaTriangulation(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateConvexMaxMinAreaTriangulation(vertices, cumulatedEdgeLength);
//        std::vector<uint32_t> indices = TriangleFactory::CreateMaxAreaTriangulation(vertices, cumulatedEdgeLength);

        std::cout << "Cumulated edge length: " << cumulatedEdgeLength << std::endl;

        _gridParams = [self createGridParamsForSegmentCount:2];

        _useOverDrawPass = false;

        _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                    length:vertices.size() * sizeof(Vertex)
                                   options:MTLResourceStorageModeShared];

        _indexBuffer = [_device newBufferWithBytes:indices.data()
                                    length:indices.size() * 4
                                   options:MTLResourceStorageModeShared];

        MTLResourceOptions opts = MTLResourceStorageModeManaged;
        _counterBuffer = [_device newBufferWithLength:sizeof(uint32_t) options:opts];

        _tileExplorer = [[TileExplorer alloc] initWithDevice:_device
                                                     colorFormat:mtkView.colorPixelFormat];
        _isMeasuringTiles = NO;

        NSError *gerr = nil;
        id<MTLLibrary> lib = [_device newDefaultLibrary];

        MTLRenderPipelineDescriptor *gdesc = [MTLRenderPipelineDescriptor new];
        gdesc.vertexFunction   = [lib newFunctionWithName:@"tilegrid_vs"];
        gdesc.fragmentFunction = [lib newFunctionWithName:@"tilegrid_fs"];
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

        _gridPipeline = [_device newRenderPipelineStateWithDescriptor:gdesc error:&gerr];
        NSAssert(_gridPipeline, @"Grid overlay PSO failed: %@", gerr);
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
    if (!_gridPipeline) return;

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

    [enc setRenderPipelineState:_gridPipeline];
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
    if (_isMeasuringTiles) {
        // Skip normal rendering while the sweep is running
        return;
    }
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    renderPassDescriptor.tileWidth = 32;
    renderPassDescriptor.tileHeight = 32;
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
