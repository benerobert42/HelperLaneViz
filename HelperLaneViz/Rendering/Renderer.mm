//
//  Renderer.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import "Renderer.h"
#import "ShaderTypes.h"
#import "MathUtilities.h"
#import "GridOverlay.h"
#import "PipelineFactory.h"
#import "TriangulationMetrics.h"
#import "../Geometry/GeometryFactory.h"
#import "../Geometry/Triangulation.h"
#import "../InputHandling/SVGLoader.h"
#import "Measurements/TriangulationBenchmark.h"

#import <MetalKit/MetalKit.h>
#import <mach/mach_time.h>

static constexpr uint32_t kDefaultTileSize = 32;

@interface Renderer () <BenchmarkFrameExecutor>
@end

@implementation Renderer {
    // Core Metal objects
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    MTKView *_view;
    
    // Render pipelines
    id<MTLRenderPipelineState> _mainPipeline;
    id<MTLRenderPipelineState> _gridOverlayPipeline;
    
    // Depth states
    id<MTLDepthStencilState> _depthState;
    id<MTLDepthStencilState> _noDepthState;
    
    // Grid overlay
    GridOverlay *_gridOverlay;
    GridParams _gridParams;
    
    // Geometry buffers
    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;
    
    // Current mesh data (for benchmarking)
    std::vector<Vertex> _currentVertices;
    std::vector<uint32_t> _currentIndices;
    
    // View state
    vector_uint2 _viewportSize;
    simd_float4x4 _viewProjectionMatrix;
    
    // Tile heatmap compute pipelines
    id<MTLComputePipelineState> _binTrianglesPipeline;
    id<MTLComputePipelineState> _countsToTexturePipeline;
    
    // Tile heatmap resources
    id<MTLBuffer> _tileCountsBuffer;
    id<MTLBuffer> _maxCountBuffer;
    id<MTLTexture> _heatmapTexture;
    uint32_t _tileCountX;
    uint32_t _tileCountY;
    uint32_t _tileSizePixels;
    
    // GPU timing
    double _lastGPUTime;
    dispatch_semaphore_t _frameSemaphore;
    
    // Visualization flags
    BOOL _showGridOverlay;
    BOOL _showHeatmap;
    BOOL _showOverdraw;
    
    // Overdraw visualization
    id<MTLRenderPipelineState> _overdrawPipeline;
    
    // Overdraw measurement (GPU-based)
    id<MTLRenderPipelineState> _overdrawCountPipeline;
    id<MTLComputePipelineState> _sumOverdrawPipeline;
    id<MTLTexture> _overdrawCountTexture;
    id<MTLBuffer> _overdrawResultsBuffer;
    
    // Helper lane engagement (1x1 dummy texture for forcing derivative computation)
    id<MTLTexture> _helperLaneTexture;
    id<MTLSamplerState> _pointSampler;
}

@synthesize showGridOverlay = _showGridOverlay;
@synthesize showHeatmap = _showHeatmap;
@synthesize showOverdraw = _showOverdraw;

// MARK: Initialization

- (instancetype)initWithMetalKitView:(MTKView *)mtkView {
    if (!(self = [super init])) return nil;
    
    _device = mtkView.device;
    _view = mtkView;
    _commandQueue = [_device newCommandQueue];
    _frameSemaphore = dispatch_semaphore_create(1);
    _lastGPUTime = -1;
    
    // Default visualization settings
    _showGridOverlay = YES;
    _showHeatmap = NO;
    _showOverdraw = NO;

    [self setupView];
    [self setupPipelines];
    [self setupGridOverlay];
    [self setupTileHeatmapPipelines];
    
//  Configure geometry: ellipse with 300 vertices, MWT triangulation, 3x3 grid
    [self setupEllipseWithVertexCount:50
                        semiMajorAxis:1.0f
                        semiMinorAxis:0.2f
                  triangulationMethod:TriangulationMethodConstrainedDelaunay
                     instanceGridCols:10
                             gridRows:50
                         printMetrics:YES];
//    [self loadSVGFromPath:@"/Users/robi/Downloads/Tractor2.svg"
//      triangulationMethod:TriangulationMethodGreedyMaxArea
//         instanceGridCols:1
//                 gridRows:1];

    uint64_t overdrawSum;
    double overdrawRatio;
    [self computeOverdrawMetricsWithOverdrawSum:&overdrawSum overdrawRatio:&overdrawRatio];
    NSLog(@"Overdraw: sum=%llu, ratio=%.3f", overdrawSum, overdrawRatio);

    return self;
}

- (void)setupView {
    _view.clearColor = MTLClearColorMake(0, 0, 0, 1);
    _view.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
    _view.clearDepth = 1.0;
    
    _tileSizePixels = kDefaultTileSize;
    _tileCountX = 0;
    _tileCountY = 0;
}

- (void)setupPipelines {
    NSError *error = nil;
    id<MTLLibrary> library = [_device newDefaultLibrary];
    
    _mainPipeline = MakeMainPipelineState(_device, _view, library, &error);
    NSAssert(_mainPipeline, @"Failed to create main pipeline: %@", error);
    
    _overdrawPipeline = MakeOverdrawPipelineState(_device, _view, library, &error);
    NSAssert(_overdrawPipeline, @"Failed to create overdraw pipeline: %@", error);
    
    _overdrawCountPipeline = MakeOverdrawCountPipelineState(_device, library, &error);
    NSAssert(_overdrawCountPipeline, @"Failed to create overdraw count pipeline: %@", error);
    
    id<MTLFunction> sumOverdrawFunc = [library newFunctionWithName:@"sumOverdrawTexture"];
    _sumOverdrawPipeline = [_device newComputePipelineStateWithFunction:sumOverdrawFunc error:&error];
    NSAssert(_sumOverdrawPipeline, @"Failed to create sumOverdraw pipeline: %@", error);
    
    _gridOverlayPipeline = MakeGridOverlayPipelineState(_device, _view, library, &error);
    NSAssert(_gridOverlayPipeline, @"Failed to create grid overlay pipeline: %@", error);
    
    _depthState = MakeDepthState(_device);
    _noDepthState = MakeNoDepthState(_device);
    
    // Allocate overdraw results buffer (2 uints: total draws, unique pixels)
    _overdrawResultsBuffer = [_device newBufferWithLength:sizeof(uint32_t) * 2
                                                  options:MTLResourceStorageModeShared];
    
    // Create 1x1 dummy texture for helper lane engagement
    MTLTextureDescriptor *helperTexDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:1 height:1 mipmapped:NO];
    _helperLaneTexture = [_device newTextureWithDescriptor:helperTexDesc];
    uint8_t white = 255;
    [_helperLaneTexture replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&white bytesPerRow:1];
    
    MTLSamplerDescriptor *samplerDesc = [MTLSamplerDescriptor new];
    samplerDesc.minFilter = MTLSamplerMinMagFilterNearest;
    samplerDesc.magFilter = MTLSamplerMinMagFilterNearest;
    _pointSampler = [_device newSamplerStateWithDescriptor:samplerDesc];
}

- (void)setupGridOverlay {
    _gridOverlay = [[GridOverlay alloc] initWithPipelineState:_gridOverlayPipeline
                                            depthStencilState:_noDepthState];
}

- (void)setupTileHeatmapPipelines {
    NSError *error = nil;
    id<MTLLibrary> library = [_device newDefaultLibrary];
    
    id<MTLFunction> binTrianglesFunc = [library newFunctionWithName:@"binTrianglesToTiles"];
    _binTrianglesPipeline = [_device newComputePipelineStateWithFunction:binTrianglesFunc error:&error];
    NSAssert(_binTrianglesPipeline, @"Failed to create binTrianglesToTiles pipeline: %@", error);
    
    id<MTLFunction> countsToTextureFunc = [library newFunctionWithName:@"countsToTexture"];
    _countsToTexturePipeline = [_device newComputePipelineStateWithFunction:countsToTextureFunc error:&error];
    NSAssert(_countsToTexturePipeline, @"Failed to create countsToTexture pipeline: %@", error);
}

// MARK: - Geometry Setup

- (void)setupEllipseWithVertexCount:(int)vertexCount
                      semiMajorAxis:(float)a
                      semiMinorAxis:(float)b
                triangulationMethod:(TriangulationMethod)method
                   instanceGridCols:(uint32_t)cols
                           gridRows:(uint32_t)rows
                       printMetrics:(BOOL)printMetrics {
    
    _currentVertices = GeometryFactory::CreateVerticesForEllipse(vertexCount, a, b);
    _currentIndices = [self triangulateVertices:_currentVertices withMethod:method];
    
    [self uploadVertices:_currentVertices indices:_currentIndices];
    
    [self setupOrthographicProjection];
    [self setupInstanceGridWithCols:cols rows:rows];

    if (printMetrics) {
        simd_int2 framebufferSize = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
        simd_int2 tileSize = {(int)_tileSizePixels, (int)_tileSizePixels};
        TriangulationMetrics::computeAndPrintMeshMetrics(_currentVertices, _currentIndices, framebufferSize, tileSize);
    }
}

- (void)setupCircleWithVertexCount:(int)vertexCount
                            radius:(float)radius
               triangulationMethod:(TriangulationMethod)method
                  instanceGridCols:(uint32_t)cols
                          gridRows:(uint32_t)rows
                      printMetrics:(BOOL)printMetrics {
    
    [self setupEllipseWithVertexCount:vertexCount
                        semiMajorAxis:radius
                        semiMinorAxis:radius
                  triangulationMethod:method
                     instanceGridCols:cols
                             gridRows:rows
                         printMetrics:printMetrics];
}

- (BOOL)loadSVGFromPath:(NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows {
    
    // Create triangulator that uses the selected method
    SVGLoader::Triangulator triangulator = [self, method](std::vector<Vertex>& verts) -> std::vector<uint32_t> {
        return [self triangulateVertices:verts withMethod:method];
    };
    
    // Tessellate and triangulate each path with the chosen method
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    if (!SVGLoader::TessellateSvgToMesh(path.UTF8String, vertices, indices, triangulator, 20.0f)) {
        NSLog(@"Failed to load SVG: %@", path);
        return NO;
    }
    
    if (vertices.size() < 3 || indices.empty()) {
        NSLog(@"SVG produced insufficient geometry: %@", path);
        return NO;
    }
    
    _currentVertices = vertices;
    _currentIndices = indices;
    
    [self uploadVertices:vertices indices:indices];
    [self setupOrthographicProjection];
    [self setupInstanceGridWithCols:cols rows:rows];
    
    NSLog(@"Loaded SVG: %@ (%zu vertices, %zu triangles, method=%ld, grid=%dx%d)",
          path, vertices.size(), indices.size() / 3, (long)method, cols, rows);
    
    return YES;
}

- (std::vector<uint32_t>)triangulateVertices:(std::vector<Vertex>&)vertices
                                  withMethod:(TriangulationMethod)method {
    std::vector<uint32_t> indices;

    switch (method) {
        case TriangulationMethodEarClipping:
            indices = Triangulation::earClippingTriangulation(vertices);
            break;
            
        case TriangulationMethodMinimumWeight:
            indices = Triangulation::minimumWeightTriangulation(vertices);
            break;
            
        case TriangulationMethodCentroidFan:
            indices = Triangulation::centroidFanTriangulation(vertices);
            break;
            
        case TriangulationMethodGreedyMaxArea:
            indices = Triangulation::greedyMaxAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodStrip:
            indices = Triangulation::stripTriangulation(vertices);
            break;
            
        case TriangulationMethodMaxMinArea:
            indices = Triangulation::maxMinAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodMinMaxArea:
            indices = Triangulation::minMaxAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodConstrainedDelaunay:
            return Triangulation::constrainedDelaunay(vertices);
    }
    
    return indices;
}

- (void)uploadVertices:(const std::vector<Vertex>&)vertices
               indices:(const std::vector<uint32_t>&)indices {
    
    _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                         length:vertices.size() * sizeof(Vertex)
                                        options:MTLResourceStorageModeShared];
    
    _indexBuffer = [_device newBufferWithBytes:indices.data()
                                        length:indices.size() * sizeof(uint32_t)
                                       options:MTLResourceStorageModeShared];
}

- (void)setupOrthographicProjection {
    simd_float4x4 viewMatrix = createLookAtRhs((simd_float3){0, 0, -3},
                                               (simd_float3){0, 0, 0},
                                               (simd_float3){0, 1, 0});
    
    simd_float4x4 projMatrix = makeOrthoRhs(-1.0f, 1.0f,   // left, right
                                            -1.0f, 1.0f,   // bottom, top
                                             0.0f, 10.0f); // near, far
    
    _viewProjectionMatrix = simd_mul(projMatrix, viewMatrix);
}

- (void)setupInstanceGridWithCols:(uint32_t)cols rows:(uint32_t)rows {
    // Fill the viewport with a grid of instances
    // NDC space is [-1, 1] in both X and Y
    
    // Compute actual geometry bounds
    float minX = FLT_MAX, maxX = -FLT_MAX;
    float minY = FLT_MAX, maxY = -FLT_MAX;
    for (const auto& v : _currentVertices) {
        minX = std::min(minX, v.position.x);
        maxX = std::max(maxX, v.position.x);
        minY = std::min(minY, v.position.y);
        maxY = std::max(maxY, v.position.y);
    }
    const float geomWidth = maxX - minX;
    const float geomHeight = maxY - minY;
    const float geomAspect = (geomHeight > 0.0001f) ? (geomWidth / geomHeight) : 1.0f;
    
    // Add padding around edges and between instances
    const float edgePadding = 0.02f;
    const float instancePadding = 0.01f;

    // Available space after edge padding
    const float availableWidth = 2.0f - 2.0f * edgePadding;
    const float availableHeight = 2.0f - 2.0f * edgePadding;
    
    // Total padding between instances
    const float totalGapX = instancePadding * (cols - 1);
    const float totalGapY = instancePadding * (rows - 1);
    
    // Base cell size
    const float baseCellWidth = (availableWidth - totalGapX) / cols;
    const float baseCellHeight = (availableHeight - totalGapY) / rows;
    
    // Adjust cell dimensions to match geometry aspect ratio
    float cellWidth, cellHeight;
    if (geomAspect > 1.0f) {
        // Geometry is wider than tall
        cellWidth = baseCellWidth;
        cellHeight = baseCellWidth / geomAspect;
    } else {
        // Geometry is taller than wide (or square)
        cellHeight = baseCellHeight;
        cellWidth = baseCellHeight * geomAspect;
    }
    
    // Scale factor: geometry spans geomWidth x geomHeight, fit into cell
    const float scaleX = cellWidth / geomWidth;
    const float scaleY = cellHeight / geomHeight;
    const float shapeScale = std::min(scaleX, scaleY) * 0.9f; // 90% for breathing room
    
    // Origin is bottom-left of the grid in NDC
    const float originX = -1.0f + edgePadding + cellWidth * 0.5f;
    const float originY = -1.0f + edgePadding + cellHeight * 0.5f;
    
    _gridParams = (GridParams){
        .cols = cols,
        .rows = rows,
        .cellSize = {cellWidth + instancePadding, cellHeight + instancePadding},
        .origin = {originX, originY},
        .scale = shapeScale
    };
}

// MARK: - Tile Heatmap Computation

- (void)prepareTileHeatmapWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer {
    if (!_vertexBuffer || !_indexBuffer) return;
    
    const uint32_t framebufferWidth = (uint32_t)_view.drawableSize.width;
    const uint32_t framebufferHeight = (uint32_t)_view.drawableSize.height;
    const uint32_t tilesX = (framebufferWidth + _tileSizePixels - 1) / _tileSizePixels;
    const uint32_t tilesY = (framebufferHeight + _tileSizePixels - 1) / _tileSizePixels;
    const size_t tileCount = (size_t)tilesX * tilesY;
    
    [self reallocateTileResourcesIfNeededWithTilesX:tilesX tilesY:tilesY tileCount:tileCount];
    
    memset(_tileCountsBuffer.contents, 0, tileCount * sizeof(uint32_t));
    *(uint32_t *)_maxCountBuffer.contents = 0;
    
    [self dispatchTriangleBinningWithCommandBuffer:commandBuffer
                                  framebufferWidth:framebufferWidth
                                 framebufferHeight:framebufferHeight];
    
    [self dispatchCountsToTextureWithCommandBuffer:commandBuffer];
}

- (void)reallocateTileResourcesIfNeededWithTilesX:(uint32_t)tilesX
                                           tilesY:(uint32_t)tilesY
                                        tileCount:(size_t)tileCount {
    
    const BOOL needsReallocation = (!_tileCountsBuffer || tilesX != _tileCountX || tilesY != _tileCountY);
    if (!needsReallocation) return;
    
    _tileCountX = tilesX;
    _tileCountY = tilesY;
    
    _tileCountsBuffer = [_device newBufferWithLength:tileCount * sizeof(uint32_t)
                                             options:MTLResourceStorageModeShared];
    
    MTLTextureDescriptor *descriptor = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:tilesX
                                    height:tilesY
                                 mipmapped:NO];
    descriptor.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
    _heatmapTexture = [_device newTextureWithDescriptor:descriptor];
    
    if (!_maxCountBuffer) {
        _maxCountBuffer = [_device newBufferWithLength:sizeof(uint32_t)
                                               options:MTLResourceStorageModeShared];
    }
}

- (void)dispatchTriangleBinningWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                                framebufferWidth:(uint32_t)framebufferWidth
                               framebufferHeight:(uint32_t)framebufferHeight {
    
    const uint32_t indexCount = (uint32_t)(_indexBuffer.length / sizeof(uint32_t));
    const uint32_t triangleCount = indexCount / 3;
    
    if (triangleCount == 0) return;
    
    id<MTLComputeCommandEncoder> encoder = [commandBuffer computeCommandEncoder];
    [encoder setComputePipelineState:_binTrianglesPipeline];
    
    [encoder setBuffer:_vertexBuffer offset:0 atIndex:0];
    [encoder setBuffer:_indexBuffer offset:0 atIndex:1];
    
    struct { simd_float4x4 viewProjectionMatrix; } vpUniforms = { _viewProjectionMatrix };
    [encoder setBytes:&vpUniforms length:sizeof(vpUniforms) atIndex:2];
    
    struct {
        simd_uint2 framebufferSize;
        simd_uint2 tileSize;
        uint32_t indexCount;
        uint32_t _padding;
    } binUniforms = {
        .framebufferSize = {framebufferWidth, framebufferHeight},
        .tileSize = {_tileSizePixels, _tileSizePixels},
        .indexCount = indexCount,
        ._padding = 0
    };
    [encoder setBytes:&binUniforms length:sizeof(binUniforms) atIndex:3];
    
    [encoder setBuffer:_tileCountsBuffer offset:0 atIndex:4];
    [encoder setBuffer:_maxCountBuffer offset:0 atIndex:5];
    
    [encoder dispatchThreads:MTLSizeMake(triangleCount, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(64, 1, 1)];
    [encoder endEncoding];
}

- (void)dispatchCountsToTextureWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer {
    id<MTLComputeCommandEncoder> encoder = [commandBuffer computeCommandEncoder];
    [encoder setComputePipelineState:_countsToTexturePipeline];
    
    [encoder setBuffer:_tileCountsBuffer offset:0 atIndex:0];
    
    simd_uint2 tileGridSize = {_tileCountX, _tileCountY};
    [encoder setBytes:&tileGridSize length:sizeof(tileGridSize) atIndex:1];
    
    [encoder setBuffer:_maxCountBuffer offset:0 atIndex:2];
    [encoder setTexture:_heatmapTexture atIndex:0];
    
    [encoder dispatchThreads:MTLSizeMake(_tileCountX, _tileCountY, 1)
       threadsPerThreadgroup:MTLSizeMake(8, 8, 1)];
    [encoder endEncoding];
}

// MARK: - Overdraw Measurement (GPU-based)

- (void)reallocateOverdrawTextureIfNeeded {
    const uint32_t width = (uint32_t)_view.drawableSize.width;
    const uint32_t height = (uint32_t)_view.drawableSize.height;
    
    if (_overdrawCountTexture &&
        _overdrawCountTexture.width == width &&
        _overdrawCountTexture.height == height) {
        return;
    }
    
    MTLTextureDescriptor *desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    desc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModePrivate;
    _overdrawCountTexture = [_device newTextureWithDescriptor:desc];
}

- (void)computeOverdrawMetricsWithOverdrawSum:(uint64_t*)outSum overdrawRatio:(double*)outRatio {
    if (!_vertexBuffer || !_indexBuffer) {
        if (outSum) *outSum = 0;
        if (outRatio) *outRatio = 0.0;
        return;
    }
    
    [self reallocateOverdrawTextureIfNeeded];
    
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Step 1: Render triangles to overdraw count texture
    MTLRenderPassDescriptor *renderPass = [MTLRenderPassDescriptor new];
    renderPass.colorAttachments[0].texture = _overdrawCountTexture;
    renderPass.colorAttachments[0].loadAction = MTLLoadActionClear;
    renderPass.colorAttachments[0].storeAction = MTLStoreActionStore;
    renderPass.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 0);
    
    id<MTLRenderCommandEncoder> renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPass];
    [renderEncoder setRenderPipelineState:_overdrawCountPipeline];
    [renderEncoder setCullMode:MTLCullModeNone];
    [renderEncoder setTriangleFillMode:MTLTriangleFillModeFill];
    
    FrameConstants frameConstants = {
        .viewProjectionMatrix = _viewProjectionMatrix,
        .viewPortSize = _viewportSize
    };
    
    [renderEncoder setVertexBuffer:_vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [renderEncoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [renderEncoder setVertexBytes:&_gridParams length:sizeof(_gridParams) atIndex:VertexInputGridParams];
    
    const NSUInteger indexCount = _indexBuffer.length / sizeof(uint32_t);
    const NSUInteger instanceCount = _gridParams.cols * _gridParams.rows;
    
    [renderEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                              indexCount:indexCount
                               indexType:MTLIndexTypeUInt32
                             indexBuffer:_indexBuffer
                       indexBufferOffset:0
                           instanceCount:instanceCount];
    [renderEncoder endEncoding];
    
    // Step 2: Clear results buffer
    uint32_t* resultsPtr = (uint32_t*)_overdrawResultsBuffer.contents;
    resultsPtr[0] = 0;  // Total draws
    resultsPtr[1] = 0;  // Unique pixels
    
    // Step 3: Sum overdraw texture with compute shader
    id<MTLComputeCommandEncoder> computeEncoder = [commandBuffer computeCommandEncoder];
    [computeEncoder setComputePipelineState:_sumOverdrawPipeline];
    [computeEncoder setTexture:_overdrawCountTexture atIndex:0];
    [computeEncoder setBuffer:_overdrawResultsBuffer offset:0 atIndex:0];
    
    const uint32_t width = (uint32_t)_overdrawCountTexture.width;
    const uint32_t height = (uint32_t)_overdrawCountTexture.height;
    [computeEncoder dispatchThreads:MTLSizeMake(width, height, 1)
              threadsPerThreadgroup:MTLSizeMake(16, 16, 1)];
    [computeEncoder endEncoding];
    
    // Wait for completion
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    
    // Read results
    uint64_t totalDraws = resultsPtr[0];
    uint64_t uniquePixels = resultsPtr[1];
    
    if (outSum) *outSum = totalDraws;
    if (outRatio) *outRatio = (uniquePixels > 0) ? ((double)totalDraws / uniquePixels) : 0.0;
}

// MARK: - MTKViewDelegate

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    _viewportSize = (vector_uint2){(uint32_t)size.width, (uint32_t)size.height};
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Prepare heatmap if grid overlay with heatmap is enabled
    if (_showGridOverlay && _showHeatmap) {
        [self prepareTileHeatmapWithCommandBuffer:commandBuffer];
    }

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if (!renderPassDescriptor) {
        [commandBuffer commit];
        return;
    }
    
    renderPassDescriptor.tileWidth = _tileSizePixels;
    renderPassDescriptor.tileHeight = _tileSizePixels;
    
    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
    
    // Choose between overdraw visualization and normal rendering
    if (_showOverdraw) {
        [self encodeOverdrawGeometryWithEncoder:encoder];
    } else {
        [self encodeMainGeometryWithEncoder:encoder];
    }

    // Draw grid overlay if enabled (not shown during overdraw mode)
    if (_showGridOverlay && !_showOverdraw) {
        [self encodeGridOverlayWithEncoder:encoder drawableSize:view.drawableSize];
    }

    [encoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
    [commandBuffer commit];
}

- (void)encodeMainGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder {
    [encoder setRenderPipelineState:_mainPipeline];
    [encoder setDepthStencilState:_depthState];
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill];

    FrameConstants frameConstants = {
        .viewProjectionMatrix = _viewProjectionMatrix,
        .viewPortSize = _viewportSize
    };
    
    [encoder setVertexBuffer:_vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [encoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [encoder setVertexBytes:&_gridParams length:sizeof(_gridParams) atIndex:VertexInputGridParams];
    
    // Bind dummy texture to engage helper lanes via texture sampling
    [encoder setFragmentTexture:_helperLaneTexture atIndex:0];
    [encoder setFragmentSamplerState:_pointSampler atIndex:0];
    
    const NSUInteger indexCount = _indexBuffer.length / sizeof(uint32_t);
    const NSUInteger instanceCount = _gridParams.cols * _gridParams.rows;
    
    [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:indexCount
                         indexType:MTLIndexTypeUInt32
                       indexBuffer:_indexBuffer
                 indexBufferOffset:0
                     instanceCount:instanceCount];
}

- (void)encodeOverdrawGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder {
    // Use overdraw pipeline with additive blending - each triangle adds to pixel brightness
    [encoder setRenderPipelineState:_overdrawPipeline];
    [encoder setDepthStencilState:_noDepthState]; // Disable depth test for overdraw
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill]; // Always fill for overdraw

    FrameConstants frameConstants = {
        .viewProjectionMatrix = _viewProjectionMatrix,
        .viewPortSize = _viewportSize
    };
    
    [encoder setVertexBuffer:_vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [encoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [encoder setVertexBytes:&_gridParams length:sizeof(_gridParams) atIndex:VertexInputGridParams];
    
    const NSUInteger indexCount = _indexBuffer.length / sizeof(uint32_t);
    const NSUInteger instanceCount = _gridParams.cols * _gridParams.rows;
    
    [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:indexCount
                         indexType:MTLIndexTypeUInt32
                       indexBuffer:_indexBuffer
                 indexBufferOffset:0
                     instanceCount:instanceCount];
}

- (void)encodeGridOverlayWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                        drawableSize:(CGSize)drawableSize {
    
    MTLViewport viewport = {
        .originX = 0,
        .originY = 0,
        .width = drawableSize.width,
        .height = drawableSize.height,
        .znear = 0,
        .zfar = 1
    };
    [encoder setViewport:viewport];
    
    [_gridOverlay drawWithEncoder:encoder
                   heatmapTexture:_heatmapTexture
                         tileSize:_tileSizePixels
                     drawableSize:drawableSize
                      showHeatmap:_showHeatmap];
}

// MARK: - BenchmarkFrameExecutor Protocol

- (double)prepareSceneWithVertexCount:(int)vertexCount
                        semiMajorAxis:(float)a
                        semiMinorAxis:(float)b
                  triangulationMethod:(int)method
                     instanceGridCols:(uint32_t)cols
                             gridRows:(uint32_t)rows {
    
    // Measure triangulation time
    uint64_t startTime = mach_absolute_time();
    
    [self setupEllipseWithVertexCount:vertexCount
                        semiMajorAxis:a
                        semiMinorAxis:b
                  triangulationMethod:(TriangulationMethod)method
                     instanceGridCols:cols
                             gridRows:rows
                         printMetrics:NO];
    
    uint64_t endTime = mach_absolute_time();
    
    // Convert to seconds
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    double nanoseconds = (double)(endTime - startTime) * timebase.numer / timebase.denom;
    return nanoseconds / 1e9;
}

- (double)executeFrameAndMeasureGPUTime {
    __block double gpuTime = -1;
    
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Always prepare and render heatmap during benchmarks
    [self prepareTileHeatmapWithCommandBuffer:commandBuffer];
    
    MTLRenderPassDescriptor *renderPassDescriptor = _view.currentRenderPassDescriptor;
    if (!renderPassDescriptor) {
        [commandBuffer commit];
        [commandBuffer waitUntilCompleted];
        return -1;
    }
    
    renderPassDescriptor.tileWidth = _tileSizePixels;
    renderPassDescriptor.tileHeight = _tileSizePixels;
    
    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
    
    [self encodeMainGeometryWithEncoder:encoder];
    
    // Always render with heatmap during benchmark for consistent measurements
    MTLViewport viewport = {
        .originX = 0,
        .originY = 0,
        .width = _view.drawableSize.width,
        .height = _view.drawableSize.height,
        .znear = 0,
        .zfar = 1
    };
    [encoder setViewport:viewport];
    [_gridOverlay drawWithEncoder:encoder
                   heatmapTexture:_heatmapTexture
                         tileSize:_tileSizePixels
                     drawableSize:_view.drawableSize
                      showHeatmap:YES];
    
    [encoder endEncoding];
    [commandBuffer presentDrawable:_view.currentDrawable];
    
    // Use GPU timestamps for accurate timing
    [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> buffer) {
        if (buffer.GPUStartTime > 0 && buffer.GPUEndTime > 0) {
            gpuTime = buffer.GPUEndTime - buffer.GPUStartTime;
        }
    }];
    
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    
    return gpuTime;
}

- (void)getCurrentMeshVertices:(std::vector<Vertex>*)outVertices
                       indices:(std::vector<uint32_t>*)outIndices {
    if (outVertices) *outVertices = _currentVertices;
    if (outIndices) *outIndices = _currentIndices;
}

// MARK: - Benchmark API

- (void)runDefaultBenchmark {
    // Use the standard test matrix: 3 shapes × 4 vertex counts × 3 instance counts = 36 scenes
    Benchmark::BenchmarkConfig config = Benchmark::BenchmarkConfig::standardTestMatrix();
    config.framebufferSize = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
    config.tileSize = _tileSizePixels;
    
    [self runBenchmarkInternal:config];
}

- (void)runQuickBenchmark {
    // Reduced test set for quick iteration
    Benchmark::BenchmarkConfig config = Benchmark::BenchmarkConfig::quickTest();
    config.framebufferSize = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
    config.tileSize = _tileSizePixels;
    
    [self runBenchmarkInternal:config];
}

- (void)runBenchmarkInternal:(const Benchmark::BenchmarkConfig&)config {
    _view.paused = YES;
    
    Benchmark::BenchmarkResults results = Benchmark::runBenchmark(self, config);
    
    // Print summary table
    results.printSummary();
    
    // Print detailed report
    results.printDetailedReport();
    
    // Export CSV
    std::string csv = results.toCSV();
    NSString *csvString = [NSString stringWithUTF8String:csv.c_str()];
    NSString *tempPath = [NSTemporaryDirectory() stringByAppendingPathComponent:@"triangulation_benchmark.csv"];
    [csvString writeToFile:tempPath atomically:YES encoding:NSUTF8StringEncoding error:nil];
    printf("\n📊 CSV exported to: %s\n\n", tempPath.UTF8String);
    
    // Restore a default scene
    [self setupEllipseWithVertexCount:300
                        semiMajorAxis:1.0f
                        semiMinorAxis:0.5f
                  triangulationMethod:TriangulationMethodMinimumWeight
                     instanceGridCols:3
                             gridRows:3
                         printMetrics:NO];
    
    _view.paused = NO;
}

@end
