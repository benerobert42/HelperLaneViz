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

#import <MetalKit/MetalKit.h>

// =============================================================================
// MARK: - Constants
// =============================================================================

static constexpr uint32_t kDefaultTileSize = 32;

// =============================================================================
// MARK: - Triangulation Method Selection
// =============================================================================

typedef NS_ENUM(NSInteger, TriangulationMethod) {
    TriangulationMethodMinimumWeight,
    TriangulationMethodCentroidFan,
    TriangulationMethodGreedyMaxArea,
    TriangulationMethodStrip,
    TriangulationMethodMaxMinArea,
    TriangulationMethodMinMaxArea,
    TriangulationMethodConstrainedDelaunay
};

// =============================================================================
// MARK: - Implementation
// =============================================================================

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
}

// =============================================================================
// MARK: - Initialization
// =============================================================================

- (instancetype)initWithMetalKitView:(MTKView *)mtkView {
    if (!(self = [super init])) return nil;
    
    _device = mtkView.device;
    _view = mtkView;
    _commandQueue = [_device newCommandQueue];
    
    [self setupView];
    [self setupPipelines];
    [self setupGridOverlay];
    [self setupTileHeatmapPipelines];
    
    // Configure geometry: ellipse with 300 vertices, MWT triangulation, 3x3 grid
    [self setupEllipseWithVertexCount:300
                           semiMajorAxis:1.0f
                           semiMinorAxis:0.5f
                     triangulationMethod:TriangulationMethodMinimumWeight
                           instanceGridSize:3
                            printMetrics:YES];
    
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
    
    _gridOverlayPipeline = MakeGridOverlayPipelineState(_device, _view, library, &error);
    NSAssert(_gridOverlayPipeline, @"Failed to create grid overlay pipeline: %@", error);
    
    _depthState = MakeDepthState(_device);
    _noDepthState = MakeNoDepthState(_device);
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

// =============================================================================
// MARK: - Geometry Setup
// =============================================================================

- (void)setupEllipseWithVertexCount:(int)vertexCount
                      semiMajorAxis:(float)a
                      semiMinorAxis:(float)b
                triangulationMethod:(TriangulationMethod)method
                  instanceGridSize:(uint32_t)gridSize
                       printMetrics:(BOOL)printMetrics {
    
    // Generate ellipse vertices
    std::vector<Vertex> vertices = GeometryFactory::CreateVerticesForEllipse(vertexCount, a, b);
    
    // Triangulate using selected method
    std::vector<uint32_t> indices = [self triangulateVertices:vertices withMethod:method];
    
    // Upload to GPU
    [self uploadVertices:vertices indices:indices];
    
    // Configure orthographic projection for 2D viewing
    [self setupOrthographicProjection];
    
    // Configure instancing grid
    [self setupInstanceGridWithSize:gridSize origin:simd_make_float2(-0.7f, -0.7f) scale:0.4f];
    
    // Print metrics if requested
    if (printMetrics) {
        simd_int2 framebufferSize = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
        simd_int2 tileSize = {(int)_tileSizePixels, (int)_tileSizePixels};
        TriangulationMetrics::computeAndPrintMeshMetrics(vertices, indices, framebufferSize, tileSize);
    }
}

- (void)setupCircleWithVertexCount:(int)vertexCount
                            radius:(float)radius
               triangulationMethod:(TriangulationMethod)method
                 instanceGridSize:(uint32_t)gridSize
                      printMetrics:(BOOL)printMetrics {
    
    // Circle is an ellipse with equal semi-axes
    [self setupEllipseWithVertexCount:vertexCount
                        semiMajorAxis:radius
                        semiMinorAxis:radius
                  triangulationMethod:method
                    instanceGridSize:gridSize
                         printMetrics:printMetrics];
}

- (std::vector<uint32_t>)triangulateVertices:(std::vector<Vertex>&)vertices
                                  withMethod:(TriangulationMethod)method {
    Triangulation::Result result;
    
    switch (method) {
        case TriangulationMethodMinimumWeight:
            result = Triangulation::minimumWeightTriangulation(vertices);
            break;
            
        case TriangulationMethodCentroidFan:
            result = Triangulation::centroidFanTriangulation(vertices);
            break;
            
        case TriangulationMethodGreedyMaxArea:
            result = Triangulation::greedyMaxAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodStrip:
            result = Triangulation::stripTriangulation(vertices);
            break;
            
        case TriangulationMethodMaxMinArea:
            result = Triangulation::maxMinAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodMinMaxArea:
            result = Triangulation::minMaxAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodConstrainedDelaunay:
            return Triangulation::constrainedDelaunay(vertices);
    }
    
    return result.indices;
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

- (void)setupInstanceGridWithSize:(uint32_t)gridSize
                           origin:(simd_float2)origin
                            scale:(float)scale {
    _gridParams = (GridParams){
        .cols = gridSize,
        .rows = gridSize,
        .cellSize = {2.0f / gridSize, 2.0f / gridSize},
        .origin = origin,
        .scale = scale
    };
}

// =============================================================================
// MARK: - Tile Heatmap Computation
// =============================================================================

- (void)prepareTileHeatmapWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer {
    if (!_vertexBuffer || !_indexBuffer) return;
    
    const uint32_t framebufferWidth = (uint32_t)_view.drawableSize.width;
    const uint32_t framebufferHeight = (uint32_t)_view.drawableSize.height;
    const uint32_t tilesX = (framebufferWidth + _tileSizePixels - 1) / _tileSizePixels;
    const uint32_t tilesY = (framebufferHeight + _tileSizePixels - 1) / _tileSizePixels;
    const size_t tileCount = (size_t)tilesX * tilesY;
    
    // Reallocate resources if tile grid changed
    [self reallocateTileResourcesIfNeededWithTilesX:tilesX tilesY:tilesY tileCount:tileCount];
    
    // Clear tile counts and max buffer
    memset(_tileCountsBuffer.contents, 0, tileCount * sizeof(uint32_t));
    *(uint32_t *)_maxCountBuffer.contents = 0;
    
    // Dispatch triangle binning compute pass
    [self dispatchTriangleBinningWithCommandBuffer:commandBuffer
                                  framebufferWidth:framebufferWidth
                                 framebufferHeight:framebufferHeight];
    
    // Dispatch counts-to-texture compute pass
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
    
    // Set buffers
    [encoder setBuffer:_vertexBuffer offset:0 atIndex:0];
    [encoder setBuffer:_indexBuffer offset:0 atIndex:1];
    
    // Set view-projection matrix
    struct { simd_float4x4 viewProjectionMatrix; } vpUniforms = { _viewProjectionMatrix };
    [encoder setBytes:&vpUniforms length:sizeof(vpUniforms) atIndex:2];
    
    // Set binning uniforms
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
    
    // Dispatch one thread per triangle
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
    
    // Dispatch one thread per tile
    [encoder dispatchThreads:MTLSizeMake(_tileCountX, _tileCountY, 1)
       threadsPerThreadgroup:MTLSizeMake(8, 8, 1)];
    [encoder endEncoding];
}

// =============================================================================
// MARK: - MTKViewDelegate
// =============================================================================

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    _viewportSize = (vector_uint2){(uint32_t)size.width, (uint32_t)size.height};
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Compute tile heatmap
    [self prepareTileHeatmapWithCommandBuffer:commandBuffer];
    
    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if (!renderPassDescriptor) {
        [commandBuffer commit];
        return;
    }
    
    renderPassDescriptor.tileWidth = _tileSizePixels;
    renderPassDescriptor.tileHeight = _tileSizePixels;
    
    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
    
    // Draw main geometry
    [self encodeMainGeometryWithEncoder:encoder];
    
    // Draw tile heatmap overlay
    [self encodeHeatmapOverlayWithEncoder:encoder drawableSize:view.drawableSize];
    
    [encoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
    [commandBuffer commit];
}

- (void)encodeMainGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder {
    [encoder setRenderPipelineState:_mainPipeline];
    [encoder setDepthStencilState:_depthState];
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeLines];
    
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

- (void)encodeHeatmapOverlayWithEncoder:(id<MTLRenderCommandEncoder>)encoder
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
                     drawableSize:drawableSize];
}

@end
