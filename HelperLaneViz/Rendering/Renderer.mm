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
#import "../ThirdParty/ImGUI/ImGUIHelper.h"
#import "../Geometry/GeometryFactory.h"
#import "../Geometry/Triangulation.h"
#import "../InputHandling/SVGLoader.h"

#import <MetalKit/MetalKit.h>
#import <mach/mach_time.h>

#include "../../external/imgui/imgui.h"

static constexpr uint32_t kDefaultTileSize = 32;

@implementation Renderer {
    // Core Metal objects
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    MTKView *_view;
    
    // Render pipelines
    id<MTLRenderPipelineState> _mainPipeline;
    id<MTLRenderPipelineState> _wireframePipeline;
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
    
    // GPU timing
    double _lastGPUTime;
    dispatch_semaphore_t _frameSemaphore;
    
    // Visualization flags
    BOOL _showGridOverlay;
    VisualizationMode _visualizationMode;
    uint32_t _tileSizePx;
    
    // Instance grid and SVG settings
    uint32_t _instanceGridCols;
    uint32_t _instanceGridRows;
    float _bezierMaxDeviationPx;
    NSString *_currentSVGPath;
    TriangulationMethod _currentTriangulationMethod;
    
    // Overdraw visualization
    id<MTLRenderPipelineState> _overdrawPipeline;
    
    // Overdraw measurement (GPU-based)
    id<MTLRenderPipelineState> _overdrawCountPipeline;
    id<MTLComputePipelineState> _sumOverdrawPipeline;
    id<MTLTexture> _overdrawCountTexture;
    id<MTLBuffer> _overdrawResultsBuffer;
    
    // Helper invocation measurement (GPU-based)
    id<MTLRenderPipelineState> _helperInvocationCountPipeline;
    id<MTLComputePipelineState> _sumHelperInvocationPipeline;
    id<MTLTexture> _helperInvocationDummyRT;
    id<MTLBuffer> _helperInvocationCountsBuffer; // uint32_t per pixel
    id<MTLBuffer> _helperInvocationResultsBuffer; // 2 uint32: total, unique
    uint32_t _helperInvocationCountLen; // width*height
    
    // Helper lane engagement (1x1 dummy texture for forcing derivative computation)
    id<MTLTexture> _helperLaneTexture;
    id<MTLSamplerState> _pointSampler;
    
    // ImGUI
    ImGUIHelper *_imguiHelper;
    
    // Cached metrics (computed on demand)
    uint64_t _lastOverdrawSum;
    double _lastOverdrawRatio;
    uint64_t _lastHelperSum;
    double _lastHelperRatio;
    TriangulationMetrics::MeshMetrics _lastMeshMetrics;
    BOOL _hasMeshMetrics;
}

@synthesize showGridOverlay = _showGridOverlay;

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
    _visualizationMode = VisualizationModeHelperLane;
    _tileSizePx = kDefaultTileSize;
    _instanceGridCols = 5;
    _instanceGridRows = 5;
    _bezierMaxDeviationPx = 1.0f;
    _currentSVGPath = nil;
    _currentTriangulationMethod = TriangulationMethodMinMaxArea;
    _lastOverdrawSum = 0;
    _lastOverdrawRatio = 0.0;
    _lastHelperSum = 0;
    _lastHelperRatio = 0.0;
    _hasMeshMetrics = NO;

    [self setupView];
    [self setupPipelines];
    [self setupGridOverlay];

    // Initialize ImGUI
    _imguiHelper = [[ImGUIHelper alloc] initWithDevice:_device view:mtkView];
    
//  Configure geometry: ellipse with 300 vertices, MWT triangulation, 3x3 grid
//    [self setupEllipseWithVertexCount:1000
//                        semiMajorAxis:1.0f
//                        semiMinorAxis:1.0f
//                  triangulationMethod:TriangulationMethodMinimumWeight
//                     instanceGridCols:100
//                             gridRows:100
//                         printMetrics:YES];
    NSString *defaultSVGPath = @"/Users/robi/Downloads/146024.svg";
    _currentSVGPath = defaultSVGPath;
    _currentTriangulationMethod = TriangulationMethodMinMaxArea;
    [self loadSVGFromPath:defaultSVGPath
      triangulationMethod:_currentTriangulationMethod
         instanceGridCols:_instanceGridCols
                 gridRows:_instanceGridRows
    bezierMaxDeviationPx:_bezierMaxDeviationPx];

    return self;
}

- (void)setupView {
    _view.clearColor = MTLClearColorMake(0, 0, 0, 1);
    _view.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
    _view.clearDepth = 1.0;
}

- (void)setupPipelines {
    NSError *error = nil;
    id<MTLLibrary> library = [_device newDefaultLibrary];
    
    _mainPipeline = MakeMainPipelineState(_device, _view, library, &error);
    NSAssert(_mainPipeline, @"Failed to create main pipeline: %@", error);
    
    _wireframePipeline = MakeWireframePipelineState(_device, _view, library, &error);
    NSAssert(_wireframePipeline, @"Failed to create wireframe pipeline: %@", error);
    
    _overdrawPipeline = MakeOverdrawPipelineState(_device, _view, library, &error);
    NSAssert(_overdrawPipeline, @"Failed to create overdraw pipeline: %@", error);
    
    _overdrawCountPipeline = MakeOverdrawCountPipelineState(_device, library, &error);
    NSAssert(_overdrawCountPipeline, @"Failed to create overdraw count pipeline: %@", error);
    
    id<MTLFunction> sumOverdrawFunc = [library newFunctionWithName:@"sumOverdrawTexture"];
    _sumOverdrawPipeline = [_device newComputePipelineStateWithFunction:sumOverdrawFunc error:&error];
    NSAssert(_sumOverdrawPipeline, @"Failed to create sumOverdraw pipeline: %@", error);
    
    _helperInvocationCountPipeline = MakeHelperInvocationCountPipelineState(_device, library, &error);
    NSAssert(_helperInvocationCountPipeline, @"Failed to create helper invocation count pipeline: %@", error);
    
    id<MTLFunction> sumHelperFunc = [library newFunctionWithName:@"sumHelperInvocationCounts"];
    _sumHelperInvocationPipeline = [_device newComputePipelineStateWithFunction:sumHelperFunc error:&error];
    NSAssert(_sumHelperInvocationPipeline, @"Failed to create sumHelperInvocation pipeline: %@", error);
    
    _gridOverlayPipeline = MakeGridOverlayPipelineState(_device, _view, library, &error);
    NSAssert(_gridOverlayPipeline, @"Failed to create grid overlay pipeline: %@", error);
    
    _depthState = MakeDepthState(_device);
    _noDepthState = MakeNoDepthState(_device);
    
    // Allocate overdraw results buffer (2 uints: total draws, unique pixels)
    _overdrawResultsBuffer = [_device newBufferWithLength:sizeof(uint32_t) * 2
                                                  options:MTLResourceStorageModeShared];
    
    // Allocate helper invocation results buffer (2 uints: total helpers, unique helper pixels)
    _helperInvocationResultsBuffer = [_device newBufferWithLength:sizeof(uint32_t) * 2
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
    
    // Verify vertex count (debug)
    NSLog(@"Ellipse: requested %d vertices, created %zu vertices, %zu triangles", 
          vertexCount, _currentVertices.size(), _currentIndices.size() / 3);
    
    [self uploadVertices:_currentVertices indices:_currentIndices];
    
    [self setupOrthographicProjection];
    [self setupInstanceGridWithCols:cols rows:rows];

    if (printMetrics) {
        simd_int2 framebufferSize = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
        TriangulationMetrics::computeAndPrintMeshMetrics(_currentVertices, _currentIndices, framebufferSize, {32, 32});
    }
}

- (BOOL)loadSVGFromPath:(NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows
    bezierMaxDeviationPx:(float)bezierMaxDeviationPx {
    
    // Create triangulator that uses the selected method
    SVGLoader::Triangulator triangulator = [self, method](const std::vector<Vertex>& verts, bool shouldHandleConcave, bool handleHoles, const std::vector<Vertex>& outerVertices, const std::vector<std::vector<Vertex>>& holes) -> std::vector<uint32_t> {
        std::vector<Vertex> mutableVerts = verts;
        std::vector<uint32_t> indices;
        
        switch (method) {
            case TriangulationMethodEarClipping:
                indices = Triangulation::EarClippingTriangulation(mutableVerts);
                break;
                
            case TriangulationMethodMinimumWeight:
                indices = Triangulation::MinimumWeightTriangulation(mutableVerts, shouldHandleConcave, handleHoles, outerVertices, holes);
                break;
                
            case TriangulationMethodCentroidFan:
                indices = Triangulation::CentroidFanTriangulation(mutableVerts);
                break;
                
            case TriangulationMethodGreedyMaxArea:
                indices = Triangulation::GreedyMaxAreaTriangulation(mutableVerts, shouldHandleConcave, handleHoles, outerVertices, holes);
                break;
                
            case TriangulationMethodStrip:
                indices = Triangulation::StripTriangulation(mutableVerts);
                break;
                
            case TriangulationMethodMaxMinArea:
                indices = Triangulation::MaxMinAreaTriangulation(mutableVerts, shouldHandleConcave, handleHoles, outerVertices, holes);
                break;
                
            case TriangulationMethodMinMaxArea:
                indices = Triangulation::MinMaxAreaTriangulation(mutableVerts, shouldHandleConcave, handleHoles, outerVertices, holes);
                break;
                
            case TriangulationMethodConstrainedDelaunay:
                indices = Triangulation::ConstrainedDelaunayTriangulation(mutableVerts);
                break;
        }
        
        return indices;
    };
    
    // Tessellate and triangulate each path with the chosen method
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    if (!SVGLoader::TessellateSvgToMesh(path.UTF8String, vertices, indices, triangulator, bezierMaxDeviationPx)) {
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
            indices = Triangulation::EarClippingTriangulation(vertices);
            break;
            
        case TriangulationMethodMinimumWeight:
            indices = Triangulation::MinimumWeightTriangulation(vertices, true);
            break;
            
        case TriangulationMethodCentroidFan:
            indices = Triangulation::CentroidFanTriangulation(vertices);
            break;
            
        case TriangulationMethodGreedyMaxArea:
            indices = Triangulation::GreedyMaxAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodStrip:
            indices = Triangulation::StripTriangulation(vertices);
            break;
            
        case TriangulationMethodMaxMinArea:
            indices = Triangulation::MaxMinAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodMinMaxArea:
            indices = Triangulation::MinMaxAreaTriangulation(vertices);
            break;
            
        case TriangulationMethodConstrainedDelaunay:
            return Triangulation::ConstrainedDelaunayTriangulation(vertices);
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
    const float edgePadding = 0.00f;
    const float instancePadding = 0.00f;

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

    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if (!renderPassDescriptor) {
        [commandBuffer commit];
        return;
    }
    
    // Start ImGUI frame
    [_imguiHelper newFrameWithRenderPassDescriptor:renderPassDescriptor];
    
    // Main UI: visualization controls + metrics
    ImGui::Begin("HelperLaneViz");
    
    // Visualization Mode
    {
        const char* vizModes[] = { "Helper Lane", "Wireframe", "Overdraw" };
        int currentMode = (int)_visualizationMode;
        if (ImGui::Combo("Visualization Mode", &currentMode, vizModes, 3)) {
            _visualizationMode = (VisualizationMode)currentMode;
        }
    }
    
    // Tile Grid
    {
        bool showGrid = _showGridOverlay;
        if (ImGui::Checkbox("Show Tile Grid", &showGrid)) {
            _showGridOverlay = showGrid ? YES : NO;
        }
    }
    
    // Tile Size (16x16 or 32x32 only)
    {
        int tileSizeIndex = (_tileSizePx == 16) ? 0 : 1;
        const char* tileSizes[] = { "16x16", "32x32" };
        if (ImGui::Combo("Tile Size", &tileSizeIndex, tileSizes, 2)) {
            _tileSizePx = (tileSizeIndex == 0) ? 16 : 32;
        }
    }
    
    // Instance Grid
    {
        int cols = (int)_instanceGridCols;
        int rows = (int)_instanceGridRows;
        bool gridChanged = false;
        if (ImGui::InputInt("Instance Cols", &cols, 1, 10)) {
            cols = MAX(1, cols);
            _instanceGridCols = (uint32_t)cols;
            gridChanged = true;
        }
        if (ImGui::InputInt("Instance Rows", &rows, 1, 10)) {
            rows = MAX(1, rows);
            _instanceGridRows = (uint32_t)rows;
            gridChanged = true;
        }
        if (gridChanged && _currentSVGPath) {
            [self loadSVGFromPath:_currentSVGPath
              triangulationMethod:_currentTriangulationMethod
                 instanceGridCols:_instanceGridCols
                         gridRows:_instanceGridRows
            bezierMaxDeviationPx:_bezierMaxDeviationPx];
        }
    }
    
    // Max Bezier Deviation
    {
        float bezierDev = _bezierMaxDeviationPx;
        if (ImGui::SliderFloat("Max Bezier Deviation (px)", &bezierDev, 0.1f, 50.0f, "%.1f")) {
            _bezierMaxDeviationPx = bezierDev;
            if (_currentSVGPath) {
                [self loadSVGFromPath:_currentSVGPath
                  triangulationMethod:_currentTriangulationMethod
                     instanceGridCols:_instanceGridCols
                             gridRows:_instanceGridRows
                bezierMaxDeviationPx:_bezierMaxDeviationPx];
            }
        }
    }
    
    ImGui::Separator();
    
    if (ImGui::Button("Compute Overdraw Summary")) {
        [self computeOverdrawMetricsWithOverdrawSum:&_lastOverdrawSum overdrawRatio:&_lastOverdrawRatio];
    }
    ImGui::SameLine();
    ImGui::Text("Total=%llu  Ratio=%.3f", _lastOverdrawSum, _lastOverdrawRatio);
    
    if (ImGui::Button("Compute Helper Invocation Summary")) {
        [self computeHelperInvocationMetricsWithHelperSum:&_lastHelperSum helperRatio:&_lastHelperRatio];
    }
    ImGui::SameLine();
    ImGui::Text("Total=%llu  Ratio=%.3f", _lastHelperSum, _lastHelperRatio);
    
    if (ImGui::Button("Compute Tile Stats (CPU)")) {
        if (!_currentVertices.empty() && !_currentIndices.empty()) {
            simd_int2 fb = {(int)view.drawableSize.width, (int)view.drawableSize.height};
            simd_int2 tile = {(int)_tileSizePx, (int)_tileSizePx};
            _lastMeshMetrics = TriangulationMetrics::computeMeshMetrics(_currentVertices, _currentIndices, fb, tile);
            _hasMeshMetrics = YES;
        } else {
            _hasMeshMetrics = NO;
        }
    }
    if (_hasMeshMetrics) {
        ImGui::Text("Tris=%zu  TotalTileOverlaps=%.0f  BCI=%.3f",
                    _lastMeshMetrics.triangleCount,
                    _lastMeshMetrics.totalTileOverlaps,
                    _lastMeshMetrics.binningCostIndex);
        ImGui::Text("Tris/Tile mean=%.2f med=%.2f p95=%.2f",
                    _lastMeshMetrics.trianglesPerTile_Mean,
                    _lastMeshMetrics.trianglesPerTile_Median,
                    _lastMeshMetrics.trianglesPerTile_P95);
        ImGui::Text("Tiles/Tri mean=%.2f med=%.2f p95=%.2f",
                    _lastMeshMetrics.tilesPerTriangle_Mean,
                    _lastMeshMetrics.tilesPerTriangle_Median,
                    _lastMeshMetrics.tilesPerTriangle_P95);
    }
    ImGui::End();
    
    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
    
    // Choose rendering mode based on visualization mode
    switch (_visualizationMode) {
        case VisualizationModeOverdraw:
            [self encodeOverdrawGeometryWithEncoder:encoder];
            break;
        case VisualizationModeWireframe:
            [self encodeWireframeGeometryWithEncoder:encoder];
            break;
        case VisualizationModeHelperLane:
        default:
            [self encodeMainGeometryWithEncoder:encoder];
            break;
    }

    // Draw grid overlay if enabled (not shown during overdraw mode)
    if (_showGridOverlay && _visualizationMode != VisualizationModeOverdraw) {
        [self encodeGridOverlayWithEncoder:encoder drawableSize:view.drawableSize];
    }
    
    // Render ImGUI (on top of everything)
    [_imguiHelper renderWithCommandBuffer:commandBuffer commandEncoder:encoder];

    [encoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
    
    [commandBuffer commit];
}

- (void)encodeMainGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder {
    [encoder setRenderPipelineState:_mainPipeline];
    [encoder setDepthStencilState:_depthState];
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeFill]; // Fill for helper lane visualization

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

- (void)encodeWireframeGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder {
    [encoder setRenderPipelineState:_wireframePipeline];
    [encoder setDepthStencilState:_depthState];
    [encoder setCullMode:MTLCullModeNone];
    [encoder setTriangleFillMode:MTLTriangleFillModeLines]; // Lines for wireframe

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
                         tileSize:_tileSizePx
                     drawableSize:drawableSize];
}

// MARK: - Helper Invocation Measurement (GPU-based)

- (void)reallocateHelperInvocationResourcesIfNeeded {
    const uint32_t width = (uint32_t)_view.drawableSize.width;
    const uint32_t height = (uint32_t)_view.drawableSize.height;
    if (width == 0 || height == 0) return;
    
    const uint32_t len = width * height;
    if (_helperInvocationCountsBuffer && _helperInvocationCountLen == len && _helperInvocationDummyRT &&
        _helperInvocationDummyRT.width == width && _helperInvocationDummyRT.height == height) {
        return;
    }
    
    _helperInvocationCountLen = len;
    _helperInvocationCountsBuffer = [_device newBufferWithLength:sizeof(uint32_t) * (NSUInteger)len
                                                        options:MTLResourceStorageModeShared];
    
    MTLTextureDescriptor *desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm
                                                                                   width:width
                                                                                  height:height
                                                                               mipmapped:NO];
    desc.usage = MTLTextureUsageRenderTarget;
    desc.storageMode = MTLStorageModePrivate;
    _helperInvocationDummyRT = [_device newTextureWithDescriptor:desc];
}

- (void)computeHelperInvocationMetricsWithHelperSum:(uint64_t*)outSum helperRatio:(double*)outRatio {
    if (!_vertexBuffer || !_indexBuffer) {
        if (outSum) *outSum = 0;
        if (outRatio) *outRatio = 0.0;
        return;
    }
    
    [self reallocateHelperInvocationResourcesIfNeeded];
    if (!_helperInvocationCountsBuffer || !_helperInvocationDummyRT || _helperInvocationCountLen == 0) {
        if (outSum) *outSum = 0;
        if (outRatio) *outRatio = 0.0;
        return;
    }
    
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Clear per-pixel helper count buffer on GPU
    id<MTLBlitCommandEncoder> blit = [commandBuffer blitCommandEncoder];
    [blit fillBuffer:_helperInvocationCountsBuffer range:NSMakeRange(0, _helperInvocationCountsBuffer.length) value:0];
    [blit endEncoding];
    
    // Clear results buffer on CPU (shared)
    uint32_t* resultsPtr = (uint32_t*)_helperInvocationResultsBuffer.contents;
    resultsPtr[0] = 0; // total helpers
    resultsPtr[1] = 0; // unique helper pixels
    
    // Render pass to run fragment stage (counts are written into helperCounts buffer)
    MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor new];
    rp.colorAttachments[0].texture = _helperInvocationDummyRT;
    rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
    rp.colorAttachments[0].storeAction = MTLStoreActionDontCare;
    
    id<MTLRenderCommandEncoder> re = [commandBuffer renderCommandEncoderWithDescriptor:rp];
    [re setRenderPipelineState:_helperInvocationCountPipeline];
    [re setCullMode:MTLCullModeNone];
    [re setTriangleFillMode:MTLTriangleFillModeFill];
    
    FrameConstants frameConstants = {
        .viewProjectionMatrix = _viewProjectionMatrix,
        .viewPortSize = _viewportSize
    };
    [re setVertexBuffer:_vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [re setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [re setVertexBytes:&_gridParams length:sizeof(_gridParams) atIndex:VertexInputGridParams];
    
    const uint32_t fb[2] = {(uint32_t)_view.drawableSize.width, (uint32_t)_view.drawableSize.height};
    [re setFragmentBuffer:_helperInvocationCountsBuffer offset:0 atIndex:0];
    [re setFragmentBytes:&fb length:sizeof(fb) atIndex:1];
    [re setFragmentTexture:_helperLaneTexture atIndex:0];
    [re setFragmentSamplerState:_pointSampler atIndex:0];
    
    const NSUInteger indexCount = _indexBuffer.length / sizeof(uint32_t);
    const NSUInteger instanceCount = _gridParams.cols * _gridParams.rows;
    [re drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                   indexCount:indexCount
                    indexType:MTLIndexTypeUInt32
                  indexBuffer:_indexBuffer
            indexBufferOffset:0
                instanceCount:instanceCount];
    [re endEncoding];
    
    // Sum helper counts (compute)
    id<MTLComputeCommandEncoder> ce = [commandBuffer computeCommandEncoder];
    [ce setComputePipelineState:_sumHelperInvocationPipeline];
    [ce setBuffer:_helperInvocationCountsBuffer offset:0 atIndex:0];
    [ce setBuffer:_helperInvocationResultsBuffer offset:0 atIndex:1];
    uint32_t len = _helperInvocationCountLen;
    [ce setBytes:&len length:sizeof(len) atIndex:2];
    
    const NSUInteger threadsPerTG = 256;
    [ce dispatchThreads:MTLSizeMake(len, 1, 1)
  threadsPerThreadgroup:MTLSizeMake(threadsPerTG, 1, 1)];
    [ce endEncoding];
    
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    
    const uint64_t totalHelpers = resultsPtr[0];
    const uint64_t uniquePixels = resultsPtr[1];
    
    if (outSum) *outSum = totalHelpers;
    if (outRatio) *outRatio = (uniquePixels > 0) ? ((double)totalHelpers / uniquePixels) : 0.0;
}

@end
