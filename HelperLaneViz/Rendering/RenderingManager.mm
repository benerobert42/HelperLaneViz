//
//  RenderingManager.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import "RenderingManager.h"
#import "GeometryManager.h"
#import "MetricsComputer.h"
#import "PipelineFactory.h"
#import "GridOverlay.h"
#import "TriangulationMetrics.h"
#import "../ThirdParty/ImGUI/ImGUIHelper.h"

#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>

#include "../../external/imgui/imgui.h"

@implementation RenderingManager {
    id<MTLDevice> _device;
    MTKView *_view;
    
    // Render pipelines
    id<MTLRenderPipelineState> _mainPipeline;
    id<MTLRenderPipelineState> _wireframePipeline;
    id<MTLRenderPipelineState> _overdrawPipeline;
    id<MTLRenderPipelineState> _gridOverlayPipeline;
    
    // Grid overlay
    GridOverlay *_gridOverlay;
    
    // Visualization state
    VisualizationMode _visualizationMode;
    uint32_t _tileSizePx;
    BOOL _showGridOverlay;
    
    // Helper lane engagement
    id<MTLTexture> _helperLaneTexture;
    id<MTLSamplerState> _pointSampler;
    
    // ImGUI
    ImGUIHelper *_imguiHelper;
    
    // Cached metrics for UI display
    uint64_t _lastOverdrawSum;
    double _lastOverdrawRatio;
    uint64_t _lastHelperSum;
    double _lastHelperRatio;
    TriangulationMetrics::MeshMetrics _lastMeshMetrics;
    BOOL _hasMeshMetrics;
    
    // UI state for geometry reload
    NSString *_currentSVGPath;
    TriangulationMethod _currentTriangulationMethod;
    float _bezierMaxDeviationPx;
    uint32_t _instanceGridCols;
    uint32_t _instanceGridRows;
}

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view {
    if (!(self = [super init])) return nil;
    
    _device = device;
    _view = view;
    
    _visualizationMode = VisualizationModeHelperLane;
    _tileSizePx = 32;
    _showGridOverlay = YES;
    _lastOverdrawSum = 0;
    _lastOverdrawRatio = 0.0;
    _lastHelperSum = 0;
    _lastHelperRatio = 0.0;
    _hasMeshMetrics = NO;
    _bezierMaxDeviationPx = 1.0f;
    _instanceGridCols = 5;
    _instanceGridRows = 5;
    
    [self setupPipelines];
    [self setupGridOverlay];
    [self setupHelperLaneResources];
    
    // Initialize ImGUI
    _imguiHelper = [[ImGUIHelper alloc] initWithDevice:_device view:view];
    
    return self;
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
    
    _gridOverlayPipeline = MakeGridOverlayPipelineState(_device, _view, library, &error);
    NSAssert(_gridOverlayPipeline, @"Failed to create grid overlay pipeline: %@", error);
}

- (void)setupGridOverlay {
    _gridOverlay = [[GridOverlay alloc] initWithPipelineState:_gridOverlayPipeline];
}

- (void)setupHelperLaneResources {
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

// MARK: - Public Interface

- (void)renderUIWithGeometry:(GeometryManager *)geometry
                      metrics:(MetricsComputer *)metrics
              onGeometryReload:(void(^)(NSString *path, TriangulationMethod method, uint32_t cols, uint32_t rows, float bezierDev))reloadBlock {
    
    MTLRenderPassDescriptor *renderPassDescriptor = _view.currentRenderPassDescriptor;
    if (!renderPassDescriptor) return;
    
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
    
    ImGui::Separator();
    ImGui::Text("Geometry Settings");
    
    // Triangulation Method
    {
        const char* triMethods[] = {
            "Ear Clipping",
            "Minimum Weight",
            "Centroid Fan",
            "Greedy Max Area",
            "Strip",
            "Max-Min Area",
            "Min-Max Area",
            "Constrained Delaunay"
        };
        int currentMethod = (int)_currentTriangulationMethod;
        if (ImGui::Combo("Triangulation Method", &currentMethod, triMethods, 8)) {
            _currentTriangulationMethod = (TriangulationMethod)currentMethod;
            if (_currentSVGPath) {
                reloadBlock(_currentSVGPath, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
            }
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
            reloadBlock(_currentSVGPath, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
        }
    }
    
    // Max Bezier Deviation
    {
        float bezierDev = _bezierMaxDeviationPx;
        if (ImGui::SliderFloat("Max Bezier Deviation (px)", &bezierDev, 0.1f, 50.0f, "%.1f")) {
            _bezierMaxDeviationPx = bezierDev;
            if (_currentSVGPath) {
                reloadBlock(_currentSVGPath, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
            }
        }
    }
    
    ImGui::Separator();
    
    if (ImGui::Button("Compute Overdraw Summary")) {
        [metrics computeOverdrawMetricsWithGeometry:geometry overdrawSum:&_lastOverdrawSum overdrawRatio:&_lastOverdrawRatio];
    }
    ImGui::SameLine();
    ImGui::Text("Total=%llu  Ratio=%.3f", _lastOverdrawSum, _lastOverdrawRatio);
    
    if (ImGui::Button("Compute Helper Invocation Summary")) {
        [metrics computeHelperInvocationMetricsWithGeometry:geometry helperSum:&_lastHelperSum helperRatio:&_lastHelperRatio];
    }
    ImGui::SameLine();
    ImGui::Text("Total=%llu  Ratio=%.3f", _lastHelperSum, _lastHelperRatio);
    
    if (ImGui::Button("Compute Tile Stats (CPU)")) {
        const auto& baseVertices = geometry.currentVertices;
        const auto& baseIndices = geometry.currentIndices;
        if (!baseVertices.empty() && !baseIndices.empty()) {
            // Expand vertices for all instances using grid params
            GridParams gp = geometry.gridParams;
            const uint32_t cols = gp.cols;
            const uint32_t rows = gp.rows;
            const uint32_t instanceCount = cols * rows;
            
            std::vector<Vertex> expandedVertices;
            std::vector<uint32_t> expandedIndices;
            expandedVertices.reserve(baseVertices.size() * instanceCount);
            expandedIndices.reserve(baseIndices.size() * instanceCount);
            
            for (uint32_t inst = 0; inst < instanceCount; ++inst) {
                const uint32_t col = inst % cols;
                const uint32_t row = inst / cols;
                const simd_float2 cellOrigin = simd_make_float2(col, row) * gp.cellSize + gp.origin;
                const uint32_t vertexOffset = (uint32_t)expandedVertices.size();
                
                // Transform and add vertices for this instance
                for (const auto& v : baseVertices) {
                    Vertex tv;
                    simd_float2 local = simd_make_float2(v.position.x, v.position.y) * gp.scale;
                    tv.position = simd_make_float3(cellOrigin.x + local.x, cellOrigin.y + local.y, v.position.z);
                    expandedVertices.push_back(tv);
                }
                
                // Add indices for this instance (offset by vertex count)
                for (uint32_t idx : baseIndices) {
                    expandedIndices.push_back(vertexOffset + idx);
                }
            }
            
            simd_int2 fb = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
            simd_int2 tile = {(int)_tileSizePx, (int)_tileSizePx};
            _lastMeshMetrics = TriangulationMetrics::ComputeMeshMetrics(expandedVertices, expandedIndices, fb, tile);
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
}

- (void)renderImGUIWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                        commandEncoder:(id<MTLRenderCommandEncoder>)commandEncoder {
    [_imguiHelper renderWithCommandBuffer:commandBuffer commandEncoder:commandEncoder];
}

- (void)encodeGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                          geometry:(GeometryManager *)geometry
                              mode:(VisualizationMode)mode {
    
    // Common setup for all modes
    FrameConstants frameConstants = {
        .viewProjectionMatrix = geometry.viewProjectionMatrix,
        .viewPortSize = geometry.viewportSize
    };
    GridParams gridParams = geometry.gridParams;
    
    [encoder setVertexBuffer:geometry.vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [encoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [encoder setVertexBytes:&gridParams length:sizeof(gridParams) atIndex:VertexInputGridParams];
    
    const NSUInteger indexCount = geometry.indexCount;
    const NSUInteger instanceCount = geometry.instanceCount;
    
    // Common render state (no depth, no culling for 2D)
    [encoder setCullMode:MTLCullModeNone];
    
    // Mode-specific setup
    switch (mode) {
        case VisualizationModeHelperLane:
            [encoder setRenderPipelineState:_mainPipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeFill];
            [encoder setFragmentTexture:_helperLaneTexture atIndex:0];
            [encoder setFragmentSamplerState:_pointSampler atIndex:0];
            break;
            
        case VisualizationModeWireframe:
            [encoder setRenderPipelineState:_wireframePipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeLines];
            break;
            
        case VisualizationModeOverdraw:
            [encoder setRenderPipelineState:_overdrawPipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeFill];
            break;
    }
    
    [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:indexCount
                         indexType:MTLIndexTypeUInt32
                       indexBuffer:geometry.indexBuffer
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

- (BOOL)showGridOverlay {
    return _showGridOverlay;
}

- (void)setShowGridOverlay:(BOOL)show {
    _showGridOverlay = show;
}

- (VisualizationMode)visualizationMode {
    return _visualizationMode;
}

- (void)setVisualizationMode:(VisualizationMode)mode {
    _visualizationMode = mode;
}

- (uint32_t)tileSizePx {
    return _tileSizePx;
}

- (void)setTileSizePx:(uint32_t)tileSize {
    _tileSizePx = tileSize;
}

- (void)setCurrentSVGPath:(NSString *)path
                    method:(TriangulationMethod)method
                      cols:(uint32_t)cols
                      rows:(uint32_t)rows
          bezierDeviation:(float)bezierDev {
    _currentSVGPath = path;
    _currentTriangulationMethod = method;
    _instanceGridCols = cols;
    _instanceGridRows = rows;
    _bezierMaxDeviationPx = bezierDev;
}

@end

