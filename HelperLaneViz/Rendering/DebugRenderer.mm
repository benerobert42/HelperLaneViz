//
//  DebugRenderer.mm
//  HelperLaneViz
//
//  Created on 2025. 01. 27..
//

#import "DebugRenderer.h"
#import "GeometryManager.h"
#import "MetricsComputer.h"
#import "TriangulationMetrics.h"
#import "Measurements/GPUFrameTimer.h"
#include "../Geometry/Triangulation.h"

#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <AppKit/AppKit.h>
#include <mach/mach_time.h>

// ImGUI includes - use directly as external library (following best practices)
#include "../../external/imgui/imgui.h"
#include "../../external/imgui/backends/imgui_impl_metal.h"
#include "../../external/imgui/backends/imgui_impl_osx.h"

// Low-overhead timing utility using mach_absolute_time (nanosecond precision, minimal overhead)
static inline double machTimeToMs(uint64_t start, uint64_t end) {
    static mach_timebase_info_data_t timebase = {0};
    if (timebase.denom == 0) {
        mach_timebase_info(&timebase);
    }
    uint64_t elapsed = end - start;
    return (double)elapsed * (double)timebase.numer / (double)timebase.denom / 1e6; // Convert to ms
}

@implementation DebugRenderer {
    id<MTLDevice> _device;
    MTKView *_view;
    
    // Visualization state
    VisualizationMode _visualizationMode;
    uint32_t _tileSizePx;
    BOOL _showGridOverlay;
    int _msaaSampleCount;  // 1, 2, or 4
    int _pendingMSAASampleCount;  // 0 = no pending change
    
    // GPU frame timer
    GPUFrameTimer *_gpuFrameTimer;
    
    // Cached metrics for UI display
    uint64_t _lastOverdrawSum;
    double _lastOverdrawRatio;
    uint64_t _lastHelperSum;
    double _lastHelperRatio;
    TriangulationMetrics::MeshMetrics _lastMeshMetrics;
    BOOL _hasMeshMetrics;
    
    // GPU frametime results
    GPUFrameTimer::Results _lastGPUFrameResults;
    BOOL _hasGPUFrameResults;
    
    // UI state for geometry reload
    NSString *_currentSVGPath;
    TriangulationMethod _currentTriangulationMethod;
    float _bezierMaxDeviationPx;
    uint32_t _instanceGridCols;
    uint32_t _instanceGridRows;
    
    // Shape type: 0 = SVG, 1 = Ellipse
    int _shapeType;
    float _ellipseAxisRatio;
    int _ellipseVertexCount;
    
    // Benchmark state
    BOOL _benchmarkRunning;
    int _benchmarkMethodIndex;
    int _benchmarkPhase;  // 0=reload, 1=compute metrics, 2=delay, 3=record frametime, 4=wait for frametime
    int _benchmarkDelayFrames;
    struct BenchmarkResult {
        uint64_t helperSum;
        double totalEdgeLength;
        double helperRatio;
        double trisPerTileMean, trisPerTileMed, trisPerTileP95;
        double tilesPerTriMean, tilesPerTriMed, tilesPerTriP95;
        double frametimeMean, frametimeMed, frametimeDev;
        size_t triCount;
    };
    BenchmarkResult _benchmarkResults[11];  // One per triangulation method
    
    // Display size tracking
    CGSize _displaySize;
}

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view {
    if (!(self = [super init])) return nil;
    
    _device = device;
    _view = view;
    
    _visualizationMode = VisualizationModeHelperLane;
    _tileSizePx = 32;
    _showGridOverlay = YES;
    _msaaSampleCount = (int)_view.sampleCount;  // Read initial value from view
    _pendingMSAASampleCount = 0;
    _lastOverdrawSum = 0;
    _lastOverdrawRatio = 0.0;
    _lastHelperSum = 0;
    _lastHelperRatio = 0.0;
    _hasMeshMetrics = NO;
    _hasGPUFrameResults = NO;
    _bezierMaxDeviationPx = 1.0f;
    _instanceGridCols = 5;
    _instanceGridRows = 5;
    _shapeType = 0;
    _ellipseAxisRatio = 1.0f;
    _ellipseVertexCount = 64;
    _benchmarkRunning = NO;
    _displaySize = CGSizeZero;
    
    // Setup Dear ImGui context - following official example pattern
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    
    // Setup Platform/Renderer backends - direct ImGUI API usage
    ImGui_ImplMetal_Init(_device);
    ImGui_ImplOSX_Init(_view);
    
    // Initialize GPU frame timer
    _gpuFrameTimer = new GPUFrameTimer();
    
    return self;
}

- (void)updateDisplaySize:(CGSize)size {
    _displaySize = size;
}

- (BOOL)newFrameWithRenderPassDescriptor:(MTLRenderPassDescriptor *)renderPassDescriptor {
    // Apply pending MSAA change at frame start - skip this frame to let drawable update
    if (_pendingMSAASampleCount > 0) {
        _msaaSampleCount = _pendingMSAASampleCount;
        _view.sampleCount = _msaaSampleCount;
        _pendingMSAASampleCount = 0;
        return NO;  // Skip rendering - current drawable has old sample count
    }
    
    if (!renderPassDescriptor) {
        return YES;
    }
    
    // Update ImGUI display size and framebuffer scale - following official example pattern
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize.x = _displaySize.width;
    io.DisplaySize.y = _displaySize.height;
    
    CGFloat framebufferScale = _view.window.screen.backingScaleFactor ?: NSScreen.mainScreen.backingScaleFactor;
    io.DisplayFramebufferScale = ImVec2(framebufferScale, framebufferScale);
    
    // Start the Dear ImGui frame - direct backend API calls
    ImGui_ImplMetal_NewFrame(renderPassDescriptor);
    ImGui_ImplOSX_NewFrame(_view);
    ImGui::NewFrame();
    
    return YES;
}

- (void)renderUIWithGeometry:(GeometryManager *)geometry
                      metrics:(MetricsComputer *)metrics
              onGeometryReload:(void(^)(NSString *path, TriangulationMethod method, uint32_t cols, uint32_t rows, float bezierDev))reloadBlock
               onEllipseReload:(void(^)(float axisRatio, int vertexCount, TriangulationMethod method, uint32_t cols, uint32_t rows))ellipseBlock {
    
    // Main UI: visualization controls + metrics
    ImGui::Begin("HelperLaneViz");
    
    // Visualization Mode
    {
        const char* vizModes[] = { "Helper Lane", "Wireframe", "Overdraw", "Print Friendly" };
        int currentMode = (int)_visualizationMode;
        if (ImGui::Combo("Visualization Mode", &currentMode, vizModes, 4)) {
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
    
    // MSAA
    {
        int msaaIndex = (_msaaSampleCount == 1) ? 0 : ((_msaaSampleCount == 2) ? 1 : 2);
        const char* msaaOptions[] = { "Off", "2x", "4x" };
        if (ImGui::Combo("MSAA", &msaaIndex, msaaOptions, 3)) {
            int newSampleCount = (msaaIndex == 0) ? 1 : ((msaaIndex == 1) ? 2 : 4);
            if (newSampleCount != _msaaSampleCount) {
                _pendingMSAASampleCount = newSampleCount;  // Apply next frame
            }
        }
    }
    
    // Note: When MSAA changes, newFrameWithRenderPassDescriptor will skip the frame
    // and RenderingManager will update pipelines on the next frame via updatePipelinesForCurrentSampleCount
    
    ImGui::Separator();
    ImGui::Text("Geometry Settings");
    
    // Shape Type
    {
        const char* shapeTypes[] = { "SVG File", "Ellipse" };
        if (ImGui::Combo("Shape Type", &_shapeType, shapeTypes, 2)) {
            if (_shapeType == 0 && _currentSVGPath) {
                reloadBlock(_currentSVGPath, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
            } else if (_shapeType == 1) {
                ellipseBlock(_ellipseAxisRatio, _ellipseVertexCount, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows);
            }
        }
    }
    
    // Ellipse settings (only shown when Ellipse is selected)
    if (_shapeType == 1) {
        if (ImGui::SliderFloat("Axis Ratio (minor/major)", &_ellipseAxisRatio, 0.1f, 1.0f, "%.2f")) {
            ellipseBlock(_ellipseAxisRatio, _ellipseVertexCount, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows);
        }
        if (ImGui::InputInt("Vertex Count", &_ellipseVertexCount, 1, 10)) {
            _ellipseVertexCount = MAX(3, _ellipseVertexCount);
            ellipseBlock(_ellipseAxisRatio, _ellipseVertexCount, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows);
        }
    }
    
    // Triangulation Method
    {
        const char* triMethods[] = {
            "Ear Clipping",
            "Ear Clipping (Triangulator)",
            "Ear Clipping (Triangulator Flipped)",
            "Centroid Fan",
            "Strip",
            "Greedy Max Area",
            "Minimum Weight",
            "Max-Min Area",
            "Min-Max Area",
            "Constrained Delaunay",
            "Constrained Delaunay (Flipped)"
        };
        int currentMethod = (int)_currentTriangulationMethod;
        if (ImGui::Combo("Triangulation Method", &currentMethod, triMethods, 11)) {
            _currentTriangulationMethod = (TriangulationMethod)currentMethod;
            if (_shapeType == 0 && _currentSVGPath) {
                reloadBlock(_currentSVGPath, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
            } else if (_shapeType == 1) {
                ellipseBlock(_ellipseAxisRatio, _ellipseVertexCount, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows);
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
        if (gridChanged) {
            if (_shapeType == 0 && _currentSVGPath) {
                reloadBlock(_currentSVGPath, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
            } else if (_shapeType == 1) {
                ellipseBlock(_ellipseAxisRatio, _ellipseVertexCount, _currentTriangulationMethod, _instanceGridCols, _instanceGridRows);
            }
        }
    }
    
    // Max Bezier Deviation (only shown for SVG)
    if (_shapeType == 0) {
        float bezierDev = _bezierMaxDeviationPx;
        if (ImGui::SliderFloat("Max Bezier Deviation (px)", &bezierDev, 0.05f, 5.0f, "%.2f")) {
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
    
    if (ImGui::Button("Record GPU Frametimes (100 frames)")) {
        _hasGPUFrameResults = NO;
        _gpuFrameTimer->startMeasurement(100, [self](const GPUFrameTimer::Results& results) {
            self->_lastGPUFrameResults = results;
            self->_hasGPUFrameResults = YES;
        });
    }
    if (_gpuFrameTimer->isActive()) {
        ImGui::SameLine();
        ImGui::Text("Recording... %d frames remaining", _gpuFrameTimer->framesRemaining());
    }
    if (_hasGPUFrameResults) {
        ImGui::Text("Mean=%.3f ms  Median=%.3f ms  StdDev=%.3f ms",
                    _lastGPUFrameResults.avgMs,
                    _lastGPUFrameResults.p50Ms,
                    _lastGPUFrameResults.stdDevMs);
    }
    
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
        ImGui::Text("Edges=%f",
                    _lastMeshMetrics.totalEdgeLength);
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
    
    ImGui::Separator();
    if (ImGui::Button("Print Summary for Table")) {
        printf("TriCount\tTotalEdgeLength\tHelperSum\tHelperRatio\tTris/Tile Mean\tTris/Tile Med\tTris/Tile P95\tTiles/Tri Mean\tTiles/Tri Med\tTiles/Tri P95\tFrametime Mean\tFrametime Med\tFrametime Dev\n");
        printf("%zu\t%.2f\t%llu\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%.3f\t%.3f\n",
               _hasMeshMetrics ? _lastMeshMetrics.triangleCount : 0,
               _hasMeshMetrics ? _lastMeshMetrics.totalEdgeLength : 0,
               _lastHelperSum,
               _lastHelperRatio,
               _hasMeshMetrics ? _lastMeshMetrics.trianglesPerTile_Mean : 0.0,
               _hasMeshMetrics ? _lastMeshMetrics.trianglesPerTile_Median : 0.0,
               _hasMeshMetrics ? _lastMeshMetrics.trianglesPerTile_P95 : 0.0,
               _hasMeshMetrics ? _lastMeshMetrics.tilesPerTriangle_Mean : 0.0,
               _hasMeshMetrics ? _lastMeshMetrics.tilesPerTriangle_Median : 0.0,
               _hasMeshMetrics ? _lastMeshMetrics.tilesPerTriangle_P95 : 0.0,
               _hasGPUFrameResults ? _lastGPUFrameResults.avgMs : 0.0,
               _hasGPUFrameResults ? _lastGPUFrameResults.p50Ms : 0.0,
               _hasGPUFrameResults ? _lastGPUFrameResults.stdDevMs : 0.0);
    }
    
    // Benchmark buttons and state
    if (!_benchmarkRunning) {
        if (ImGui::Button("Benchmark All Methods")) {
            _benchmarkRunning = YES;
            _benchmarkMethodIndex = 0;
            _benchmarkPhase = 0;
            memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
        }
    } else {
        // Display 1-based index, accounting for skipped CentroidFan (index 3)
        int displayIndex = _benchmarkMethodIndex < 3 ? _benchmarkMethodIndex + 1 : _benchmarkMethodIndex;
        ImGui::Text("Benchmarking method %d/10, phase %d...", displayIndex, _benchmarkPhase);
    }
    ImGui::End();
    
    // Benchmark state machine
    if (_benchmarkRunning) {
        [self runBenchmarkStepWithGeometry:geometry metrics:metrics reloadBlock:reloadBlock ellipseBlock:ellipseBlock];
    }
}

- (void)renderDrawDataWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                          commandEncoder:(id<MTLRenderCommandEncoder>)commandEncoder {
    // Rendering - following official example pattern
    ImGui::Render();
    ImDrawData* drawData = ImGui::GetDrawData();
    
    if (drawData) {
        [commandEncoder pushDebugGroup:@"Dear ImGui rendering"];
        ImGui_ImplMetal_RenderDrawData(drawData, commandBuffer, commandEncoder);
        [commandEncoder popDebugGroup];
    }
}

- (void)shutdown {
    // Cleanup ImGUI - following official example pattern
    ImGui_ImplMetal_Shutdown();
    ImGui_ImplOSX_Shutdown();
    ImGui::DestroyContext();
    
    if (_gpuFrameTimer) {
        delete _gpuFrameTimer;
        _gpuFrameTimer = nullptr;
    }
}

- (void)dealloc {
    [self shutdown];
}

// MARK: - Accessors

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

- (int)msaaSampleCount {
    return _msaaSampleCount;
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

- (void*)gpuFrameTimer {
    return _gpuFrameTimer;
}

// MARK: - Benchmark Implementation

- (void)runBenchmarkStepWithGeometry:(GeometryManager *)geometry
                             metrics:(MetricsComputer *)metrics
                         reloadBlock:(void(^)(NSString *, TriangulationMethod, uint32_t, uint32_t, float))reloadBlock
                        ellipseBlock:(void(^)(float, int, TriangulationMethod, uint32_t, uint32_t))ellipseBlock {
    
    const char* methodNames[] =
        {"EarClipping", "EarClippingTriangulator", "EarClippingTriangulatorFlipped", "CentroidFan", "Strip", "GreedyMaxArea", "MinWeight", "MaxMinArea", "MinMaxArea", "ConstrainedDelaunay", "ConstrainedDelaunayFlipped"};

    // Skip CentroidFan it modifies vertices which breaks metrics
    if (_benchmarkMethodIndex == 3) {
        _benchmarkMethodIndex = 4;
    }
    
    switch (_benchmarkPhase) {
        case 0: // Reload geometry with current method
            if (_shapeType == 0 && _currentSVGPath) {
                reloadBlock(_currentSVGPath,
                            (TriangulationMethod)_benchmarkMethodIndex,
                            _instanceGridCols,
                            _instanceGridRows,
                            _bezierMaxDeviationPx);
            } else if (_shapeType == 1) {
                ellipseBlock(_ellipseAxisRatio,
                             _ellipseVertexCount,
                             (TriangulationMethod)_benchmarkMethodIndex,
                             _instanceGridCols,
                             _instanceGridRows);
            }
            _benchmarkPhase = 1;
            break;
            
        case 1: // Compute helper invocation and tile stats
        {
            [metrics computeHelperInvocationMetricsWithGeometry:geometry helperSum:&_lastHelperSum helperRatio:&_lastHelperRatio];
            
            const auto& baseVertices = geometry.currentVertices;
            const auto& baseIndices = geometry.currentIndices;
            if (!baseVertices.empty() && !baseIndices.empty()) {
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
                    for (const auto& v : baseVertices) {
                        Vertex tv;
                        simd_float2 local = simd_make_float2(v.position.x, v.position.y) * gp.scale;
                        tv.position = simd_make_float3(cellOrigin.x + local.x, cellOrigin.y + local.y, v.position.z);
                        expandedVertices.push_back(tv);
                    }
                    for (uint32_t idx : baseIndices) {
                        expandedIndices.push_back(vertexOffset + idx);
                    }
                }
                
                simd_int2 fb = {(int)_view.drawableSize.width, (int)_view.drawableSize.height};
                simd_int2 tile = {(int)_tileSizePx, (int)_tileSizePx};
                _lastMeshMetrics = TriangulationMetrics::ComputeMeshMetrics(expandedVertices, expandedIndices, fb, tile);
                _hasMeshMetrics = YES;
            }
            _benchmarkDelayFrames = 50;
            _benchmarkPhase = 2;
        }
            break;
            
        case 2: // Delay frames to let GPU settle after geometry change
            if (--_benchmarkDelayFrames <= 0) {
                _benchmarkPhase = 3;
            }
            break;
            
        case 3: // Start frametime recording
            _gpuFrameTimer->startMeasurement(300, [self](const GPUFrameTimer::Results& results) {
                self->_lastGPUFrameResults = results;
                self->_hasGPUFrameResults = YES;
            });
            _benchmarkPhase = 4;
            break;
            
        case 4: // Wait for frametime recording to complete
            if (!_gpuFrameTimer->isActive()) {
                // Store results
                BenchmarkResult& r = _benchmarkResults[_benchmarkMethodIndex];
                r.helperSum = _lastHelperSum;
                r.helperRatio = _lastHelperRatio;
                r.totalEdgeLength = _hasMeshMetrics ? _lastMeshMetrics.totalEdgeLength : 0;
                r.trisPerTileMean = _hasMeshMetrics ? _lastMeshMetrics.trianglesPerTile_Mean : 0;
                r.trisPerTileMed = _hasMeshMetrics ? _lastMeshMetrics.trianglesPerTile_Median : 0;
                r.trisPerTileP95 = _hasMeshMetrics ? _lastMeshMetrics.trianglesPerTile_P95 : 0;
                r.tilesPerTriMean = _hasMeshMetrics ? _lastMeshMetrics.tilesPerTriangle_Mean : 0;
                r.tilesPerTriMed = _hasMeshMetrics ? _lastMeshMetrics.tilesPerTriangle_Median : 0;
                r.tilesPerTriP95 = _hasMeshMetrics ? _lastMeshMetrics.tilesPerTriangle_P95 : 0;
                r.frametimeMean = _hasGPUFrameResults ? _lastGPUFrameResults.avgMs : 0;
                r.frametimeMed = _hasGPUFrameResults ? _lastGPUFrameResults.p50Ms : 0;
                r.frametimeDev = _hasGPUFrameResults ? _lastGPUFrameResults.stdDevMs : 0;
                r.triCount = _hasMeshMetrics ? _lastMeshMetrics.triangleCount : 0;
                
                _benchmarkMethodIndex++;
                if (_benchmarkMethodIndex >= 11) {
                    // Done - print results
                    printf("\nMethod\tTriCount\tTotalEdgeLength\tHelperSum\tHelperRatio\tTris/Tile Mean\tTris/Tile Med\tTris/Tile P95\tTiles/Tri Mean\tTiles/Tri Med\tTiles/Tri P95\tFrametime Mean\tFrametime Med\tFrametime Dev\n");
                    for (int i = 0; i < 11; i++) {
                        if (i == 3) continue;  // Skip CentroidFan
                        BenchmarkResult& br = _benchmarkResults[i];
                        printf("%s\t%zu\t%.2f\t%llu\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%.3f\t%.3f\n",
                               methodNames[i], br.triCount, br.totalEdgeLength, br.helperSum, br.helperRatio,
                               br.trisPerTileMean, br.trisPerTileMed, br.trisPerTileP95,
                               br.tilesPerTriMean, br.tilesPerTriMed, br.tilesPerTriP95,
                               br.frametimeMean, br.frametimeMed, br.frametimeDev);
                    }
                    printf("\n");
                    _benchmarkRunning = NO;
                } else {
                    _benchmarkPhase = 0;
                }
            }
            break;
    }
}

@end

