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
#include "../Geometry/GeometryFactory.h"

#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <AppKit/AppKit.h>
#include <mach/mach_time.h>

#include "../../external/imgui/imgui.h"
#include "../../external/imgui/backends/imgui_impl_metal.h"
#include "../../external/imgui/backends/imgui_impl_osx.h"

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
    BenchmarkResult _benchmarkResults[11];  // One per triangulation method (using actual method indices)
    // Benchmark methods in order: CDT, CDT flipped, Earcut, Earcut flipped, Greedy Max Area, MWT
    int _benchmarkMethodIndices[6];  // Maps benchmark index to actual method index
    
    // Batch benchmark state
    NSArray<NSString *> *_batchFiles;  // Array of SVG file paths
    int _batchFileIndex;  // Current file being benchmarked
    NSString *_batchOutputFolder;  // Folder path for CSV output
    FILE *_csvFile;  // CSV file handle
    BOOL _batchIncludeSyntheticShapes;  // Whether to include circle and ellipse in batch
    int _syntheticShapeIndex;  // 0 = circle, 1 = ellipse, -1 = none
    
    // Display size tracking
    CGSize _displaySize;
    
    // Window size controls
    int _windowWidth;
    int _windowHeight;
    BOOL _windowSizeChanged;
    
    // Helper texture usage
    BOOL _useHelperTexture;
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
    _batchFiles = nil;
    _batchFileIndex = -1;
    _batchOutputFolder = nil;
    _csvFile = nullptr;
    _batchIncludeSyntheticShapes = NO;
    _syntheticShapeIndex = -1;
    _displaySize = CGSizeZero;
    
    // Initialize benchmark method indices in order: CDT, CDT flipped, Earcut, Earcut flipped, Greedy Max Area, MWT
    _benchmarkMethodIndices[0] = TriangulationMethodConstrainedDelaunay;  // CDT
    _benchmarkMethodIndices[1] = TriangulationMethodConstrainedDelaunayFlipped;  // CDT flipped
    _benchmarkMethodIndices[2] = TriangulationMethodEarClippingTriangulator;  // Earcut
    _benchmarkMethodIndices[3] = TriangulationMethodEarClippingTriangulatorFlipped;  // Earcut flipped
    _benchmarkMethodIndices[4] = TriangulationMethodGreedyMaxArea;  // Greedy Max Area
    _benchmarkMethodIndices[5] = TriangulationMethodMinimumWeight;  // MWT
    
    // Initialize window size from actual window
    NSWindow *window = view.window;
    if (window) {
        NSRect contentRect = [window contentRectForFrameRect:window.frame];
        _windowWidth = (int)contentRect.size.width;
        _windowHeight = (int)contentRect.size.height;
    } else {
    _windowWidth = 1024;
    _windowHeight = 768;
    }
    _windowSizeChanged = NO;
    _useHelperTexture = YES;  // Enabled by default
    
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
    
    // Update window size controls to match actual window size (only if not programmatically changing)
    if (!_windowSizeChanged) {
        NSWindow *window = _view.window;
        if (window) {
            NSRect contentRect = [window contentRectForFrameRect:window.frame];
            _windowWidth = (int)contentRect.size.width;
            _windowHeight = (int)contentRect.size.height;
        } else {
            _windowWidth = (int)size.width;
            _windowHeight = (int)size.height;
        }
    }
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
               onEllipseReload:(void(^)(float axisRatio, int vertexCount, TriangulationMethod method, uint32_t cols, uint32_t rows))ellipseBlock
            onHelperTextureChange:(void(^)(BOOL use))helperTextureBlock {
    
    // Main UI: visualization controls + metrics
    ImGui::Begin("HelperLaneViz");
    
    // Visualization Mode
    {
        const char* vizModes[] = { "Helper Lane", "Wireframe", "Overdraw", "Print Friendly", "Simple Texture" };
        int currentMode = (int)_visualizationMode;
        if (ImGui::Combo("Visualization Mode", &currentMode, vizModes, 5)) {
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
    ImGui::Text("Rendering Settings");
    
    // Helper Texture
    {
        bool useTexture = _useHelperTexture;
        if (ImGui::Checkbox("Use Helper Texture (2048x2048)", &useTexture)) {
            _useHelperTexture = useTexture ? YES : NO;
            if (helperTextureBlock) {
                helperTextureBlock(_useHelperTexture);
            }
        }
    }
    
    ImGui::Separator();
    ImGui::Text("Window Settings");
    
    // Window Size
    {
        int width = _windowWidth;
        int height = _windowHeight;
        // Use InputInt without step constraints to allow any value
        bool widthChanged = ImGui::InputInt("Width (px)", &width, 0, 0, ImGuiInputTextFlags_None);
        bool heightChanged = ImGui::InputInt("Height (px)", &height, 0, 0, ImGuiInputTextFlags_None);
        
        if (widthChanged || heightChanged) {
            // Clamp to reasonable bounds (allow up to 8K resolution)
            width = MAX(100, MIN(8192, width));
            height = MAX(100, MIN(8192, height));
            
            if (width != _windowWidth || height != _windowHeight) {
                _windowWidth = width;
                _windowHeight = height;
                _windowSizeChanged = YES;
            }
        }
    }
    
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
            // Update grid without reloading geometry (fast)
            [geometry updateInstanceGridWithCols:_instanceGridCols rows:_instanceGridRows];
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
            _batchFiles = nil;
            _batchFileIndex = -1;
            memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
        }
        ImGui::SameLine();
        if (ImGui::Button("Benchmark Folder")) {
            [self selectFolderAndStartBatchBenchmark:reloadBlock];
        }
    } else {
        // Display progress
        if (_batchFiles && _batchFileIndex >= 0 && _batchFileIndex < (int)_batchFiles.count) {
            NSString *fileName = [[_batchFiles[_batchFileIndex] lastPathComponent] stringByDeletingPathExtension];
            ImGui::Text("File %d/%zu: %s, Method %d/6, phase %d...",
                       _batchFileIndex + 1, _batchFiles.count, fileName.UTF8String, _benchmarkMethodIndex + 1, _benchmarkPhase);
        } else if (_syntheticShapeIndex >= 0) {
            const char* shapeName = (_syntheticShapeIndex == 0) ? "Circle (512 vertices)" : "Ellipse (512 vertices, y=0.5x)";
            ImGui::Text("Shape: %s, Method %d/6, phase %d...", shapeName, _benchmarkMethodIndex + 1, _benchmarkPhase);
        } else {
            ImGui::Text("Benchmarking method %d/6, phase %d...", _benchmarkMethodIndex + 1, _benchmarkPhase);
        }
    }
    ImGui::End();
    
    // Benchmark state machine
    if (_benchmarkRunning) {
        [self runBenchmarkStepWithGeometry:geometry metrics:metrics reloadBlock:reloadBlock ellipseBlock:ellipseBlock];
    }
    
    // Apply window size changes
    if (_windowSizeChanged) {
        NSWindow *window = _view.window;
        if (window) {
            NSRect frame = window.frame;
            NSRect contentRect = [window contentRectForFrameRect:frame];
            CGFloat titleBarHeight = frame.size.height - contentRect.size.height;
            
            // Calculate new frame size (content size + title bar)
            NSSize newContentSize = NSMakeSize((CGFloat)_windowWidth, (CGFloat)_windowHeight);
            NSRect newContentRect = NSMakeRect(frame.origin.x, frame.origin.y, newContentSize.width, newContentSize.height);
            NSRect newFrame = [window frameRectForContentRect:newContentRect];
            
            // Keep the top-left corner in place
            newFrame.origin.y = frame.origin.y + frame.size.height - newFrame.size.height;
            
            [window setFrame:newFrame display:YES animate:NO];
        }
        _windowSizeChanged = NO;
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
    
    // Close CSV file if still open
    if (_csvFile) {
        fclose(_csvFile);
        _csvFile = nullptr;
    }
    
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
    
    // Map benchmark index to actual method index
    int actualMethodIndex = _benchmarkMethodIndices[_benchmarkMethodIndex];
    
    switch (_benchmarkPhase) {
        case 0: // Reload geometry with current method
            if (_syntheticShapeIndex >= 0) {
                // Generate synthetic shape using GeometryFactory
                std::vector<Vertex> vertices;
                if (_syntheticShapeIndex == 0) {
                    // Circle with 512 vertices, radius = 1.0
                    vertices = GeometryFactory::CreateVerticesForCircle(512, 1.0f);
                } else {
                    // Ellipse with 512 vertices, a=1.0, b=0.5 (y-axis is 0.5 * x-axis)
                    vertices = GeometryFactory::CreateVerticesForEllipse(512, 1.0f, 0.5f);
                }
                
                // Triangulate using the current method
                std::vector<uint32_t> indices;
                switch ((TriangulationMethod)actualMethodIndex) {
                    case TriangulationMethodEarClipping:
                        indices = Triangulation::EarClipping(vertices);
                        break;
                    case TriangulationMethodEarClippingTriangulator:
                        indices = Triangulation::EarClippingMapbox(vertices);
                        break;
                    case TriangulationMethodEarClippingTriangulatorFlipped:
                        indices = Triangulation::EarClippingMapboxWithEdgeFlips(vertices);
                        break;
                    case TriangulationMethodCentroidFan:
                        indices = Triangulation::CentroidFan(vertices);
                        break;
                    case TriangulationMethodStrip:
                        indices = Triangulation::Strip(vertices);
                        break;
                    case TriangulationMethodGreedyMaxArea:
                        indices = Triangulation::GreedyMaxArea(vertices, false);
                        break;
                    case TriangulationMethodMinimumWeight:
                        indices = Triangulation::MinimumWeight(vertices, false);
                        break;
                    case TriangulationMethodMaxMinArea:
                        indices = Triangulation::MaxMinArea(vertices, false);
                        break;
                    case TriangulationMethodMinMaxArea:
                        indices = Triangulation::MinMaxArea(vertices, false);
                        break;
                    case TriangulationMethodConstrainedDelaunay:
                        indices = Triangulation::ConstrainedDelaunay(vertices);
                        break;
                    case TriangulationMethodConstrainedDelaunayFlipped:
                        indices = Triangulation::ConstrainedDelaunayWithEdgeFlips(vertices);
                        break;
                }
                
                // Assert that triangulation succeeded - crash if it failed
                NSAssert(!indices.empty() && vertices.size() >= 3, 
                        @"Triangulation failed for synthetic shape: method=%d, shape=%d, vertices=%zu, indices=%zu", 
                        actualMethodIndex, _syntheticShapeIndex, vertices.size(), indices.size());
                
                // Upload geometry to GeometryManager
                [geometry loadGeometryFromVertices:vertices indices:indices instanceGridCols:_instanceGridCols gridRows:_instanceGridRows];
            } else if (_shapeType == 0 && _currentSVGPath) {
                reloadBlock(_currentSVGPath,
                            (TriangulationMethod)actualMethodIndex,
                            _instanceGridCols,
                            _instanceGridRows,
                            _bezierMaxDeviationPx);
            } else if (_shapeType == 1) {
                ellipseBlock(_ellipseAxisRatio,
                             _ellipseVertexCount,
                             (TriangulationMethod)actualMethodIndex,
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
            _benchmarkDelayFrames = 100;
            _benchmarkPhase = 2;
        }
            break;
            
        case 2: // Delay frames to let GPU settle after geometry change
            if (--_benchmarkDelayFrames <= 0) {
                _benchmarkPhase = 3;
            }
            break;
            
        case 3: // Start frametime recording
            _gpuFrameTimer->startMeasurement(200, [self](const GPUFrameTimer::Results& results) {
                self->_lastGPUFrameResults = results;
                self->_hasGPUFrameResults = YES;
            });
            _benchmarkPhase = 4;
            break;
            
        case 4: // Wait for frametime recording to complete
            if (!_gpuFrameTimer->isActive()) {
                // Store results (using actual method index for storage)
                BenchmarkResult& r = _benchmarkResults[actualMethodIndex];
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
                if (_benchmarkMethodIndex >= 6) {  // Only 6 methods to benchmark
                    // Done with all methods - print results
                    NSString *fileName = nil;
                    if (_batchFiles && _batchFileIndex >= 0 && _batchFileIndex < (int)_batchFiles.count) {
                        fileName = [[_batchFiles[_batchFileIndex] lastPathComponent] stringByDeletingPathExtension];
                        printf("\n=== File: %s ===\n", fileName.UTF8String);
                    } else if (_syntheticShapeIndex >= 0) {
                        fileName = (_syntheticShapeIndex == 0) ? @"Circle_512" : @"Ellipse_512_y0.5x";
                        printf("\n=== Shape: %s ===\n", fileName.UTF8String);
                    }
                    printf("Method\tTriCount\tTotalEdgeLength\tHelperSum\tHelperRatio\tTris/Tile Mean\tTris/Tile Med\tTris/Tile P95\tTiles/Tri Mean\tTiles/Tri Med\tTiles/Tri P95\tFrametime Mean\tFrametime Med\tFrametime Dev\n");
                    // Output results in benchmark order: CDT, CDT flipped, Earcut, Earcut flipped, Greedy Max Area, MWT
                    for (int i = 0; i < 6; i++) {
                        int methodIdx = _benchmarkMethodIndices[i];
                        BenchmarkResult& br = _benchmarkResults[methodIdx];
                        printf("%s\t%zu\t%.2f\t%llu\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%.3f\t%.3f\n",
                               methodNames[methodIdx], br.triCount, br.totalEdgeLength, br.helperSum, br.helperRatio,
                               br.trisPerTileMean, br.trisPerTileMed, br.trisPerTileP95,
                               br.tilesPerTriMean, br.tilesPerTriMed, br.tilesPerTriP95,
                               br.frametimeMean, br.frametimeMed, br.frametimeDev);
                        
                        // Write to CSV if batch mode
                        if (_csvFile && fileName) {
                            fprintf(_csvFile, "%s,%s,%zu,%.2f,%llu,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
                                    fileName.UTF8String, methodNames[methodIdx], br.triCount, br.totalEdgeLength, br.helperSum, br.helperRatio,
                                    br.trisPerTileMean, br.trisPerTileMed, br.trisPerTileP95,
                                    br.tilesPerTriMean, br.tilesPerTriMed, br.tilesPerTriP95,
                                    br.frametimeMean, br.frametimeMed, br.frametimeDev);
                            fflush(_csvFile);  // Flush after each write to ensure data is saved
                        }
                    }
                    printf("\n");
                    
                    // Check if batch mode - move to next file or synthetic shape
                    if (_batchFiles && _batchFileIndex >= 0) {
                        _batchFileIndex++;
                        if (_batchFileIndex < (int)_batchFiles.count) {
                            // Load next file and restart benchmark
                            NSString *nextFile = _batchFiles[_batchFileIndex];
                            reloadBlock(nextFile, (TriangulationMethod)0, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
                            _currentSVGPath = nextFile;
                            _benchmarkMethodIndex = 0;
                            _benchmarkPhase = 1;  // Skip reload phase, go straight to compute metrics (file already loaded)
                            memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
                        } else if (_batchIncludeSyntheticShapes && _syntheticShapeIndex < 0) {
                            // Move to synthetic shapes (first one: circle)
                            _syntheticShapeIndex = 0;
                            _benchmarkMethodIndex = 0;
                            _benchmarkPhase = 0;  // Start with reload phase for synthetic shape
                            memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
                        } else if (_batchIncludeSyntheticShapes && _syntheticShapeIndex >= 0 && _syntheticShapeIndex < 1) {
                            // Move to next synthetic shape (ellipse)
                            _syntheticShapeIndex = 1;
                            _benchmarkMethodIndex = 0;
                            _benchmarkPhase = 0;  // Start with reload phase for synthetic shape
                            memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
                        } else {
                            // Done with all files and synthetic shapes
                            printf("=== Batch benchmark complete ===\n\n");
                            if (_csvFile) {
                                fclose(_csvFile);
                                _csvFile = nullptr;
                                printf("Results saved to: %s/benchmark_results.csv\n", _batchOutputFolder.UTF8String);
                            }
                            _benchmarkRunning = NO;
                            _batchFiles = nil;
                            _batchFileIndex = -1;
                            _batchOutputFolder = nil;
                            _batchIncludeSyntheticShapes = NO;
                            _syntheticShapeIndex = -1;
                        }
                    } else if (_syntheticShapeIndex >= 0) {
                        // Processing synthetic shapes (standalone, not in batch mode)
                        if (_syntheticShapeIndex < 1) {
                            // Move to next synthetic shape (ellipse)
                            _syntheticShapeIndex = 1;
                            _benchmarkMethodIndex = 0;
                            _benchmarkPhase = 0;  // Start with reload phase for synthetic shape
                            memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
                        } else {
                            // Done with all synthetic shapes
                            printf("=== Batch benchmark complete ===\n\n");
                            if (_csvFile) {
                                fclose(_csvFile);
                                _csvFile = nullptr;
                                printf("Results saved to: %s/benchmark_results.csv\n", _batchOutputFolder.UTF8String);
                            }
                            _benchmarkRunning = NO;
                            _batchFiles = nil;
                            _batchFileIndex = -1;
                            _batchOutputFolder = nil;
                            _batchIncludeSyntheticShapes = NO;
                            _syntheticShapeIndex = -1;
                        }
                    } else {
                        // Single file benchmark done
                        _benchmarkRunning = NO;
                    }
                } else {
                    _benchmarkPhase = 0;
                }
            }
            break;
    }
}

- (void)selectFolderAndStartBatchBenchmark:(void(^)(NSString *path, TriangulationMethod method, uint32_t cols, uint32_t rows, float bezierDev))reloadBlock {
    NSOpenPanel *panel = [NSOpenPanel openPanel];
    panel.canChooseFiles = NO;
    panel.canChooseDirectories = YES;
    panel.allowsMultipleSelection = NO;
    panel.message = @"Select folder containing SVG files";
    
    // Request write access for the selected folder (needed for CSV output)
    if ([panel runModal] == NSModalResponseOK) {
        NSURL *folderURL = panel.URL;
        
        // Start accessing security-scoped resource (needed for sandboxed apps)
        BOOL accessing = [folderURL startAccessingSecurityScopedResource];
        if (!accessing) {
            NSLog(@"Warning: Could not access security-scoped resource");
        }
        
        NSString *folderPath = folderURL.path;
        
        // Get all SVG files in folder
        NSFileManager *fileManager = [NSFileManager defaultManager];
        NSError *error = nil;
        NSArray<NSString *> *allFiles = [fileManager contentsOfDirectoryAtPath:folderPath error:&error];
        
        if (error) {
            NSLog(@"Error reading folder: %@", error);
            return;
        }
        
        // Filter for .svg files
        NSMutableArray<NSString *> *svgFiles = [NSMutableArray array];
        for (NSString *file in allFiles) {
            if ([[file pathExtension] caseInsensitiveCompare:@"svg"] == NSOrderedSame) {
                NSString *fullPath = [folderPath stringByAppendingPathComponent:file];
                [svgFiles addObject:fullPath];
            }
        }
        
        if (svgFiles.count == 0) {
            NSLog(@"No SVG files found in folder: %@", folderPath);
            return;
        }
        
        // Sort files alphabetically
        [svgFiles sortUsingSelector:@selector(compare:)];
        
        // Create CSV file in selected folder
        _batchOutputFolder = folderPath;
        NSString *csvPath = [folderPath stringByAppendingPathComponent:@"benchmark_results.csv"];
        _csvFile = fopen(csvPath.UTF8String, "w");
        if (_csvFile) {
            fprintf(_csvFile, "File,Method,TriCount,TotalEdgeLength,HelperSum,HelperRatio,TrisPerTileMean,TrisPerTileMed,TrisPerTileP95,TilesPerTriMean,TilesPerTriMed,TilesPerTriP95,FrametimeMean,FrametimeMed,FrametimeDev\n");
            fflush(_csvFile);  // Ensure header is written immediately
            printf("CSV file created: %s\n", csvPath.UTF8String);
        } else {
            NSLog(@"Failed to create CSV file at: %@", csvPath);
            printf("ERROR: Failed to create CSV file at: %s\n", csvPath.UTF8String);
        }
        
        // Start batch benchmark with first file
        _batchFiles = svgFiles;
        _batchFileIndex = 0;
        _batchIncludeSyntheticShapes = YES;  // Include circle and ellipse after SVGs
        _syntheticShapeIndex = -1;  // Start with SVGs
        _benchmarkRunning = YES;
        _benchmarkMethodIndex = 0;
        _benchmarkPhase = 0;
        memset(_benchmarkResults, 0, sizeof(_benchmarkResults));
        
        NSString *firstFile = _batchFiles[0];
        reloadBlock(firstFile, (TriangulationMethod)0, _instanceGridCols, _instanceGridRows, _bezierMaxDeviationPx);
        _currentSVGPath = firstFile;
        _shapeType = 0;  // Ensure SVG mode
        
        printf("\n=== Starting batch benchmark: %zu SVG files + 2 synthetic shapes ===\n\n", svgFiles.count);
    }
}

@end

