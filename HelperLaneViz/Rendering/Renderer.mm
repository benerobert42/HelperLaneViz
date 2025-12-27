//
//  Renderer.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import "Renderer.h"
#import "GeometryManager.h"
#import "MetricsComputer.h"
#import "RenderingManager.h"
#import "Measurements/GPUFrameTimer.h"

#import <MetalKit/MetalKit.h>

@implementation Renderer {
    // Core Metal objects
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    MTKView *_view;
    
    // Managers
    GeometryManager *_geometryManager;
    MetricsComputer *_metricsComputer;
    RenderingManager *_renderingManager;
}

@synthesize showGridOverlay = _showGridOverlay;

// MARK: - Initialization

- (instancetype)initWithMetalKitView:(MTKView *)mtkView {
    if (!(self = [super init])) return nil;
    
    _device = mtkView.device;
    _view = mtkView;
    _commandQueue = [_device newCommandQueue];
    
    // Set initial MSAA (must be before RenderingManager so pipelines match)
    NSInteger initialSampleCount = 2;
    if ([_device supportsTextureSampleCount:initialSampleCount]) {
        _view.sampleCount = initialSampleCount;
    }
    
    // Initialize managers
    _geometryManager = [[GeometryManager alloc] initWithDevice:_device];
    _metricsComputer = [[MetricsComputer alloc] initWithDevice:_device commandQueue:_commandQueue];
    _renderingManager = [[RenderingManager alloc] initWithDevice:_device view:mtkView];
    
    // Set initial viewport size
    vector_uint2 viewportSize = {(uint32_t)mtkView.drawableSize.width, (uint32_t)mtkView.drawableSize.height};
    [_geometryManager updateViewportSize:viewportSize];
    
    // Setup view (no depth buffer needed for 2D visualization)
    _view.clearColor = MTLClearColorMake(0, 0, 0, 1);
    _view.depthStencilPixelFormat = MTLPixelFormatInvalid;

    
    // Load default SVG (can be removed or made configurable)
    NSString *defaultSVGPath = @"/Users/robi/Downloads/1295383.svg";
    [self loadSVGFromPath:defaultSVGPath
      triangulationMethod:TriangulationMethodMinimumWeight
         instanceGridCols:5
                 gridRows:5
    bezierMaxDeviationPx:1.0f];
    
    return self;
}

// MARK: - Public Interface

- (BOOL)loadSVGFromPath:(NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows
    bezierMaxDeviationPx:(float)bezierMaxDeviationPx {
    
    BOOL success = [_geometryManager loadSVGFromPath:path
                                  triangulationMethod:method
                                     instanceGridCols:cols
                                             gridRows:rows
                                  bezierMaxDeviationPx:bezierMaxDeviationPx];
    
    if (success) {
        [_renderingManager setCurrentSVGPath:path
                                      method:method
                                        cols:cols
                                        rows:rows
                            bezierDeviation:bezierMaxDeviationPx];
    }
    
    return success;
}

- (void)computeOverdrawMetricsWithOverdrawSum:(uint64_t*)outSum overdrawRatio:(double*)outRatio {
    [_metricsComputer computeOverdrawMetricsWithGeometry:_geometryManager
                                               overdrawSum:outSum
                                              overdrawRatio:outRatio];
}

- (void)computeHelperInvocationMetricsWithHelperSum:(uint64_t*)outSum helperRatio:(double*)outRatio {
    [_metricsComputer computeHelperInvocationMetricsWithGeometry:_geometryManager
                                                       helperSum:outSum
                                                      helperRatio:outRatio];
}

- (BOOL)showGridOverlay {
    return _renderingManager.showGridOverlay;
}

- (void)setShowGridOverlay:(BOOL)show {
    _renderingManager.showGridOverlay = show;
}

// MARK: - MTKViewDelegate

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    vector_uint2 viewportSize = {(uint32_t)size.width, (uint32_t)size.height};
    [_geometryManager updateViewportSize:viewportSize];
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    
    // Render UI (may return NO to skip frame, e.g. on MSAA change)
    BOOL shouldRender = [_renderingManager renderUIWithGeometry:_geometryManager
                                     metrics:_metricsComputer
                             onGeometryReload:^(NSString *path, TriangulationMethod method, uint32_t cols, uint32_t rows, float bezierDev) {
        [self->_geometryManager loadSVGFromPath:path
              triangulationMethod:method
                 instanceGridCols:cols
                         gridRows:rows
            bezierMaxDeviationPx:bezierDev];
    }
                              onEllipseReload:^(float axisRatio, int vertexCount, TriangulationMethod method, uint32_t cols, uint32_t rows) {
        [self->_geometryManager generateEllipseWithAxisRatio:axisRatio
                                                 vertexCount:vertexCount
                                         triangulationMethod:method
                                            instanceGridCols:cols
                                                    gridRows:rows];
    }];
    
    if (!shouldRender) {
        [commandBuffer commit];
        return;
    }
    
    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;
    if (!renderPassDescriptor) {
        [commandBuffer commit];
        return;
    }
    
    id<MTLRenderCommandEncoder> encoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

    // Encode geometry rendering
    [_renderingManager encodeGeometryWithEncoder:encoder
                                         geometry:_geometryManager
                                             mode:_renderingManager.visualizationMode];

    // Draw grid overlay if enabled (not shown during overdraw mode)
    if (_renderingManager.showGridOverlay && _renderingManager.visualizationMode != VisualizationModeOverdraw) {
        [_renderingManager encodeGridOverlayWithEncoder:encoder drawableSize:view.drawableSize];
    }
    
    // Render ImGUI (on top of everything)
    [_renderingManager renderImGUIWithCommandBuffer:commandBuffer commandEncoder:encoder];

    [encoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
    
    // Record GPU frame time (actual GPU execution: GPUEndTime - GPUStartTime)
    GPUFrameTimer *gpuFrameTimer = (GPUFrameTimer *)[_renderingManager gpuFrameTimer];
    if (gpuFrameTimer && gpuFrameTimer->isActive()) {
        [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> buffer) {
            gpuFrameTimer->recordGPUEndTime(buffer.GPUStartTime, buffer.GPUEndTime);
        }];
    }
    
    [commandBuffer commit];
}

@end
