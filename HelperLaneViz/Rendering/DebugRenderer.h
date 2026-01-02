//
//  DebugRenderer.h
//  HelperLaneViz
//
//  Created on 2025. 01. 27..
//

#import <MetalKit/MetalKit.h>
#import "ShaderTypes.h"
#import "GeometryManager.h"

@class GeometryManager;
@class MetricsComputer;

typedef NS_ENUM(NSInteger, VisualizationMode) {
    VisualizationModeHelperLane,
    VisualizationModeWireframe,
    VisualizationModeOverdraw,
    VisualizationModePrintFriendly
};

NS_ASSUME_NONNULL_BEGIN

@interface DebugRenderer : NSObject

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view;

- (void)updateDisplaySize:(CGSize)size;

// Start a new ImGUI frame (call before rendering UI)
// Returns NO if frame should be skipped (e.g., MSAA change pending)
- (BOOL)newFrameWithRenderPassDescriptor:(MTLRenderPassDescriptor *)renderPassDescriptor;

// Render all debug UI windows
- (void)renderUIWithGeometry:(GeometryManager *)geometry
                      metrics:(MetricsComputer *)metrics
              onGeometryReload:(void(^)(NSString *path, TriangulationMethod method, uint32_t cols, uint32_t rows, float bezierDev))reloadBlock
               onEllipseReload:(void(^)(float axisRatio, int vertexCount, TriangulationMethod method, uint32_t cols, uint32_t rows))ellipseBlock;

// Render ImGUI draw data to the command encoder (call after renderUI)
- (void)renderDrawDataWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                          commandEncoder:(id<MTLRenderCommandEncoder>)commandEncoder;

// Cleanup (call in dealloc)
- (void)shutdown;

// Accessors for UI state
- (BOOL)showGridOverlay;
- (void)setShowGridOverlay:(BOOL)show;
- (VisualizationMode)visualizationMode;
- (void)setVisualizationMode:(VisualizationMode)mode;
- (int)msaaSampleCount;
- (uint32_t)tileSizePx;
- (void)setTileSizePx:(uint32_t)tileSize;

- (void)setCurrentSVGPath:(NSString *)path
                    method:(TriangulationMethod)method
                      cols:(uint32_t)cols
                      rows:(uint32_t)rows
          bezierDeviation:(float)bezierDev;

- (void*)gpuFrameTimer;  // Returns GPUFrameTimer* (void* for Obj-C compatibility)

@end

NS_ASSUME_NONNULL_END

