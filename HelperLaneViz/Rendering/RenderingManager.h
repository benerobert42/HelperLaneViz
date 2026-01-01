//
//  RenderingManager.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import <MetalKit/MetalKit.h>
#import "ShaderTypes.h"
#import "DebugRenderer.h"
#import "GeometryManager.h"

@class GeometryManager;
@class MetricsComputer;

NS_ASSUME_NONNULL_BEGIN

@interface RenderingManager : NSObject

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view;

// Update pipelines when MSAA sample count changes
- (void)updatePipelinesForCurrentSampleCount;

// Encode geometry rendering for specified visualization mode
- (void)encodeGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                         geometry:(GeometryManager *)geometry
                             mode:(VisualizationMode)mode;

- (void)encodeGridOverlayWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                        drawableSize:(CGSize)drawableSize;

// Access DebugRenderer for UI rendering
- (DebugRenderer *)debugRenderer;

// Accessors for UI state (delegated to DebugRenderer)
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

