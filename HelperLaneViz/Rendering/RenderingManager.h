//
//  RenderingManager.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import <MetalKit/MetalKit.h>
#import "ShaderTypes.h"
#import "GeometryManager.h"

@class GeometryManager;
@class MetricsComputer;

typedef NS_ENUM(NSInteger, VisualizationMode) {
    VisualizationModeHelperLane,  // Helper lane visualization (default)
    VisualizationModeWireframe,   // Wireframe with fill mode lines
    VisualizationModeOverdraw      // Overdraw visualization
};

NS_ASSUME_NONNULL_BEGIN

@interface RenderingManager : NSObject

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view;

- (void)renderUIWithGeometry:(GeometryManager *)geometry
                     metrics:(MetricsComputer *)metrics
            onGeometryReload:(void(^)(NSString *path, TriangulationMethod method, uint32_t cols, uint32_t rows, float bezierDev))reloadBlock;

// Encode geometry rendering for specified visualization mode
- (void)encodeGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                         geometry:(GeometryManager *)geometry
                             mode:(VisualizationMode)mode;

- (void)encodeGridOverlayWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                        drawableSize:(CGSize)drawableSize;

- (void)renderImGUIWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                      commandEncoder:(id<MTLRenderCommandEncoder>)commandEncoder;

// Accessors for UI state
- (BOOL)showGridOverlay;
- (void)setShowGridOverlay:(BOOL)show;
- (VisualizationMode)visualizationMode;
- (void)setVisualizationMode:(VisualizationMode)mode;
- (uint32_t)tileSizePx;
- (void)setTileSizePx:(uint32_t)tileSize;

- (void)setCurrentSVGPath:(NSString *)path
                    method:(TriangulationMethod)method
                      cols:(uint32_t)cols
                      rows:(uint32_t)rows
          bezierDeviation:(float)bezierDev;

@end

NS_ASSUME_NONNULL_END

