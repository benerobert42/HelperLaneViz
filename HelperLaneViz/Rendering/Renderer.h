//
//  Renderer.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import <MetalKit/MetalKit.h>

#ifdef __cplusplus
#import "GeometryManager.h"
#import "RenderingManager.h"
#else
// Forward declarations for Objective-C
@class GeometryManager;
@class RenderingManager;
typedef NS_ENUM(NSInteger, TriangulationMethod) {
    TriangulationMethodEarClipping,
    TriangulationMethodMinimumWeight,
    TriangulationMethodCentroidFan,
    TriangulationMethodGreedyMaxArea,
    TriangulationMethodStrip,
    TriangulationMethodMaxMinArea,
    TriangulationMethodMinMaxArea,
    TriangulationMethodConstrainedDelaunay
};
typedef NS_ENUM(NSInteger, VisualizationMode) {
    VisualizationModeHelperLane,
    VisualizationModeWireframe,
    VisualizationModeOverdraw
};
#endif

// Forward declarations
@class MetricsComputer;

@interface Renderer : NSObject <MTKViewDelegate>

@property (nonatomic, assign) BOOL showGridOverlay;

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView;

- (BOOL)loadSVGFromPath:(nonnull NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows
    bezierMaxDeviationPx:(float)bezierMaxDeviationPx;

// Compute overdraw metrics using GPU rasterization (accurate)
// Returns total pixel draws and overdraw ratio (totalDraws / uniquePixels)
- (void)computeOverdrawMetricsWithOverdrawSum:(uint64_t* _Nullable)outSum
                                overdrawRatio:(double* _Nullable)outRatio;

// Compute helper invocation metrics (GPU-based)
// Returns total helper invocations and helper ratio (totalHelpers / uniqueHelperPixels)
- (void)computeHelperInvocationMetricsWithHelperSum:(uint64_t* _Nullable)outSum
                                       helperRatio:(double* _Nullable)outRatio;

@end
