//
//  Renderer.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import <MetalKit/MetalKit.h>

typedef NS_ENUM(NSInteger, TriangulationMethod) {
    TriangulationMethodMinimumWeight,
    TriangulationMethodCentroidFan,
    TriangulationMethodGreedyMaxArea,
    TriangulationMethodStrip,
    TriangulationMethodMaxMinArea,
    TriangulationMethodMinMaxArea,
    TriangulationMethodConstrainedDelaunay
};

@interface Renderer : NSObject <MTKViewDelegate>

@property (nonatomic, assign) BOOL showGridOverlay;
@property (nonatomic, assign) BOOL showHeatmap;    // Only has effect when showGridOverlay is YES
@property (nonatomic, assign) BOOL showOverdraw;   // Visualize pixel overdraw (how often each pixel is drawn)

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView;

// Load SVG and triangulate with specified method
- (BOOL)loadSVGFromPath:(nonnull NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridSize:(uint32_t)gridSize;

// Run the complete benchmark suite (36 scenes × 7 methods = 252 tests).
// 3 shapes × 4 vertex counts × 3 instance counts.
// Results are printed to console and exported as CSV.
- (void)runDefaultBenchmark;

// Run a quick benchmark with reduced test matrix for faster iteration.
- (void)runQuickBenchmark;

// Compute overdraw metrics using GPU rasterization (accurate)
// Returns total pixel draws and overdraw ratio (totalDraws / uniquePixels)
- (void)computeOverdrawMetricsWithOverdrawSum:(uint64_t* _Nullable)outSum
                                overdrawRatio:(double* _Nullable)outRatio;

@end
