//
//  Renderer.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import <MetalKit/MetalKit.h>

@interface Renderer : NSObject <MTKViewDelegate>

@property (nonatomic, assign) BOOL showGridOverlay;
@property (nonatomic, assign) BOOL showHeatmap; // Only has effect when showGridOverlay is YES
@property (nonatomic, assign) BOOL showOverdraw; // Show overdraw visualization (pixels colored by draw count)

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView;

// Load an SVG file, tessellate it, and use as render geometry.
// triangulationMethod: 0=MWT, 1=CentroidFan, 2=GreedyMaxArea, 3=Strip, 4=MaxMinArea, 5=MinMaxArea, 6=CDT
// gridSize: Number of instances in each row/column (e.g., 3 for a 3×3 grid)
- (BOOL)loadSVGFromPath:(nonnull NSString *)path
    triangulationMethod:(int)method
       instanceGridSize:(uint32_t)gridSize;

// Run the complete benchmark suite (36 scenes × 7 methods = 252 tests).
// 3 shapes × 4 vertex counts × 3 instance counts.
// Results are printed to console and exported as CSV.
- (void)runDefaultBenchmark;

// Run a quick benchmark with reduced test matrix for faster iteration.
- (void)runQuickBenchmark;

@end
