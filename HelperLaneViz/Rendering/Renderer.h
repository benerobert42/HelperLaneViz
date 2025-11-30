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

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView;

// Run the complete benchmark suite (36 scenes × 7 methods = 252 tests).
// 3 shapes × 4 vertex counts × 3 instance counts.
// Results are printed to console and exported as CSV.
- (void)runDefaultBenchmark;

// Run a quick benchmark with reduced test matrix for faster iteration.
- (void)runQuickBenchmark;

@end
