//
//  Renderer.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import <MetalKit/MetalKit.h>

@interface Renderer : NSObject <MTKViewDelegate>

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtkView;

/// Run the complete benchmark suite with default configuration.
/// Results are printed to console and exported as CSV.
- (void)runDefaultBenchmark;

@end
