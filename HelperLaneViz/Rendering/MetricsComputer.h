//
//  MetricsComputer.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import <Metal/Metal.h>
#import <simd/simd.h>

@class GeometryManager;

NS_ASSUME_NONNULL_BEGIN

@interface MetricsComputer : NSObject

- (instancetype)initWithDevice:(id<MTLDevice>)device commandQueue:(id<MTLCommandQueue>)commandQueue;

// Compute overdraw metrics using GPU rasterization (accurate)
// Returns total pixel draws and overdraw ratio (totalDraws / uniquePixels)
- (void)computeOverdrawMetricsWithGeometry:(GeometryManager *)geometry
                                overdrawSum:(uint64_t* _Nullable)outSum
                               overdrawRatio:(double* _Nullable)outRatio;

// Compute helper invocation metrics (GPU-based)
// Returns total helper invocations and helper ratio (totalHelpers / uniqueHelperPixels)
- (void)computeHelperInvocationMetricsWithGeometry:(GeometryManager *)geometry
                                         helperSum:(uint64_t* _Nullable)outSum
                                        helperRatio:(double* _Nullable)outRatio;

@end

NS_ASSUME_NONNULL_END

