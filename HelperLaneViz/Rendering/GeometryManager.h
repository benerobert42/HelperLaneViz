#pragma once

#include <vector>
#import "ShaderTypes.h"
#import <MetalKit/MetalKit.h>

@class GeometryManager;

typedef NS_ENUM(NSInteger, TriangulationMethod) {
    TriangulationMethodEarClipping,
    TriangulationMethodEarClippingTriangulator,
    TriangulationMethodEarClippingTriangulatorFlipped,
    TriangulationMethodCentroidFan,
    TriangulationMethodStrip,
    TriangulationMethodGreedyMaxArea,
    TriangulationMethodMinimumWeight,
    TriangulationMethodMaxMinArea,
    TriangulationMethodMinMaxArea,
    TriangulationMethodConstrainedDelaunay,
    TriangulationMethodConstrainedDelaunayFlipped
};

NS_ASSUME_NONNULL_BEGIN

@interface GeometryManager : NSObject

- (instancetype)initWithDevice:(id<MTLDevice>)device;

- (BOOL)loadSVGFromPath:(NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows
    bezierMaxDeviationPx:(float)bezierMaxDeviationPx
       useMeshOptimizer:(BOOL)useMeshOptimizer;

- (BOOL)generateEllipseWithAxisRatio:(float)axisRatio
                         vertexCount:(int)vertexCount
                 triangulationMethod:(TriangulationMethod)method
                    instanceGridCols:(uint32_t)cols
                            gridRows:(uint32_t)rows
                    useMeshOptimizer:(BOOL)useMeshOptimizer;

- (void)loadGeometryFromVertices:(const std::vector<Vertex>&)vertices
                          indices:(const std::vector<uint32_t>&)indices
                 instanceGridCols:(uint32_t)cols
                         gridRows:(uint32_t)rows;

- (void)updateViewportSize:(vector_uint2)size;

// Update instance grid without reloading geometry (for performance)
- (void)updateInstanceGridWithCols:(uint32_t)cols rows:(uint32_t)rows;

// Accessors for geometry data
- (id<MTLBuffer>)vertexBuffer;
- (id<MTLBuffer>)indexBuffer;
- (NSUInteger)indexCount;
- (NSUInteger)instanceCount;

// Accessors for view state
- (GridParams)gridParams;
- (simd_float4x4)viewProjectionMatrix;
- (vector_uint2)viewportSize;

// Accessors for raw geometry data (for metrics computation)
- (const std::vector<Vertex>&)currentVertices;
- (const std::vector<uint32_t>&)currentIndices;

@end

NS_ASSUME_NONNULL_END

