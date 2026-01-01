//
//  GeometryManager1.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 12. 15..
//

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

// Load SVG and triangulate with specified method
- (BOOL)loadSVGFromPath:(NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows
    bezierMaxDeviationPx:(float)bezierMaxDeviationPx;

// Generate ellipse geometry with given axis ratio (minorAxis/majorAxis)
- (BOOL)generateEllipseWithAxisRatio:(float)axisRatio
                         vertexCount:(int)vertexCount
                 triangulationMethod:(TriangulationMethod)method
                    instanceGridCols:(uint32_t)cols
                            gridRows:(uint32_t)rows;

// Update viewport size (called when view size changes)
- (void)updateViewportSize:(vector_uint2)size;

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

