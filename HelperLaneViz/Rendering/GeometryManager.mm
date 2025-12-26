//
//  GeometryManager.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import "GeometryManager.h"
#import "MathUtilities.h"
#import "../Geometry/GeometryFactory.h"
#import "../Geometry/Triangulation.h"
#import "../InputHandling/SVGLoader.h"

#import <Metal/Metal.h>
#include <vector>

@implementation GeometryManager {
    id<MTLDevice> _device;
    
    // Geometry buffers
    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;
    
    // Current mesh data
    std::vector<Vertex> _currentVertices;
    std::vector<uint32_t> _currentIndices;
    
    // View state
    vector_uint2 _viewportSize;
    simd_float4x4 _viewProjectionMatrix;
    GridParams _gridParams;
    
    // Instance grid settings
    uint32_t _instanceGridCols;
    uint32_t _instanceGridRows;
}

- (instancetype)initWithDevice:(id<MTLDevice>)device {
    if (!(self = [super init])) return nil;
    
    _device = device;
    _viewportSize = (vector_uint2){0, 0};
    _instanceGridCols = 5;
    _instanceGridRows = 5;
    
    [self setupOrthographicProjection];
    
    return self;
}

// MARK: - Public Interface

- (BOOL)loadSVGFromPath:(NSString *)path
    triangulationMethod:(TriangulationMethod)method
       instanceGridCols:(uint32_t)cols
               gridRows:(uint32_t)rows
    bezierMaxDeviationPx:(float)bezierMaxDeviationPx {
    
    _instanceGridCols = cols;
    _instanceGridRows = rows;
    
    // Create triangulator that uses the selected method
    SVGLoader::Triangulator triangulator = [self, method](const std::vector<Vertex>& verts, bool shouldHandleConcave) -> std::vector<uint32_t> {
        return [self triangulateVertices:verts method:method shouldHandleConcave:shouldHandleConcave];
    };
    
    // Tessellate and triangulate each path with the chosen method
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    if (!SVGLoader::TessellateSvgToMesh(path.UTF8String, vertices, indices, triangulator, bezierMaxDeviationPx)) {
        NSLog(@"Failed to load SVG: %@", path);
        return NO;
    }
    
    if (vertices.size() < 3 || indices.empty()) {
        NSLog(@"SVG produced insufficient geometry: %@", path);
        return NO;
    }
    
    _currentVertices = vertices;
    _currentIndices = indices;
    
    [self uploadVertices:vertices indices:indices];
    [self setupOrthographicProjection];
    [self setupInstanceGridWithCols:cols rows:rows];
    
    NSLog(@"Loaded SVG: %@ (%zu vertices, %zu triangles, method=%ld, grid=%dx%d)",
          path, vertices.size(), indices.size() / 3, (long)method, cols, rows);
    
    return YES;
}

- (BOOL)generateEllipseWithAxisRatio:(float)axisRatio
                         vertexCount:(int)vertexCount
                 triangulationMethod:(TriangulationMethod)method
                    instanceGridCols:(uint32_t)cols
                            gridRows:(uint32_t)rows {
    
    _instanceGridCols = cols;
    _instanceGridRows = rows;
    
    // Generate ellipse vertices (centered at origin, major axis = 1.0)
    const int segments = MAX(3, vertexCount);
    const float majorAxis = 1.0f;
    const float minorAxis = majorAxis * axisRatio;
    
    std::vector<Vertex> vertices;
    vertices.reserve(segments);
    
    for (int i = 0; i < segments; ++i) {
        float angle = (float)i / segments * 2.0f * M_PI;
        Vertex v;
        v.position = simd_make_float3(majorAxis * cosf(angle), minorAxis * sinf(angle), 0.0f);
        vertices.push_back(v);
    }
    
    // Triangulate the ellipse polygon
    std::vector<uint32_t> indices = [self triangulateVertices:vertices method:method shouldHandleConcave:NO];
    
    if (indices.empty()) {
        NSLog(@"Failed to triangulate ellipse");
        return NO;
    }
    
    _currentVertices = vertices;
    _currentIndices = indices;
    
    [self uploadVertices:vertices indices:indices];
    [self setupOrthographicProjection];
    [self setupInstanceGridWithCols:cols rows:rows];
    
    NSLog(@"Generated ellipse (ratio=%.2f, %zu vertices, %zu triangles, method=%ld, grid=%dx%d)",
          axisRatio, vertices.size(), indices.size() / 3, (long)method, cols, rows);
    
    return YES;
}

- (void)updateViewportSize:(vector_uint2)size {
    _viewportSize = size;
}

- (id<MTLBuffer>)vertexBuffer {
    return _vertexBuffer;
}

- (id<MTLBuffer>)indexBuffer {
    return _indexBuffer;
}

- (NSUInteger)indexCount {
    return _indexBuffer ? (_indexBuffer.length / sizeof(uint32_t)) : 0;
}

- (NSUInteger)instanceCount {
    return _gridParams.cols * _gridParams.rows;
}

- (GridParams)gridParams {
    return _gridParams;
}

- (simd_float4x4)viewProjectionMatrix {
    return _viewProjectionMatrix;
}

- (vector_uint2)viewportSize {
    return _viewportSize;
}

- (const std::vector<Vertex>&)currentVertices {
    return _currentVertices;
}

- (const std::vector<uint32_t>&)currentIndices {
    return _currentIndices;
}

// MARK: - Private Helpers

- (std::vector<uint32_t>)triangulateVertices:(const std::vector<Vertex>&)vertices
                                       method:(TriangulationMethod)method
                          shouldHandleConcave:(bool)shouldHandleConcave {
    std::vector<Vertex> mutableVerts = vertices;
    std::vector<uint32_t> indices;
    
    switch (method) {
        case TriangulationMethodEarClipping:
            indices = Triangulation::EarClippingTriangulation(mutableVerts);
            break;
            
        case TriangulationMethodEarClippingMinDiagonalPQ:
            indices = Triangulation::EarClippingTriangulation_MinDiagonalPQ(mutableVerts);
            break;
            
        case TriangulationMethodEarClippingTriangulator:
            indices = Triangulation::EarClippingTriangulation_Triangulator(mutableVerts);
            break;
            
        case TriangulationMethodMinimumWeight:
            indices = Triangulation::MinimumWeightTriangulation(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodCentroidFan:
            indices = Triangulation::CentroidFanTriangulation(mutableVerts);
            break;
            
        case TriangulationMethodGreedyMaxArea:
            indices = Triangulation::GreedyMaxAreaTriangulation(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodStrip:
            indices = Triangulation::StripTriangulation(mutableVerts);
            break;
            
        case TriangulationMethodMaxMinArea:
            indices = Triangulation::MaxMinAreaTriangulation(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodMinMaxArea:
            indices = Triangulation::MinMaxAreaTriangulation(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodConstrainedDelaunay:
            indices = Triangulation::ConstrainedDelaunayTriangulation(mutableVerts);
            break;
    }
    
    return indices;
}

- (void)uploadVertices:(const std::vector<Vertex>&)vertices
               indices:(const std::vector<uint32_t>&)indices {
    
    _vertexBuffer = [_device newBufferWithBytes:vertices.data()
                                         length:vertices.size() * sizeof(Vertex)
                                        options:MTLResourceStorageModeShared];
    
    _indexBuffer = [_device newBufferWithBytes:indices.data()
                                        length:indices.size() * sizeof(uint32_t)
                                       options:MTLResourceStorageModeShared];
}

- (void)setupOrthographicProjection {
    simd_float4x4 viewMatrix = createLookAtRhs((simd_float3){0, 0, -3},
                                               (simd_float3){0, 0, 0},
                                               (simd_float3){0, 1, 0});
    
    simd_float4x4 projMatrix = makeOrthoRhs(-1.0f, 1.0f,   // left, right
                                            -1.0f, 1.0f,   // bottom, top
                                             0.0f, 10.0f); // near, far
    
    _viewProjectionMatrix = simd_mul(projMatrix, viewMatrix);
}

- (void)setupInstanceGridWithCols:(uint32_t)cols rows:(uint32_t)rows {
    // Fill the viewport with a grid of instances
    // NDC space is [-1, 1] in both X and Y
    
    // Compute actual geometry bounds
    float minX = FLT_MAX, maxX = -FLT_MAX;
    float minY = FLT_MAX, maxY = -FLT_MAX;
    for (const auto& v : _currentVertices) {
        minX = std::min(minX, v.position.x);
        maxX = std::max(maxX, v.position.x);
        minY = std::min(minY, v.position.y);
        maxY = std::max(maxY, v.position.y);
    }
    const float geomWidth = maxX - minX;
    const float geomHeight = maxY - minY;
    const float geomAspect = (geomHeight > 0.0001f) ? (geomWidth / geomHeight) : 1.0f;
    
    // Add padding around edges and between instances
    const float edgePadding = 0.00f;
    const float instancePadding = 0.00f;
    
    // Available space after edge padding
    const float availableWidth = 2.0f - 2.0f * edgePadding;
    const float availableHeight = 2.0f - 2.0f * edgePadding;
    
    // Total padding between instances
    const float totalGapX = instancePadding * (cols - 1);
    const float totalGapY = instancePadding * (rows - 1);
    
    // Base cell size
    const float baseCellWidth = (availableWidth - totalGapX) / cols;
    const float baseCellHeight = (availableHeight - totalGapY) / rows;
    
    // Adjust cell dimensions to match geometry aspect ratio
    float cellWidth, cellHeight;
    if (geomAspect > 1.0f) {
        // Geometry is wider than tall
        cellWidth = baseCellWidth;
        cellHeight = baseCellWidth / geomAspect;
    } else {
        // Geometry is taller than wide (or square)
        cellHeight = baseCellHeight;
        cellWidth = baseCellHeight * geomAspect;
    }
    
    // Scale factor: geometry spans geomWidth x geomHeight, fit into cell
    const float scaleX = cellWidth / geomWidth;
    const float scaleY = cellHeight / geomHeight;
    const float shapeScale = std::min(scaleX, scaleY) * 0.9f; // 90% for breathing room
    
    // Origin is bottom-left of the grid in NDC
    const float originX = -1.0f + edgePadding + cellWidth * 0.5f;
    const float originY = -1.0f + edgePadding + cellHeight * 0.5f;
    
    _gridParams = (GridParams){
        .cols = cols,
        .rows = rows,
        .cellSize = {cellWidth + instancePadding, cellHeight + instancePadding},
        .origin = {originX, originY},
        .scale = shapeScale
    };
}

@end

