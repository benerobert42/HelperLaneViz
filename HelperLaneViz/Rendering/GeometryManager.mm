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
            indices = Triangulation::EarClipping(mutableVerts);
            break;
            
        case TriangulationMethodEarClippingTriangulator:
            indices = Triangulation::EarClippingMapbox(mutableVerts);
            break;
            
        case TriangulationMethodEarClippingTriangulatorFlipped:
            indices = Triangulation::EarClippingMapboxWithEdgeFlips(mutableVerts);
            break;
            
        case TriangulationMethodCentroidFan:
            indices = Triangulation::CentroidFan(mutableVerts);
            break;
            
        case TriangulationMethodStrip:
            indices = Triangulation::Strip(mutableVerts);
            break;
            
        case TriangulationMethodGreedyMaxArea:
            indices = Triangulation::GreedyMaxArea(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodMinimumWeight:
            indices = Triangulation::MinimumWeight(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodMaxMinArea:
            indices = Triangulation::MaxMinArea(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodMinMaxArea:
            indices = Triangulation::MinMaxArea(mutableVerts, shouldHandleConcave);
            break;
            
        case TriangulationMethodConstrainedDelaunay:
            indices = Triangulation::ConstrainedDelaunay(mutableVerts);
            break;
            
        case TriangulationMethodConstrainedDelaunayFlipped:
            indices = Triangulation::ConstrainedDelaunayWithEdgeFlips(mutableVerts);
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
    _instanceGridCols = cols;
    _instanceGridRows = rows;
    [self updateInstanceGridWithCols:cols rows:rows];
}

- (void)updateInstanceGridWithCols:(uint32_t)cols rows:(uint32_t)rows {
    float minX = FLT_MAX;
    float maxX = -FLT_MAX;
    float minY = FLT_MAX;
    float maxY = -FLT_MAX;

    for (const auto& v : _currentVertices) {
        minX = std::min(minX, v.position.x);
        maxX = std::max(maxX, v.position.x);
        minY = std::min(minY, v.position.y);
        maxY = std::max(maxY, v.position.y);
    }
    const float geomWidth = maxX - minX;
    const float geomHeight = maxY - minY;
    
    // Equal cell sizes - divide NDC space evenly
    const float edgePadding = 0.05f; // Small padding to keep instances inside frame
    const float availableWidth = 2.0f - 2.0f * edgePadding;
    const float availableHeight = 2.0f - 2.0f * edgePadding;
    
    const float cellSize = std::min(availableWidth / cols, availableHeight / rows);
    
    // Scale to fit geometry in cell while maintaining aspect ratio
    const float scaleX = (geomWidth > 0.0001f) ? (cellSize * 0.95f / geomWidth) : 1.0f;
    const float scaleY = (geomHeight > 0.0001f) ? (cellSize * 0.95f / geomHeight) : 1.0f;
    const float shapeScale = std::min(scaleX, scaleY); // Fit geometry inside cell
    
    // Grid spans from -1+padding to 1-padding, centered
    const float gridWidth = cellSize * cols;
    const float gridHeight = cellSize * rows;
    const float gridStartX = -1.0f + edgePadding + (availableWidth - gridWidth) * 0.5f;
    const float gridStartY = -1.0f + edgePadding + (availableHeight - gridHeight) * 0.5f;

    _gridParams = (GridParams){
        .cols = cols,
        .rows = rows,
        .cellSize = {cellSize, cellSize},
        .origin = {gridStartX, gridStartY},
        .scale = shapeScale
    };
}

@end

