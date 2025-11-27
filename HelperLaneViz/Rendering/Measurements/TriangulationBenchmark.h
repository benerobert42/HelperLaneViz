//
//  TriangulationBenchmark.h
//  HelperLaneViz
//
//  Automated benchmarking system for comparing triangulation methods.
//

#pragma once

#import <Metal/Metal.h>
#import <simd/simd.h>
#include <vector>
#include <string>
#include "ShaderTypes.h"
#include "TriangulationMetrics.h"

// =============================================================================
// MARK: - Benchmark Frame Executor Protocol (Objective-C)
// =============================================================================

@protocol BenchmarkFrameExecutor <NSObject>

- (double)prepareSceneWithVertexCount:(int)vertexCount
                        semiMajorAxis:(float)a
                        semiMinorAxis:(float)b
                  triangulationMethod:(int)method
                     instanceGridSize:(uint32_t)gridSize;

- (double)executeFrameAndMeasureGPUTime;

- (void)getCurrentMeshVertices:(std::vector<Vertex>*)outVertices
                       indices:(std::vector<uint32_t>*)outIndices;

@end

// =============================================================================
// MARK: - C++ Benchmark Types
// =============================================================================

namespace Benchmark {

// -----------------------------------------------------------------------------
// Triangulation Methods
// -----------------------------------------------------------------------------

enum class TriangulationMethod {
    MinimumWeight,
    CentroidFan,
    GreedyMaxArea,
    Strip,
    MaxMinArea,
    MinMaxArea,
    ConstrainedDelaunay,
    
    COUNT
};

const char* methodName(TriangulationMethod method);

// -----------------------------------------------------------------------------
// Shape Types
// -----------------------------------------------------------------------------

enum class ShapeType {
    Circle,              // a/b = 1
    Ellipse_1_2,         // a/b = 1/2 (width = 2 * height)
    Ellipse_1_10,        // a/b = 1/10 (elongated)
    
    COUNT
};

const char* shapeName(ShapeType shape);

/// Get semi-axes for a shape type. Returns (semiMajorAxis, semiMinorAxis).
/// Shapes are normalized so the larger axis = 1.0
std::pair<float, float> shapeAxes(ShapeType shape);

// -----------------------------------------------------------------------------
// Scene Configuration
// -----------------------------------------------------------------------------

struct SceneConfig {
    ShapeType shape = ShapeType::Circle;
    int vertexCount = 100;
    uint32_t instanceGridSize = 3;  // NxN grid
    
    // Computed from shape type
    float semiMajorAxis() const;
    float semiMinorAxis() const;
    
    // Total instances
    uint32_t totalInstances() const { return instanceGridSize * instanceGridSize; }
    
    // Human-readable description
    std::string description() const;
};

// -----------------------------------------------------------------------------
// Benchmark Configuration
// -----------------------------------------------------------------------------

struct BenchmarkConfig {
    std::vector<SceneConfig> scenes;
    int warmupFrames = 10;
    int measureFrames = 100;
    simd_int2 framebufferSize = {1920, 1080};
    uint32_t tileSize = 32;
    
    /// Generate the standard test matrix:
    /// 3 shapes × 4 vertex counts × 3 instance counts = 36 scenes
    static BenchmarkConfig standardTestMatrix();
    
    /// Generate a quick test with fewer combinations
    static BenchmarkConfig quickTest();
};

// -----------------------------------------------------------------------------
// Results
// -----------------------------------------------------------------------------

struct MethodResult {
    TriangulationMethod method;
    TriangulationMetrics::MeshMetrics meshMetrics;
    
    double gpuTimeMs_Mean = 0.0;
    double gpuTimeMs_Min = 0.0;
    double gpuTimeMs_Max = 0.0;
    double gpuTimeMs_StdDev = 0.0;
    double triangulationTimeMs = 0.0;
};

struct SceneResult {
    SceneConfig config;
    std::vector<MethodResult> methodResults;
};

struct BenchmarkResults {
    BenchmarkConfig config;
    std::vector<SceneResult> sceneResults;
    
    void printSummary() const;
    void printDetailedReport() const;
    std::string toCSV() const;
};

// -----------------------------------------------------------------------------
// Benchmark Runner
// -----------------------------------------------------------------------------

BenchmarkResults runBenchmark(id<BenchmarkFrameExecutor> executor,
                              const BenchmarkConfig& config);

} // namespace Benchmark
