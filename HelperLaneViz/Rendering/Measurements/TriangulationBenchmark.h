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

/// Protocol for objects that can execute benchmark frames.
/// Must be declared outside C++ namespace.
@protocol BenchmarkFrameExecutor <NSObject>

/// Prepare geometry for the given scene and triangulation method.
/// Returns the time taken for triangulation in seconds.
- (double)prepareSceneWithVertexCount:(int)vertexCount
                        semiMajorAxis:(float)a
                        semiMinorAxis:(float)b
                  triangulationMethod:(int)method
                     instanceGridSize:(uint32_t)gridSize;

/// Execute one frame and return GPU execution time in seconds.
/// Returns -1 if timing is not available.
- (double)executeFrameAndMeasureGPUTime;

/// Get current mesh vertices and indices for metric computation
- (void)getCurrentMeshVertices:(std::vector<Vertex>*)outVertices
                       indices:(std::vector<uint32_t>*)outIndices;

@end

// =============================================================================
// MARK: - C++ Benchmark Types
// =============================================================================

namespace Benchmark {

/// Available triangulation methods to benchmark
enum class TriangulationMethod {
    MinimumWeight,
    CentroidFan,
    GreedyMaxArea,
    Strip,
    MaxMinArea,
    MinMaxArea,
    ConstrainedDelaunay,
    
    COUNT  // Number of methods
};

/// Returns human-readable name for a triangulation method
const char* methodName(TriangulationMethod method);

/// Scene configuration for benchmarking
struct SceneConfig {
    int vertexCount = 100;
    float semiMajorAxis = 1.0f;
    float semiMinorAxis = 0.5f;
    uint32_t instanceGridSize = 5;
    
    static SceneConfig ellipse(int verts, float a, float b, uint32_t instances) {
        return {verts, a, b, instances};
    }
    static SceneConfig circle(int verts, float radius, uint32_t instances) {
        return {verts, radius, radius, instances};
    }
};

/// Benchmark run configuration
struct BenchmarkConfig {
    std::vector<SceneConfig> scenes;
    int warmupFrames = 10;
    int measureFrames = 100;
    simd_int2 framebufferSize = {1920, 1080};
    uint32_t tileSize = 32;
};

/// Results for a single triangulation method on a single scene
struct MethodResult {
    TriangulationMethod method;
    TriangulationMetrics::MeshMetrics meshMetrics;
    
    double gpuTimeMs_Mean = 0.0;
    double gpuTimeMs_Min = 0.0;
    double gpuTimeMs_Max = 0.0;
    double gpuTimeMs_StdDev = 0.0;
    double triangulationTimeMs = 0.0;
};

/// Results for a single scene across all triangulation methods
struct SceneResult {
    SceneConfig config;
    std::vector<MethodResult> methodResults;
};

/// Complete benchmark results
struct BenchmarkResults {
    BenchmarkConfig config;
    std::vector<SceneResult> sceneResults;
    
    void printSummary() const;
    std::string toCSV() const;
};

/// Runs the complete benchmark suite
BenchmarkResults runBenchmark(id<BenchmarkFrameExecutor> executor,
                              const BenchmarkConfig& config);

} // namespace Benchmark
