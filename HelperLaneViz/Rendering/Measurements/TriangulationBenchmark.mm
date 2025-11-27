//
//  TriangulationBenchmark.mm
//  HelperLaneViz
//

#import "TriangulationBenchmark.h"
#import <cmath>
#import <numeric>
#import <algorithm>

namespace Benchmark {

// =============================================================================
// MARK: - Method Names
// =============================================================================

const char* methodName(TriangulationMethod method) {
    switch (method) {
        case TriangulationMethod::MinimumWeight:       return "MinimumWeight";
        case TriangulationMethod::CentroidFan:         return "CentroidFan";
        case TriangulationMethod::GreedyMaxArea:       return "GreedyMaxArea";
        case TriangulationMethod::Strip:               return "Strip";
        case TriangulationMethod::MaxMinArea:          return "MaxMinArea";
        case TriangulationMethod::MinMaxArea:          return "MinMaxArea";
        case TriangulationMethod::ConstrainedDelaunay: return "ConstrainedDelaunay";
        default:                                       return "Unknown";
    }
}

// =============================================================================
// MARK: - Statistics Helpers
// =============================================================================

namespace {

struct Stats {
    double mean = 0.0;
    double min = 0.0;
    double max = 0.0;
    double stdDev = 0.0;
};

Stats computeStats(const std::vector<double>& values) {
    Stats stats;
    if (values.empty()) return stats;
    
    stats.min = *std::min_element(values.begin(), values.end());
    stats.max = *std::max_element(values.begin(), values.end());
    stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    
    double sumSquaredDiff = 0.0;
    for (double v : values) {
        double diff = v - stats.mean;
        sumSquaredDiff += diff * diff;
    }
    stats.stdDev = std::sqrt(sumSquaredDiff / values.size());
    
    return stats;
}

} // anonymous namespace

// =============================================================================
// MARK: - Benchmark Runner
// =============================================================================

BenchmarkResults runBenchmark(id<BenchmarkFrameExecutor> executor,
                              const BenchmarkConfig& config) {
    BenchmarkResults results;
    results.config = config;
    
    const int methodCount = static_cast<int>(TriangulationMethod::COUNT);
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║              TRIANGULATION BENCHMARK STARTING                        ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════╣\n");
    printf("║  Scenes: %zu                                                          \n", config.scenes.size());
    printf("║  Methods: %d                                                          \n", methodCount);
    printf("║  Warmup frames: %d                                                    \n", config.warmupFrames);
    printf("║  Measure frames: %d                                                   \n", config.measureFrames);
    printf("╚══════════════════════════════════════════════════════════════════════╝\n\n");
    
    for (size_t sceneIdx = 0; sceneIdx < config.scenes.size(); ++sceneIdx) {
        const SceneConfig& scene = config.scenes[sceneIdx];
        
        printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        printf("  Scene %zu: %d vertices, %.2fx%.2f ellipse, %dx%d instances\n",
               sceneIdx + 1, scene.vertexCount, scene.semiMajorAxis, scene.semiMinorAxis,
               scene.instanceGridSize, scene.instanceGridSize);
        printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        
        SceneResult sceneResult;
        sceneResult.config = scene;
        
        for (int m = 0; m < methodCount; ++m) {
            TriangulationMethod method = static_cast<TriangulationMethod>(m);
            
            printf("  Testing %-20s ... ", methodName(method));
            fflush(stdout);
            
            MethodResult methodResult;
            methodResult.method = method;
            
            // Prepare scene with this triangulation method
            double triangulationTime = [executor prepareSceneWithVertexCount:scene.vertexCount
                                                               semiMajorAxis:scene.semiMajorAxis
                                                               semiMinorAxis:scene.semiMinorAxis
                                                         triangulationMethod:m
                                                            instanceGridSize:scene.instanceGridSize];
            methodResult.triangulationTimeMs = triangulationTime * 1000.0;
            
            // Get mesh data for metrics
            std::vector<Vertex> vertices;
            std::vector<uint32_t> indices;
            [executor getCurrentMeshVertices:&vertices indices:&indices];
            
            // Compute mesh metrics
            methodResult.meshMetrics = TriangulationMetrics::computeMeshMetrics(
                vertices, indices, config.framebufferSize, simd_int2{(int)config.tileSize, (int)config.tileSize});
            
            // Warmup frames
            for (int f = 0; f < config.warmupFrames; ++f) {
                [executor executeFrameAndMeasureGPUTime];
            }
            
            // Measurement frames
            std::vector<double> gpuTimes;
            gpuTimes.reserve(config.measureFrames);
            
            for (int f = 0; f < config.measureFrames; ++f) {
                double gpuTime = [executor executeFrameAndMeasureGPUTime];
                if (gpuTime >= 0) {
                    gpuTimes.push_back(gpuTime * 1000.0); // Convert to ms
                }
            }
            
            // Compute GPU time statistics
            if (!gpuTimes.empty()) {
                Stats stats = computeStats(gpuTimes);
                methodResult.gpuTimeMs_Mean = stats.mean;
                methodResult.gpuTimeMs_Min = stats.min;
                methodResult.gpuTimeMs_Max = stats.max;
                methodResult.gpuTimeMs_StdDev = stats.stdDev;
            }
            
            sceneResult.methodResults.push_back(methodResult);
            
            printf("GPU: %.3f ms (±%.3f), Triangles: %zu, BCI: %.2f\n",
                   methodResult.gpuTimeMs_Mean,
                   methodResult.gpuTimeMs_StdDev,
                   methodResult.meshMetrics.triangleCount,
                   methodResult.meshMetrics.binningCostIndex);
        }
        
        results.sceneResults.push_back(sceneResult);
    }
    
    return results;
}

// =============================================================================
// MARK: - Results Output
// =============================================================================

void BenchmarkResults::printSummary() const {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                                    BENCHMARK RESULTS SUMMARY                                                 ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n");
    
    for (const auto& scene : sceneResults) {
        printf("┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────┐\n");
        printf("│ Scene: %d verts, %.1fx%.1f, %dx%d instances (%d total triangles drawn)                                       \n",
               scene.config.vertexCount,
               scene.config.semiMajorAxis, scene.config.semiMinorAxis,
               scene.config.instanceGridSize, scene.config.instanceGridSize,
               scene.config.instanceGridSize * scene.config.instanceGridSize * (scene.config.vertexCount - 2));
        printf("├──────────────────────────────────────────────────────────────────────────────────────────────────────────────┤\n");
        printf("│ %-20s │ %8s │ %8s │ %10s │ %10s │ %8s │ %12s │\n",
               "Method", "GPU(ms)", "±StdDev", "Triangles", "TileOvrlp", "BCI", "EdgeLength");
        printf("├──────────────────────────────────────────────────────────────────────────────────────────────────────────────┤\n");
        
        // Find best GPU time for highlighting
        double bestGpuTime = std::numeric_limits<double>::max();
        for (const auto& r : scene.methodResults) {
            if (r.gpuTimeMs_Mean < bestGpuTime) bestGpuTime = r.gpuTimeMs_Mean;
        }
        
        for (const auto& r : scene.methodResults) {
            const char* marker = (r.gpuTimeMs_Mean <= bestGpuTime * 1.05) ? "◀" : " ";
            printf("│ %-20s │ %7.3f%s │ %8.3f │ %10zu │ %10.0f │ %8.2f │ %12.4f │\n",
                   methodName(r.method),
                   r.gpuTimeMs_Mean, marker,
                   r.gpuTimeMs_StdDev,
                   r.meshMetrics.triangleCount,
                   r.meshMetrics.totalTileOverlaps,
                   r.meshMetrics.binningCostIndex,
                   r.meshMetrics.totalEdgeLength);
        }
        printf("└──────────────────────────────────────────────────────────────────────────────────────────────────────────────┘\n\n");
    }
}

std::string BenchmarkResults::toCSV() const {
    std::string csv;
    
    // Header
    csv += "Scene,Vertices,SemiMajor,SemiMinor,Instances,Method,";
    csv += "GPU_Mean_ms,GPU_StdDev_ms,GPU_Min_ms,GPU_Max_ms,";
    csv += "Triangles,Edges,EdgeLength,TotalArea,";
    csv += "TileOverlaps,NonEmptyTiles,BCI,";
    csv += "TriPerTile_Mean,TriPerTile_P95,TilesPerTri_Mean,TilesPerTri_P95,";
    csv += "TriangulationTime_ms\n";
    
    int sceneNum = 1;
    for (const auto& scene : sceneResults) {
        for (const auto& r : scene.methodResults) {
            char line[1024];
            snprintf(line, sizeof(line),
                     "%d,%d,%.2f,%.2f,%d,%s,"
                     "%.4f,%.4f,%.4f,%.4f,"
                     "%zu,%zu,%.6f,%.2f,"
                     "%.0f,%zu,%.3f,"
                     "%.2f,%.2f,%.2f,%.2f,"
                     "%.4f\n",
                     sceneNum,
                     scene.config.vertexCount,
                     scene.config.semiMajorAxis,
                     scene.config.semiMinorAxis,
                     scene.config.instanceGridSize,
                     methodName(r.method),
                     r.gpuTimeMs_Mean, r.gpuTimeMs_StdDev, r.gpuTimeMs_Min, r.gpuTimeMs_Max,
                     r.meshMetrics.triangleCount,
                     r.meshMetrics.uniqueEdgeCount,
                     r.meshMetrics.totalEdgeLength,
                     r.meshMetrics.totalTriangleArea,
                     r.meshMetrics.totalTileOverlaps,
                     r.meshMetrics.nonEmptyTileCount,
                     r.meshMetrics.binningCostIndex,
                     r.meshMetrics.trianglesPerTile_Mean,
                     r.meshMetrics.trianglesPerTile_P95,
                     r.meshMetrics.tilesPerTriangle_Mean,
                     r.meshMetrics.tilesPerTriangle_P95,
                     r.triangulationTimeMs);
            csv += line;
        }
        ++sceneNum;
    }
    
    return csv;
}

} // namespace Benchmark

