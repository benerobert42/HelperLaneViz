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
// MARK: - Name Lookups
// =============================================================================

const char* methodName(TriangulationMethod method) {
    switch (method) {
        case TriangulationMethod::EarClipping:         return "EarClipping";
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

const char* shapeName(ShapeType shape) {
    switch (shape) {
        case ShapeType::Circle:       return "Circle";
        case ShapeType::Ellipse_1_2:  return "Ellipse_1:2";
        case ShapeType::Ellipse_1_10: return "Ellipse_1:10";
        default:                      return "Unknown";
    }
}

std::pair<float, float> shapeAxes(ShapeType shape) {
    switch (shape) {
        case ShapeType::Circle:
            return {1.0f, 1.0f};
        case ShapeType::Ellipse_1_2:
            // a/b = 1/2 means b = 2a. Normalize so larger axis = 1.0
            // semiMajor (horizontal) = 1.0, semiMinor (vertical) = 0.5
            return {1.0f, 0.5f};
        case ShapeType::Ellipse_1_10:
            // a/b = 1/10 means b = 10a. Very elongated vertically.
            // semiMajor (horizontal) = 1.0, semiMinor (vertical) = 0.1
            return {1.0f, 0.1f};
        default:
            return {1.0f, 1.0f};
    }
}

// =============================================================================
// MARK: - Scene Configuration
// =============================================================================

float SceneConfig::semiMajorAxis() const {
    return shapeAxes(shape).first;
}

float SceneConfig::semiMinorAxis() const {
    return shapeAxes(shape).second;
}

std::string SceneConfig::description() const {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s, %d verts, %dx%d instances",
             shapeName(shape), vertexCount, instanceGridCols, instanceGridRows);
    return std::string(buffer);
}

// =============================================================================
// MARK: - Benchmark Configuration Presets
// =============================================================================

BenchmarkConfig BenchmarkConfig::standardTestMatrix() {
    BenchmarkConfig config;
    
    // Shape types
    const ShapeType shapes[] = {
        ShapeType::Circle,
        ShapeType::Ellipse_1_2,
        ShapeType::Ellipse_1_10
    };
    
    // Vertex counts per object
    const int vertexCounts[] = {100, 200, 500, 1000};
    
    // Instance grid sizes (3×3=9, 5×5=25, 10×10=100)
    const uint32_t instanceGrids[] = {3, 5, 10};
    
    // Generate all combinations: 3 × 4 × 3 = 36 scenes
    for (ShapeType shape : shapes) {
        for (int vertexCount : vertexCounts) {
            for (uint32_t gridSize : instanceGrids) {
                SceneConfig scene;
                scene.shape = shape;
                scene.vertexCount = vertexCount;
                scene.instanceGridCols = gridSize;
                scene.instanceGridRows = gridSize;
                config.scenes.push_back(scene);
            }
        }
    }
    
    config.warmupFrames = 10;
    config.measureFrames = 50;
    
    return config;
}

BenchmarkConfig BenchmarkConfig::quickTest() {
    BenchmarkConfig config;
    
    // Reduced set for quick testing
    const ShapeType shapes[] = {ShapeType::Circle, ShapeType::Ellipse_1_2};
    const int vertexCounts[] = {100, 500};
    const uint32_t instanceGrids[] = {3, 5};
    
    for (ShapeType shape : shapes) {
        for (int vertexCount : vertexCounts) {
            for (uint32_t gridSize : instanceGrids) {
                SceneConfig scene;
                scene.shape = shape;
                scene.vertexCount = vertexCount;
                scene.instanceGridCols = gridSize;
                scene.instanceGridRows = gridSize;
                config.scenes.push_back(scene);
            }
        }
    }
    
    config.warmupFrames = 5;
    config.measureFrames = 20;
    
    return config;
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
    const size_t totalTests = config.scenes.size() * methodCount;
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                    TRIANGULATION BENCHMARK SUITE                             ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    printf("║  Total scenes:        %-5zu                                                  ║\n", config.scenes.size());
    printf("║  Methods per scene:   %-5d                                                  ║\n", methodCount);
    printf("║  Total test configs:  %-5zu                                                  ║\n", totalTests);
    printf("║  Warmup frames:       %-5d                                                  ║\n", config.warmupFrames);
    printf("║  Measure frames:      %-5d                                                  ║\n", config.measureFrames);
    printf("╚══════════════════════════════════════════════════════════════════════════════╝\n\n");
    
    size_t currentTest = 0;
    
    for (size_t sceneIdx = 0; sceneIdx < config.scenes.size(); ++sceneIdx) {
        const SceneConfig& scene = config.scenes[sceneIdx];
        
        printf("┌──────────────────────────────────────────────────────────────────────────────┐\n");
        printf("│ Scene %2zu/%-2zu: %-60s │\n",
               sceneIdx + 1, config.scenes.size(), scene.description().c_str());
        printf("├──────────────────────────────────────────────────────────────────────────────┤\n");
        
        SceneResult sceneResult;
        sceneResult.config = scene;
        
        for (int m = 0; m < methodCount; ++m) {
            TriangulationMethod method = static_cast<TriangulationMethod>(m);
            ++currentTest;
            
            printf("│  [%3zu/%3zu] %-20s ", currentTest, totalTests, methodName(method));
            fflush(stdout);
            
            MethodResult methodResult;
            methodResult.method = method;
            
            // Prepare scene
            double triangulationTime = [executor prepareSceneWithVertexCount:scene.vertexCount
                                                               semiMajorAxis:scene.semiMajorAxis()
                                                               semiMinorAxis:scene.semiMinorAxis()
                                                         triangulationMethod:m
                                                            instanceGridCols:scene.instanceGridCols
                                                                    gridRows:scene.instanceGridRows];
            methodResult.triangulationTimeMs = triangulationTime * 1000.0;
            
            // Get mesh data for metrics
            std::vector<Vertex> vertices;
            std::vector<uint32_t> indices;
            [executor getCurrentMeshVertices:&vertices indices:&indices];
            
            // Compute mesh metrics
            methodResult.meshMetrics = TriangulationMetrics::computeMeshMetrics(
                vertices, indices,
                config.framebufferSize,
                simd_int2{(int)config.tileSize, (int)config.tileSize});
            
            // Warmup
            for (int f = 0; f < config.warmupFrames; ++f) {
                [executor executeFrameAndMeasureGPUTime];
            }
            
            // Measurement
            std::vector<double> gpuTimes;
            gpuTimes.reserve(config.measureFrames);
            
            for (int f = 0; f < config.measureFrames; ++f) {
                double gpuTime = [executor executeFrameAndMeasureGPUTime];
                if (gpuTime >= 0) {
                    gpuTimes.push_back(gpuTime * 1000.0);
                }
            }
            
            // Statistics
            if (!gpuTimes.empty()) {
                Stats stats = computeStats(gpuTimes);
                methodResult.gpuTimeMs_Mean = stats.mean;
                methodResult.gpuTimeMs_Min = stats.min;
                methodResult.gpuTimeMs_Max = stats.max;
                methodResult.gpuTimeMs_StdDev = stats.stdDev;
            }
            
            sceneResult.methodResults.push_back(methodResult);
            
            printf("GPU: %6.3f ms (±%5.3f)  BCI: %5.2f │\n",
                   methodResult.gpuTimeMs_Mean,
                   methodResult.gpuTimeMs_StdDev,
                   methodResult.meshMetrics.binningCostIndex);
        }
        
        printf("└──────────────────────────────────────────────────────────────────────────────┘\n\n");
        
        results.sceneResults.push_back(sceneResult);
    }
    
    return results;
}

// =============================================================================
// MARK: - Results Output
// =============================================================================

void BenchmarkResults::printSummary() const {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                                         BENCHMARK RESULTS SUMMARY                                                ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n");
    
    // Group results by shape type for easier comparison
    for (int shapeIdx = 0; shapeIdx < static_cast<int>(ShapeType::COUNT); ++shapeIdx) {
        ShapeType shape = static_cast<ShapeType>(shapeIdx);
        
        printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        printf("  SHAPE: %s\n", shapeName(shape));
        printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
        
        // Filter scenes for this shape
        std::vector<const SceneResult*> shapeScenes;
        for (const auto& scene : sceneResults) {
            if (scene.config.shape == shape) {
                shapeScenes.push_back(&scene);
            }
        }
        
        if (shapeScenes.empty()) continue;
        
        // Print table header
        printf("┌─────────────────────────────┬");
        for (size_t i = 0; i < shapeScenes.size(); ++i) {
            printf("─────────────────┬");
        }
        printf("\n");
        
        printf("│ %-27s │", "Triangulation Method");
        for (const auto* scene : shapeScenes) {
            char header[32];
            snprintf(header, sizeof(header), "%dv × %dx%d",
                     scene->config.vertexCount, scene->config.instanceGridCols, scene->config.instanceGridRows);
            printf(" %-15s │", header);
        }
        printf("\n");
        
        printf("├─────────────────────────────┼");
        for (size_t i = 0; i < shapeScenes.size(); ++i) {
            printf("─────────────────┼");
        }
        printf("\n");
        
        // Print each method's results
        for (int m = 0; m < static_cast<int>(TriangulationMethod::COUNT); ++m) {
            TriangulationMethod method = static_cast<TriangulationMethod>(m);
            
            printf("│ %-27s │", methodName(method));
            
            for (const auto* scene : shapeScenes) {
                // Find this method's result
                double gpuTime = 0.0;
                for (const auto& result : scene->methodResults) {
                    if (result.method == method) {
                        gpuTime = result.gpuTimeMs_Mean;
                        break;
                    }
                }
                printf(" %7.3f ms     │", gpuTime);
            }
            printf("\n");
        }
        
        printf("└─────────────────────────────┴");
        for (size_t i = 0; i < shapeScenes.size(); ++i) {
            printf("─────────────────┴");
        }
        printf("\n\n");
    }
}

void BenchmarkResults::printDetailedReport() const {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                                         DETAILED BENCHMARK REPORT                                                ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n");
    
    for (const auto& scene : sceneResults) {
        printf("┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐\n");
        printf("│ Scene: %-100s │\n", scene.config.description().c_str());
        printf("├───────────────────────┬──────────┬──────────┬──────────┬──────────┬──────────┬───────────┬────────────────────────┤\n");
        printf("│ Method                │ GPU(ms)  │ ±StdDev  │ Min(ms)  │ Max(ms)  │ Tris     │ BCI       │ Edge Length            │\n");
        printf("├───────────────────────┼──────────┼──────────┼──────────┼──────────┼──────────┼───────────┼────────────────────────┤\n");
        
        // Find best GPU time
        double bestGpu = std::numeric_limits<double>::max();
        for (const auto& r : scene.methodResults) {
            if (r.gpuTimeMs_Mean < bestGpu) bestGpu = r.gpuTimeMs_Mean;
        }
        
        for (const auto& r : scene.methodResults) {
            const char* marker = (r.gpuTimeMs_Mean <= bestGpu * 1.05) ? "◀" : " ";
            printf("│ %-21s │ %7.3f%s │ %8.3f │ %8.3f │ %8.3f │ %8zu │ %9.2f │ %22.4f │\n",
                   methodName(r.method),
                   r.gpuTimeMs_Mean, marker,
                   r.gpuTimeMs_StdDev,
                   r.gpuTimeMs_Min,
                   r.gpuTimeMs_Max,
                   r.meshMetrics.triangleCount,
                   r.meshMetrics.binningCostIndex,
                   r.meshMetrics.totalEdgeLength);
        }
        printf("└───────────────────────┴──────────┴──────────┴──────────┴──────────┴──────────┴───────────┴────────────────────────┘\n\n");
    }
}

std::string BenchmarkResults::toCSV() const {
    std::string csv;
    
    // Header
    csv += "Shape,VertexCount,GridCols,GridRows,TotalInstances,Method,";
    csv += "GPU_Mean_ms,GPU_StdDev_ms,GPU_Min_ms,GPU_Max_ms,";
    csv += "Triangles,Edges,EdgeLength,TotalArea,";
    csv += "TileOverlaps,NonEmptyTiles,BCI,";
    csv += "TriPerTile_Mean,TriPerTile_P95,TilesPerTri_Mean,TilesPerTri_P95,";
    csv += "TriangulationTime_ms\n";
    
    for (const auto& scene : sceneResults) {
        for (const auto& r : scene.methodResults) {
            char line[1024];
            snprintf(line, sizeof(line),
                     "%s,%d,%d,%d,%d,%s,"
                     "%.4f,%.4f,%.4f,%.4f,"
                     "%zu,%zu,%.6f,%.2f,"
                     "%.0f,%zu,%.3f,"
                     "%.2f,%.2f,%.2f,%.2f,"
                     "%.4f\n",
                     shapeName(scene.config.shape),
                     scene.config.vertexCount,
                     scene.config.instanceGridCols,
                     scene.config.instanceGridRows,
                     scene.config.totalInstances(),
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
    }
    
    return csv;
}

} // namespace Benchmark
