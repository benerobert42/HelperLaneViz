//
//  GPUFrameTimer.h
//  HelperLaneViz
//
//  Precise end-of-pipe to end-of-pipe GPU frame time measurement.
//  Uses MTLCommandBuffer.GPUEndTime timestamps (zero overhead).
//

#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>

class GPUFrameTimer {
public:
    struct Results {
        double minMs = 0.0;
        double maxMs = 0.0;
        double avgMs = 0.0;
        double stdDevMs = 0.0;
        double p50Ms = 0.0;   // Median
        double p95Ms = 0.0;
        double p99Ms = 0.0;
        int frameCount = 0;
        
        void print() const {
            printf("\n═══════════════════════════════════════════════════\n");
            printf("  GPU Frame Time Results (%d frames)\n", frameCount);
            printf("═══════════════════════════════════════════════════\n");
            printf("  Average:   %.3f ms\n", avgMs);
            printf("  Std Dev:   %.3f ms\n", stdDevMs);
            printf("  Min:       %.3f ms\n", minMs);
            printf("  Max:       %.3f ms\n", maxMs);
            printf("  Median:    %.3f ms (p50)\n", p50Ms);
            printf("  p95:       %.3f ms\n", p95Ms);
            printf("  p99:       %.3f ms\n", p99Ms);
            printf("═══════════════════════════════════════════════════\n\n");
        }
    };
    
    using CompletionCallback = std::function<void(const Results&)>;
    
    /// Start measuring for a given number of frames
    void startMeasurement(int frameCount, CompletionCallback callback = nullptr) {
        _targetFrames = frameCount;
        _frameTimes.clear();
        _frameTimes.reserve(frameCount);
        _isActive = true;
        _callback = callback;
    }
    
    /// Record GPU execution time (call from command buffer completion handler)
    /// Returns true if measurement session is complete
    bool recordGPUEndTime(double gpuStartTime, double gpuEndTime) {
        if (!_isActive) return false;
        
        if (gpuStartTime > 0.0 && gpuEndTime > gpuStartTime) {
            double frameTimeMs = (gpuEndTime - gpuStartTime) * 1000.0;
            _frameTimes.push_back(frameTimeMs);
        }
        
        if ((int)_frameTimes.size() >= _targetFrames) {
            _isActive = false;
            Results results = computeResults();
            if (_callback) {
                _callback(results);
            }
            return true;
        }
        return false;
    }
    
    bool isActive() const { return _isActive; }
    int framesRemaining() const { return _isActive ? _targetFrames - (int)_frameTimes.size() : 0; }
    
private:
    Results computeResults() const {
        Results r;
        if (_frameTimes.empty()) return r;
        
        r.frameCount = (int)_frameTimes.size();
        
        // Copy for sorting (percentiles)
        std::vector<double> sorted = _frameTimes;
        std::sort(sorted.begin(), sorted.end());
        
        r.minMs = sorted.front();
        r.maxMs = sorted.back();
        
        // Average
        double sum = 0.0;
        for (double t : _frameTimes) sum += t;
        r.avgMs = sum / r.frameCount;
        
        // Standard deviation
        double variance = 0.0;
        for (double t : _frameTimes) {
            double diff = t - r.avgMs;
            variance += diff * diff;
        }
        r.stdDevMs = std::sqrt(variance / r.frameCount);
        
        // Percentiles
        r.p50Ms = percentile(sorted, 50);
        r.p95Ms = percentile(sorted, 95);
        r.p99Ms = percentile(sorted, 99);
        
        return r;
    }
    
    static double percentile(const std::vector<double>& sorted, int p) {
        if (sorted.empty()) return 0.0;
        double rank = (p / 100.0) * (sorted.size() - 1);
        size_t lower = (size_t)rank;
        size_t upper = std::min(lower + 1, sorted.size() - 1);
        double frac = rank - lower;
        return sorted[lower] * (1.0 - frac) + sorted[upper] * frac;
    }
    
    bool _isActive = false;
    int _targetFrames = 0;
    std::vector<double> _frameTimes;
    CompletionCallback _callback;
};

