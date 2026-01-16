# HelperLaneViz

A macOS Metal-based visualization and benchmarking tool for analyzing GPU tile-based rendering performance of different polygon triangulation methods.

## Setup

### Prerequisites
- macOS 15.1 or later
- Xcode with Metal support
- Git (for submodules)

### Building

1. Clone the repository:
```bash
git clone <repository-url>
cd HelperLaneViz
```

2. Initialize git submodules:
```bash
git submodule update --init --recursive
```

3. Open the project in Xcode:
```bash
open HelperLaneViz.xcodeproj
```

4. Build and run (⌘R) or select Product → Run

### Initial Setup - Required User Action

**Important**: The project includes a placeholder SVG path and **will not run properly until that path points to an SVG file in your system.** To make the project work load an SVG file via code first:
1. Open `HelperLaneViz/Rendering/Renderer.mm`
2. Find line 49: `NSString *defaultSVGPath = @"Path to your SVG file";`
3. Replace with your SVG file path:
   ```objc
   NSString *defaultSVGPath = @"/path/to/your/file.svg";
   ```
4. Rebuild and run the project

## Features

### Visualization Modes
- **Helper Lane**: Visualizes GPU helper lane invocations (tile-based rendering)
- **Wireframe**: Shows triangle edges
- **Overdraw**: Highlights pixel overdraw areas
- **Print Friendly**: Black and white visualization
- **Simple Texture**: Samples from a 2048x2048 procedural texture

### Geometry Options
- **Shape Type**: Load SVG files or generate ellipses/circles
- **Triangulation Methods**: 11 different algorithms 6 of which is maintained:
  - Earcut (with/without edge flips)
  - Constrained Delaunay (with/without edge flips)
  - Greedy Max Area
  - Minimum Weight
- **MeshOptimizer**: Optional post-processing optimization (vertex cache, overdraw, vertex fetch)

### Rendering Settings
- **Tile Size**: 16x16 or 32x32 pixels
- **MSAA**: Off, 2x, or 4x
- **Instance Grid**: Configure columns and rows for instanced rendering
- **Window Size**: Adjustable resolution
- **Helper Texture**: Toggle 2048x2048 procedural texture binding

### Measurements

#### GPU Metrics
- **Helper Invocation Summary**: Total helper lane invocations and ratio
- **Overdraw Summary**: Pixel overdraw count and ratio
- **GPU Frametimes**: Records frames, displays mean/median/stddev

#### CPU Metrics
- **Tile Stats**: Triangle distribution across tiles
  - Triangles per tile (mean, median, P95)
  - Tiles per triangle (mean, median, P95)
  - Total edge length
  - Binning cost index

#### Benchmarking
- **Benchmark All Methods**: Tests all triangulation methods on current geometry
- **Benchmark Folder**: Batch processes all SVG files in a folder with all methods

#### Data Export
- **Print Summary for Table**: Prints formatted metrics to console (tab-separated) for easy copy-paste into spreadsheets
