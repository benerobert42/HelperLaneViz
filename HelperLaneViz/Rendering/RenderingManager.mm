//
//  RenderingManager.mm
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//

#import "RenderingManager.h"
#import "GeometryManager.h"
#import "MetricsComputer.h"
#import "PipelineFactory.h"
#import "GridOverlay.h"
#import "DebugRenderer.h"

#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>

@implementation RenderingManager {
    id<MTLDevice> _device;
    MTKView *_view;
    
    // Render pipelines
    id<MTLRenderPipelineState> _mainPipeline;
    id<MTLRenderPipelineState> _wireframePipeline;
    id<MTLRenderPipelineState> _overdrawPipeline;
    id<MTLRenderPipelineState> _printFriendlyPipeline;
    id<MTLRenderPipelineState> _gridOverlayPipeline;
    
    // Grid overlay
    GridOverlay *_gridOverlay;
    
    // Helper lane engagement
    id<MTLTexture> _helperLaneTexture;
    id<MTLSamplerState> _pointSampler;
    
    // Debug renderer (handles all ImGUI UI)
    DebugRenderer *_debugRenderer;
    
    // Track sample count to detect changes
    NSInteger _lastSampleCount;
}

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view {
    if (!(self = [super init])) return nil;
    
    _device = device;
    _view = view;
    _lastSampleCount = _view.sampleCount;
    
    [self setupPipelines];
    [self setupGridOverlay];
    [self setupHelperLaneResources];
    
    // Initialize DebugRenderer (handles all ImGUI UI)
    _debugRenderer = [[DebugRenderer alloc] initWithDevice:_device view:view];
    
    return self;
}

- (void)setupPipelines {
    NSError *error = nil;
    id<MTLLibrary> library = [_device newDefaultLibrary];
    
    _mainPipeline = MakeMainPipelineState(_device, _view, library, &error);
    NSAssert(_mainPipeline, @"Failed to create main pipeline: %@", error);
    
    _wireframePipeline = MakeWireframePipelineState(_device, _view, library, &error);
    NSAssert(_wireframePipeline, @"Failed to create wireframe pipeline: %@", error);
    
    _overdrawPipeline = MakeOverdrawPipelineState(_device, _view, library, &error);
    NSAssert(_overdrawPipeline, @"Failed to create overdraw pipeline: %@", error);
    
    _printFriendlyPipeline = MakePrintFriendlyPipelineState(_device, _view, library, &error);
    NSAssert(_printFriendlyPipeline, @"Failed to create print friendly pipeline: %@", error);
    
    _gridOverlayPipeline = MakeGridOverlayPipelineState(_device, _view, library, &error);
    NSAssert(_gridOverlayPipeline, @"Failed to create grid overlay pipeline: %@", error);
}

- (void)setupGridOverlay {
    _gridOverlay = [[GridOverlay alloc] initWithPipelineState:_gridOverlayPipeline];
}

- (void)updatePipelinesForCurrentSampleCount {
    // Check if sample count has changed
    NSInteger currentSampleCount = _view.sampleCount;
    if (currentSampleCount != _lastSampleCount) {
        _lastSampleCount = currentSampleCount;
        [self setupPipelines];
        [self setupGridOverlay];  // GridOverlay holds pipeline reference, needs update
    }
}

- (void)setupHelperLaneResources {
    // Create 1x1 dummy texture for helper lane engagement
    MTLTextureDescriptor *helperTexDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:1 height:1 mipmapped:NO];
    _helperLaneTexture = [_device newTextureWithDescriptor:helperTexDesc];
    uint8_t white = 255;
    [_helperLaneTexture replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&white bytesPerRow:1];
    
    MTLSamplerDescriptor *samplerDesc = [MTLSamplerDescriptor new];
    samplerDesc.minFilter = MTLSamplerMinMagFilterNearest;
    samplerDesc.magFilter = MTLSamplerMinMagFilterNearest;
    _pointSampler = [_device newSamplerStateWithDescriptor:samplerDesc];
}

// MARK: - Public Interface

- (DebugRenderer *)debugRenderer {
    return _debugRenderer;
}
    

- (void)encodeGeometryWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                          geometry:(GeometryManager *)geometry
                              mode:(VisualizationMode)mode {
    
    // Ensure pipelines match current sample count
    [self updatePipelinesForCurrentSampleCount];
    
    // Common setup for all modes
    FrameConstants frameConstants = {
        .viewProjectionMatrix = geometry.viewProjectionMatrix,
        .viewPortSize = geometry.viewportSize
    };
    GridParams gridParams = geometry.gridParams;
    
    [encoder setVertexBuffer:geometry.vertexBuffer offset:0 atIndex:VertexInputIndexVertices];
    [encoder setVertexBytes:&frameConstants length:sizeof(frameConstants) atIndex:VertexInputIndexFrameConstants];
    [encoder setVertexBytes:&gridParams length:sizeof(gridParams) atIndex:VertexInputGridParams];
    
    const NSUInteger indexCount = geometry.indexCount;
    const NSUInteger instanceCount = geometry.instanceCount;
    
    // Common render state (no depth, no culling for 2D)
    [encoder setCullMode:MTLCullModeNone];
    
    // Mode-specific setup
    switch (mode) {
        case VisualizationModeHelperLane:
            [encoder setRenderPipelineState:_mainPipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeFill];
            [encoder setFragmentTexture:_helperLaneTexture atIndex:0];
            [encoder setFragmentSamplerState:_pointSampler atIndex:0];
            break;
            
        case VisualizationModeWireframe:
            [encoder setRenderPipelineState:_wireframePipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeLines];
            break;
            
        case VisualizationModeOverdraw:
            [encoder setRenderPipelineState:_overdrawPipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeFill];
            break;
            
        case VisualizationModePrintFriendly:
            [encoder setRenderPipelineState:_printFriendlyPipeline];
            [encoder setTriangleFillMode:MTLTriangleFillModeLines];
            break;
    }
    
    [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:indexCount
                         indexType:MTLIndexTypeUInt32
                       indexBuffer:geometry.indexBuffer
                 indexBufferOffset:0
                     instanceCount:instanceCount];
}

- (void)encodeGridOverlayWithEncoder:(id<MTLRenderCommandEncoder>)encoder
                         drawableSize:(CGSize)drawableSize {
    // Ensure pipelines match current sample count
    [self updatePipelinesForCurrentSampleCount];
    
    MTLViewport viewport = {
        .originX = 0,
        .originY = 0,
        .width = drawableSize.width,
        .height = drawableSize.height,
        .znear = 0,
        .zfar = 1
    };
    [encoder setViewport:viewport];
    
    uint32_t tileSize = [_debugRenderer tileSizePx];
    [_gridOverlay drawWithEncoder:encoder
                         tileSize:tileSize
                     drawableSize:drawableSize];
}

// MARK: - Accessors (delegated to DebugRenderer)

- (BOOL)showGridOverlay {
    return [_debugRenderer showGridOverlay];
}

- (void)setShowGridOverlay:(BOOL)show {
    [_debugRenderer setShowGridOverlay:show];
}

- (VisualizationMode)visualizationMode {
    return [_debugRenderer visualizationMode];
}

- (void)setVisualizationMode:(VisualizationMode)mode {
    [_debugRenderer setVisualizationMode:mode];
}

- (int)msaaSampleCount {
    return [_debugRenderer msaaSampleCount];
}

- (uint32_t)tileSizePx {
    return [_debugRenderer tileSizePx];
}

- (void)setTileSizePx:(uint32_t)tileSize {
    [_debugRenderer setTileSizePx:tileSize];
}

- (void)setCurrentSVGPath:(NSString *)path
                    method:(TriangulationMethod)method
                      cols:(uint32_t)cols
                      rows:(uint32_t)rows
          bezierDeviation:(float)bezierDev {
    [_debugRenderer setCurrentSVGPath:path method:method cols:cols rows:rows bezierDeviation:bezierDev];
}

- (void*)gpuFrameTimer {
    return [_debugRenderer gpuFrameTimer];
}

@end

