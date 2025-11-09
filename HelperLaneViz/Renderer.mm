//
//  Renderer.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#include <MetalKit/MetalKit.h>

#import "AssetLoader.h"
#import "MathUtilities.h"
#import "Renderer.h"
#import "ShaderTypes.h"
#import "Triangulation.h"
#import "GeometryFactory.h"
#import "SVGLoader.h"
#include "TileMetrics.h"
#include "CGALTriangulator.h"

#include "RenderPipelines.h"
#include "GridOverlay.h"
#include "TriangulationMetrics.h"

#import <iostream>
#import <fstream>

#include <map>

@implementation Renderer {
    id<MTLDevice> _device;
    id<MTLCommandQueue> _cq;
    MTKView *_view;

    RenderPipelines *_pipes;
    GridOverlay     *_grid;
    GridParams _gridParams;

    id<MTLBuffer> _vb, _ib;

    vector_uint2 _vp;
    simd_float4x4 _VP;
}

- (instancetype)initWithMetalKitView:(MTKView *)mtkView {
    if (!(self = [super init])) return nil;

    _device = mtkView.device;
    _view   = mtkView;
    _view.clearColor = MTLClearColorMake(0,0,0,1);
    _view.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
    _view.clearDepth = 1.0;

    _pipes = [[RenderPipelines alloc] initWithDevice:_device view:_view];
    _grid  = [[GridOverlay alloc] initWithPSO:_pipes.gridPSO noDepth:_pipes.noDepthState];
    _gridParams = [_grid createGridParamsForSegmentCount:3 andOrigin:simd_make_float2(0, 0) andScale:0.4];

    _cq = [_device newCommandQueue];

    // Load one mesh
    std::vector<Vertex> V; std::vector<uint32_t> I;
    AssetLoader::LoadPositionsAndIndicesFromObj(_device, @"/Users/robi/Downloads/ellipsoid_oblate_cap_lowOutMWT.obj", V, I);
    _vb = [_device newBufferWithBytes:V.data() length:V.size()*sizeof(Vertex) options:MTLResourceStorageModeShared];
    _ib = [_device newBufferWithBytes:I.data() length:I.size()*sizeof(uint32_t) options:MTLResourceStorageModeShared];

    // View-projection
    simd_float4x4 viewMat = createLookAtRhs((simd_float3){0,0,3}, (simd_float3){0,0,0}, (simd_float3){0,1,0});
    simd_float4x4 projMat = makePerspective(M_PI_4, 1, 0.1, 40);
    _VP = simd_mul(projMat, viewMat);

    return self;
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    _vp = (vector_uint2){ (uint32_t)size.width, (uint32_t)size.height };
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> cb = [_cq commandBuffer];
    MTLRenderPassDescriptor *rp = view.currentRenderPassDescriptor;
    if (!rp) { [cb commit]; return; }

    // Optional: experiment with tiling
    rp.tileWidth  = 32;
    rp.tileHeight = 32;

    FrameConstants fc{ .viewProjectionMatrix = _VP, .viewPortSize = _vp };

    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:_pipes.mainPSO];
    [enc setDepthStencilState:_pipes.depthState];
    [enc setCullMode:MTLCullModeBack];
    [enc setTriangleFillMode:MTLTriangleFillModeFill];

    [enc setVertexBytes:&fc length:sizeof(fc) atIndex:VertexInputIndexFrameConstants];
    [enc setVertexBuffer:_vb offset:0 atIndex:0];
    [enc setVertexBytes:&_gridParams length:sizeof(_gridParams) atIndex:2];

    [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                    indexCount:_ib.length/sizeof(uint32_t)
                     indexType:MTLIndexTypeUInt32
                   indexBuffer:_ib
             indexBufferOffset:0];

    // ---- Grid overlay (improved) ----
    // Force fill, disable depth, full viewport, no culling
    MTLViewport vp; vp.originX=0; vp.originY=0; vp.width=view.drawableSize.width; vp.height=view.drawableSize.height; vp.znear=0; vp.zfar=1;
    [enc setViewport:vp];
    [enc setCullMode:MTLCullModeNone];
    [enc setTriangleFillMode:MTLTriangleFillModeFill];
    [enc setDepthStencilState:_pipes.noDepthState];

    [_grid drawWithEncoder:enc drawableSize:view.drawableSize]; // uses PSO=_pipes.gridPSO

    [enc endEncoding];
    [cb presentDrawable:view.currentDrawable];
    [cb commit];
}
@end
