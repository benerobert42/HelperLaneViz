//
//  GridOverlay.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import <MetalKit/MetalKit.h>
#import <simd/simd.h>

#include "ShaderTypes.h"

typedef struct {
    vector_uint2 tileSize;
    vector_uint2 framebuffer;
    float        lineWidth;
    simd_float4  lineColor;
} GridUniformsCPU;

@interface GridOverlay : NSObject
- (instancetype)initWithPSO:(id<MTLRenderPipelineState>)pso noDepth:(id<MTLDepthStencilState>)noDepth;
- (GridParams)createGridParamsForSegmentCount:(uint32_t)segmentCount
                                    andOrigin:(simd_float2)origin
                                     andScale:(float)scale;
- (void)drawWithEncoder:(id<MTLRenderCommandEncoder>)enc drawableSize:(CGSize)drawableSize;
@end
