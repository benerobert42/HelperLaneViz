//
//  ShaderTypes.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#ifndef AAPLShaderTypes_h
#define AAPLShaderTypes_h

#include <simd/simd.h>

typedef enum VertexInputIndex
{
    VertexInputIndexVertices = 0,
    VertexInputIndexFrameConstants = 1,
} VertexInputIndex;

typedef struct
{
    vector_float3 position;
} Vertex;

typedef struct
{
    simd_float4x4 viewProjectionMatrix;
    simd_uint2 viewPortSize;
} FrameConstants;

struct GridParams {
    uint32_t cols;
    uint32_t rows;
    simd_float2 cellSize; // world units or NDC scale
    simd_float2 origin; // bottom-left in world/NDC
    float scale; // optional uniform scale
};
#endif
