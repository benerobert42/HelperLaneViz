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
    vector_float3 normal;
} Vertex;

typedef struct
{
    simd_float4x4 viewProjectionMatrix;
    simd_uint2 viewPortSize;
} FrameConstants;

#endif
