//
//  AssetLoader.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//
#pragma once

#include "ShaderTypes.h"

#include <MetalKit/MetalKit.h>
#include <simd/simd.h>

namespace AssetLoader {
    MTKMesh* LoadModelFromObj(id<MTLDevice> device, NSString *filename);
    bool LoadPositionsAndIndicesFromObj(id<MTLDevice> device,
                                        NSString* filename,
                                        std::vector<Vertex>& outVertices,
                                        std::vector<uint32_t>& outIndices);
}
