//
//  AssetLoader.h
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//
#pragma once

#include <MetalKit/MetalKit.h>
#include <simd/simd.h>

namespace AssetLoader {
    MTKMesh* LoadModelFromObj(id<MTLDevice> device, NSString *filename);
}
