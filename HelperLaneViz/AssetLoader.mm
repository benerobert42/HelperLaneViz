//
//  AssetLoader.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import <MetalKit/MetalKit.h>

#include "AssetLoader.h"

MTKMesh* AssetLoader::LoadModelFromObj(id<MTLDevice> device, NSString *filename) {
    NSError *error = nil;
    MTKMeshBufferAllocator *allocator = [[MTKMeshBufferAllocator alloc] initWithDevice:device];

    NSString *assetPath = [[NSBundle mainBundle] pathForResource:filename ofType:@"obj"];
    NSURL *url = [NSURL fileURLWithPath:assetPath];
    MDLVertexDescriptor *vertexDescriptor = [[MDLVertexDescriptor alloc] init];

    int offset = 0;
    vertexDescriptor.attributes[0] = [[MDLVertexAttribute alloc] initWithName:MDLVertexAttributePosition
                                                                       format:MDLVertexFormatFloat3
                                                                       offset:0
                                                                  bufferIndex:0];
    offset += sizeof(vector_float3);

    vertexDescriptor.attributes[1] = [[MDLVertexAttribute alloc] initWithName:MDLVertexAttributeNormal
                                                                       format:MDLVertexFormatFloat3
                                                                       offset:16
                                                                  bufferIndex:0];

    vertexDescriptor.layouts[0] = [[MDLVertexBufferLayout alloc] initWithStride:32];

    vertexDescriptor.attributes[0].name = MDLVertexAttributePosition;
    vertexDescriptor.attributes[1].name = MDLVertexAttributeNormal;


    // Load asset
    MDLAsset *asset = [[MDLAsset alloc] initWithURL:url vertexDescriptor:vertexDescriptor bufferAllocator:allocator];

    NSArray<MDLMesh *> *meshes = [asset childObjectsOfClass:[MDLMesh class]];
    MDLMesh *mdlMesh = meshes.firstObject;
    MTKMesh *mtkMesh = [[MTKMesh alloc] initWithMesh:mdlMesh device:device error:&error];

    return mtkMesh;
}
