//
//  AssetLoader.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import <MetalKit/MetalKit.h>

#include "AssetLoader.h"
#include "ShaderTypes.h"

#include <vector>

MTKMesh* AssetLoader::LoadModelFromObj(id<MTLDevice> device, NSString *filename) {
    MDLVertexDescriptor *vertexDescriptor = [[MDLVertexDescriptor alloc] init];
    vertexDescriptor.attributes[0] = [[MDLVertexAttribute alloc] initWithName:MDLVertexAttributePosition
                                                                       format:MDLVertexFormatFloat3
                                                                       offset:0
                                                                  bufferIndex:0];

    vertexDescriptor.layouts[0] = [[MDLVertexBufferLayout alloc] initWithStride:32];
    vertexDescriptor.attributes[0].name = MDLVertexAttributePosition;

    NSString *assetPath = [[NSBundle mainBundle] pathForResource:filename ofType:@"obj"];
    NSURL *url = [NSURL fileURLWithPath:assetPath];
    MTKMeshBufferAllocator *allocator = [[MTKMeshBufferAllocator alloc] initWithDevice:device];
    MDLAsset *asset = [[MDLAsset alloc] initWithURL:url vertexDescriptor:vertexDescriptor bufferAllocator:allocator];
    NSArray<MDLMesh *> *meshes = [asset childObjectsOfClass:[MDLMesh class]];
    MDLMesh *mdlMesh = meshes.firstObject;
    NSError *error = nil;
    MTKMesh *mtkMesh = [[MTKMesh alloc] initWithMesh:mdlMesh device:device error:&error];

    return mtkMesh;
}

static NSURL *ResolveOBJURL(NSString *filename) {
    if ([filename hasPrefix:@"/"] || [filename containsString:@"/"] || [filename hasPrefix:@"~"]) {
        return [NSURL fileURLWithPath:[filename stringByExpandingTildeInPath]];
    }
    NSString *name = [[filename lowercaseString] hasSuffix:@".obj"] ? [filename stringByDeletingPathExtension] : filename;
    NSString *path = [[NSBundle mainBundle] pathForResource:name ofType:@"obj"];
    return [NSURL fileURLWithPath:path];
}

bool AssetLoader::LoadPositionsAndIndicesFromObj(id<MTLDevice> device,
                                                 NSString *filename,
                                                 std::vector<Vertex> &outVertices,
                                                 std::vector<uint32_t> &outIndices) {
    outVertices.clear();
    outIndices.clear();

    NSURL *url = ResolveOBJURL(filename);

    MDLVertexDescriptor *vd = [MDLVertexDescriptor new];
    vd.attributes[0] = [[MDLVertexAttribute alloc] initWithName:MDLVertexAttributePosition
                                                         format:MDLVertexFormatFloat3
                                                         offset:0
                                                    bufferIndex:0];
    vd.layouts[0] = [[MDLVertexBufferLayout alloc] initWithStride:sizeof(vector_float3)];

    MTKMeshBufferAllocator *alloc = [[MTKMeshBufferAllocator alloc] initWithDevice:device];
    MDLAsset *asset = [[MDLAsset alloc] initWithURL:url vertexDescriptor:vd bufferAllocator:alloc];
    NSArray<MTKMesh *> *meshes = [MTKMesh newMeshesFromAsset:asset device:device sourceMeshes:nil error:nil];

    for (MTKMesh *m in meshes) {
        const NSUInteger stride = m.vertexDescriptor.layouts[0].stride ?: sizeof(vector_float3);
        MTKMeshBuffer *vb = m.vertexBuffers.firstObject;
        const uint8_t *vptr = (const uint8_t *)vb.buffer.contents + vb.offset;

        const size_t base = outVertices.size();
        outVertices.resize(base + m.vertexCount);
        for (NSUInteger i = 0; i < m.vertexCount; ++i) {
            outVertices[base + i].position = *(const vector_float3 *)(vptr + i * stride);
        }

        for (MTKSubmesh *s in m.submeshes) {
            const uint8_t *iptr = (const uint8_t *)s.indexBuffer.buffer.contents + s.indexBuffer.offset;
            const size_t old = outIndices.size();
            outIndices.resize(old + s.indexCount);

            if (s.indexType == MTLIndexTypeUInt16) {
                const uint16_t *src = (const uint16_t *)iptr;
                for (NSUInteger k = 0; k < s.indexCount; ++k) outIndices[old + k] = (uint32_t)src[k] + (uint32_t)base;
            } else {
                const uint32_t *src = (const uint32_t *)iptr;
                for (NSUInteger k = 0; k < s.indexCount; ++k) outIndices[old + k] = src[k] + (uint32_t)base;
            }
        }
    }
    return true;
}
