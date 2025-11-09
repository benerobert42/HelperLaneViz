//
//  RenderPipelines.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 11. 08..
//
#import "RenderPipelines.h"

@implementation RenderPipelines {
    id<MTLDevice> _device;
}

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView*)view {
    if ((self = [super init])) {
        _device = device;
        id<MTLLibrary> lib = [_device newDefaultLibrary];

        auto makeDesc = ^MTLRenderPipelineDescriptor* {
            MTLRenderPipelineDescriptor *d = [MTLRenderPipelineDescriptor new];
            d.colorAttachments[0].pixelFormat        = view.colorPixelFormat;
            d.depthAttachmentPixelFormat             = view.depthStencilPixelFormat;   // ✅ match pass
            d.stencilAttachmentPixelFormat           = MTLPixelFormatInvalid;
            return d;
        };

        // Main
        {
            MTLRenderPipelineDescriptor *d = makeDesc();
            d.vertexFunction   = [lib newFunctionWithName:@"vertexShader"];
            d.fragmentFunction = [lib newFunctionWithName:@"fragmentShader"];
            _mainPSO = [_device newRenderPipelineStateWithDescriptor:d error:nil];
//            _mainPSO.label = @"MainPSO";
        }

        // Overdraw
        {
            MTLRenderPipelineDescriptor *d = makeDesc();
            d.vertexFunction   = [lib newFunctionWithName:@"vertexShader"];
            d.fragmentFunction = [lib newFunctionWithName:@"overdrawFragment"];
            auto *ca = d.colorAttachments[0];
            ca.blendingEnabled = YES;
            ca.rgbBlendOperation   = MTLBlendOperationAdd;
            ca.alphaBlendOperation = MTLBlendOperationAdd;
            ca.sourceRGBBlendFactor   = MTLBlendFactorOne;
            ca.destinationRGBBlendFactor = MTLBlendFactorOne;
            ca.sourceAlphaBlendFactor   = MTLBlendFactorOne;
            ca.destinationAlphaBlendFactor = MTLBlendFactorOne;
            _overdrawPSO = [_device newRenderPipelineStateWithDescriptor:d error:nil];
//            _overdrawPSO.label = @"OverdrawPSO";
        }

        // Grid overlay
        {
            MTLRenderPipelineDescriptor *d = makeDesc();
            d.vertexFunction   = [lib newFunctionWithName:@"tilegrid_vs"];
            d.fragmentFunction = [lib newFunctionWithName:@"tilegrid_fs"];
            auto *ca = d.colorAttachments[0];
            ca.blendingEnabled = YES;
            ca.rgbBlendOperation   = MTLBlendOperationAdd;
            ca.alphaBlendOperation = MTLBlendOperationAdd;
            ca.sourceRGBBlendFactor   = MTLBlendFactorSourceAlpha;
            ca.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            ca.sourceAlphaBlendFactor   = MTLBlendFactorSourceAlpha;
            ca.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            _gridPSO = [_device newRenderPipelineStateWithDescriptor:d error:nil];
//            _gridPSO.label = @"GridPSO";
        }

        // Depth states
        {
            auto ds = [MTLDepthStencilDescriptor new];
            ds.depthWriteEnabled = YES;
            ds.depthCompareFunction = MTLCompareFunctionLessEqual;
            _depthState = [_device newDepthStencilStateWithDescriptor:ds];

            auto nds = [MTLDepthStencilDescriptor new];
            nds.depthWriteEnabled = NO;
            nds.depthCompareFunction = MTLCompareFunctionAlways;
            _noDepthState = [_device newDepthStencilStateWithDescriptor:nds];
        }
    }
    return self;
}
@end
