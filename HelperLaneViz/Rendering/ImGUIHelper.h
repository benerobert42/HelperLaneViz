//
//  ImGUIHelper.h
//  HelperLaneViz
//
//  Created for ImGUI integration
//

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>

NS_ASSUME_NONNULL_BEGIN

@class MTKView;

@interface ImGUIHelper : NSObject

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view;
- (void)shutdown;

// Call these in your render loop
- (void)newFrameWithRenderPassDescriptor:(MTLRenderPassDescriptor *)renderPassDescriptor;
- (void)renderWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                  commandEncoder:(id<MTLRenderCommandEncoder>)commandEncoder;

// Access ImGui context for creating UI elements
+ (void *)getImGuiContext;

// Helper method to show example UI (for testing)
- (void)showExampleWindow;

@end

NS_ASSUME_NONNULL_END

