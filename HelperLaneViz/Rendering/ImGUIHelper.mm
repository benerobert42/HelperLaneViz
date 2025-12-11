//
//  ImGUIHelper.mm
//  HelperLaneViz
//
//  Created for ImGUI integration
//

#import "ImGUIHelper.h"

// Include ImGUI headers
#include "../../external/imgui/imgui.h"
#include "../../external/imgui/backends/imgui_impl_metal.h"
#include "../../external/imgui/backends/imgui_impl_osx.h"

@interface ImGUIHelper ()
@property (nonatomic, strong) id<MTLDevice> device;
@property (nonatomic, weak) MTKView *view;
@end

@implementation ImGUIHelper

- (instancetype)initWithDevice:(id<MTLDevice>)device view:(MTKView *)view {
    self = [super init];
    if (self) {
        _device = device;
        _view = view;
        
        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        (void)io;
        
        // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        
        // Setup Dear ImGui style
        ImGui::StyleColorsDark();
        
        // Setup Platform/Renderer backends
        ImGui_ImplMetal_Init(_device);
        ImGui_ImplOSX_Init(view);
    }
    return self;
}

- (void)shutdown {
    ImGui_ImplMetal_Shutdown();
    ImGui_ImplOSX_Shutdown();
    ImGui::DestroyContext();
}

- (void)newFrameWithRenderPassDescriptor:(MTLRenderPassDescriptor *)renderPassDescriptor {
    ImGuiIO& io = ImGui::GetIO();
    
    // Update display size
    io.DisplaySize.x = _view.bounds.size.width;
    io.DisplaySize.y = _view.bounds.size.height;
    
    // Update framebuffer scale
    CGFloat framebufferScale = _view.window.screen.backingScaleFactor ?: [NSScreen mainScreen].backingScaleFactor;
    io.DisplayFramebufferScale = ImVec2(framebufferScale, framebufferScale);
    
    // Start the Dear ImGui frame
    ImGui_ImplMetal_NewFrame(renderPassDescriptor);
    ImGui_ImplOSX_NewFrame(_view);
    ImGui::NewFrame();
}

- (void)renderWithCommandBuffer:(id<MTLCommandBuffer>)commandBuffer
                  commandEncoder:(id<MTLRenderCommandEncoder>)commandEncoder {
    // Render ImGui
    ImGui::Render();
    ImDrawData* drawData = ImGui::GetDrawData();
    ImGui_ImplMetal_RenderDrawData(drawData, commandBuffer, commandEncoder);
}

+ (void *)getImGuiContext {
    return ImGui::GetCurrentContext();
}

- (void)showExampleWindow {
    // Simple example window
    ImGui::Begin("ImGUI Example");
    ImGui::Text("Hello! ImGUI is working!");
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                1000.0f / ImGui::GetIO().Framerate, 
                ImGui::GetIO().Framerate);
    
    static int counter = 0;
    if (ImGui::Button("Click Me!")) {
        counter++;
    }
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);
    
    static bool show_demo = false;
    if (ImGui::Button("Show Demo Window")) {
        show_demo = !show_demo;
    }
    if (show_demo) {
        ImGui::ShowDemoWindow(&show_demo);
    }
    
    ImGui::End();
}

@end

