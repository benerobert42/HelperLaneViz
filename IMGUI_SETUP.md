# ImGUI Setup Guide

ImGUI has been added to your project! This guide explains how to complete the setup and start using it.

## What's Been Done

1. ✅ ImGUI has been added as a git submodule in `external/imgui/`
2. ✅ `ImGUIHelper` class created to manage ImGUI initialization and rendering
3. ✅ ImGUI integrated into the `Renderer` class
4. ✅ Example UI code provided

## Next Steps: Adding ImGUI Source Files to Xcode

You need to add the ImGUI source files to your Xcode project so they compile:

### Option 1: Add Files Manually (Recommended)

1. Open your project in Xcode
2. Right-click on the `HelperLaneViz` group in the Project Navigator
3. Select "Add Files to HelperLaneViz..."
4. Navigate to `external/imgui/` and select these files:
   - `imgui.cpp`
   - `imgui_draw.cpp`
   - `imgui_tables.cpp`
   - `imgui_widgets.cpp`
   - `backends/imgui_impl_metal.mm`
   - `backends/imgui_impl_osx.mm`
5. Make sure "Copy items if needed" is **unchecked** (they're already in the project)
6. Make sure "Create groups" is selected
7. Click "Add"

### Option 2: Configure Header Search Paths

If you prefer not to add the source files directly, you can:

1. In Xcode, select your project in the Project Navigator
2. Select the "HelperLaneViz" target
3. Go to "Build Settings"
4. Search for "Header Search Paths"
5. Add: `$(SRCROOT)/external/imgui` (recursive)

However, you'll still need to add the `.cpp` and `.mm` source files to compile them.

## Using ImGUI

### Basic Usage

The `ImGUIHelper` class is already integrated into your `Renderer`. To create UI elements, you can:

1. **Use the example window** (already enabled):
   - The `Renderer` already calls `showExampleWindow` which displays a simple demo

2. **Create your own UI**:
   ```objective-c
   // In your render loop, after ImGui::NewFrame() is called,
   // you can use ImGUI functions directly:
   
   #include "external/imgui/imgui.h"
   
   ImGui::Begin("My Window");
   ImGui::Text("Hello World!");
   if (ImGui::Button("Click Me")) {
       // Handle button click
   }
   ImGui::End();
   ```

3. **Access ImGUI from anywhere**:
   ```objective-c
   #include "external/imgui/imgui.h"
   
   // The context is already created, just use ImGUI functions
   ImGui::Begin("Settings");
   static float value = 0.5f;
   ImGui::SliderFloat("Value", &value, 0.0f, 1.0f);
   ImGui::End();
   ```

### Example: Adding UI to Control Renderer Settings

You can modify `Renderer.mm` to add UI controls:

```objective-c
// In drawInMTKView, after [_imguiHelper newFrameWithRenderPassDescriptor:renderPassDescriptor];

#include "../../external/imgui/imgui.h"

ImGui::Begin("Renderer Settings");
ImGui::Checkbox("Show Grid Overlay", &_showGridOverlay);
ImGui::Checkbox("Show Heatmap", &_showHeatmap);
ImGui::Checkbox("Show Overdraw", &_showOverdraw);
ImGui::End();
```

### Input Handling

Input is automatically handled by `ImGUIHelper` through the OSX backend. Mouse and keyboard events are forwarded to ImGUI automatically.

## Files Created

- `HelperLaneViz/Rendering/ImGUIHelper.h` - ImGUI wrapper class header
- `HelperLaneViz/Rendering/ImGUIHelper.mm` - ImGUI wrapper class implementation
- `HelperLaneViz/Rendering/ImGUIExample.h` - Example usage header (optional)
- `HelperLaneViz/Rendering/ImGUIExample.mm` - Example usage implementation (optional)

## Troubleshooting

### Build Errors About Missing ImGUI Files

If you get errors about missing ImGUI headers or undefined symbols:

1. Make sure you've added the ImGUI source files to Xcode (see "Next Steps" above)
2. Check that the header search paths include `$(SRCROOT)/external/imgui`
3. Verify the git submodule is initialized: `git submodule update --init`

### ImGUI Not Showing

- Make sure `showExampleWindow` is being called (it's already in `Renderer.mm`)
- Check that ImGUI rendering is happening after your main geometry but before ending the render encoder
- Verify the render pass descriptor is valid

## Resources

- [ImGUI Documentation](https://github.com/ocornut/imgui)
- [ImGUI Examples](https://github.com/ocornut/imgui/tree/master/examples)
- [ImGUI FAQ](https://github.com/ocornut/imgui/blob/master/docs/FAQ.md)

## Quick Start

Once you've added the source files to Xcode:

1. Build and run your project
2. You should see an "ImGUI Example" window with a button and FPS counter
3. Click "Show Demo Window" to see all ImGUI widgets
4. Start creating your own UI by modifying the code in `Renderer.mm` or creating new UI functions

Happy coding! 🎨

