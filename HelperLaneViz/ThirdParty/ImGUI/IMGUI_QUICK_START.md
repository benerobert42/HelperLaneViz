# ImGUI Quick Start

ImGUI is now integrated into your project! Here's how to use it.

## Basic Usage

### 1. Create a Simple Window

In your render loop (after `ImGui::NewFrame()` is called), you can create UI:

```cpp
#include "../../external/imgui/imgui.h"

ImGui::Begin("My Window");
ImGui::Text("Hello, World!");
ImGui::End();
```

### 2. Common Widgets

```cpp
// Button
if (ImGui::Button("Click Me")) {
    // Handle click
}

// Slider
static float value = 0.5f;
ImGui::SliderFloat("Value", &value, 0.0f, 1.0f);

// Checkbox
static bool enabled = true;
ImGui::Checkbox("Enabled", &enabled);

// Color Picker
static ImVec4 color = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
ImGui::ColorEdit3("Color", (float*)&color);

// Input Text
static char text[128] = "Hello";
ImGui::InputText("Text", text, IM_ARRAYSIZE(text));
```

### 3. Control Your Renderer Settings

You can add UI controls in `Renderer.mm` in the `drawInMTKView` method:

```objective-c
// After: [_imguiHelper newFrameWithRenderPassDescriptor:renderPassDescriptor];

#include "../../external/imgui/imgui.h"

ImGui::Begin("Renderer Controls");
ImGui::Checkbox("Show Grid", &_showGridOverlay);
// Heatmap removed; grid overlay now renders grid lines only.
ImGui::Checkbox("Show Overdraw", &_showOverdraw);
ImGui::End();
```

### 4. Show Demo Window

To see all available widgets:

```cpp
static bool show_demo = true;
ImGui::ShowDemoWindow(&show_demo);
```

## Where to Add Your UI Code

The best place to add your ImGUI UI code is in `Renderer.mm`, in the `drawInMTKView` method, right after this line:

```objective-c
[_imguiHelper newFrameWithRenderPassDescriptor:renderPassDescriptor];
```

You can replace or modify the `showExampleWindow` call with your own UI code.

## Example: Complete UI Window

```cpp
ImGui::Begin("Settings");

// Renderer settings
ImGui::Checkbox("Grid Overlay", &_showGridOverlay);
// Heatmap removed; grid overlay now renders grid lines only.

// Slider example
static float scale = 1.0f;
ImGui::SliderFloat("Scale", &scale, 0.1f, 2.0f);

// Button example
if (ImGui::Button("Reset")) {
    scale = 1.0f;
}

// Display info
ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

ImGui::End();
```

## Tips

- All ImGUI functions are available - just `#include "../../external/imgui/imgui.h"`
- The context is already initialized, so you can use ImGUI functions directly
- Input is handled automatically by the OSX backend
- Multiple windows are supported - just call `ImGui::Begin()` multiple times

## Resources

- [ImGUI Documentation](https://github.com/ocornut/imgui)
- [ImGUI Examples](https://github.com/ocornut/imgui/tree/master/examples/example_apple_metal)


