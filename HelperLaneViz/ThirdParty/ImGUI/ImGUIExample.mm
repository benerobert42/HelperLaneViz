//
//  ImGUIExample.mm
//  HelperLaneViz
//
//  Example implementation showing how to create ImGUI UI elements
//

#import "ImGUIExample.h"
#include "../../external/imgui/imgui.h"

void ShowExampleImGUIWindow(void) {
    ImGui::Begin("Settings and information");
    
    // Display some text
    ImGui::Text("Hello from ImGUI!");
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                1000.0f / ImGui::GetIO().Framerate, 
                ImGui::GetIO().Framerate);
    
    // Create a button
    static int counter = 0;
    if (ImGui::Button("Click Me!")) {
        counter++;
    }
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);
    
    // Create a slider
    static float value = 0.5f;
    ImGui::SliderFloat("Value", &value, 0.0f, 1.0f);
    
    // Create a checkbox
    static bool enabled = true;
    ImGui::Checkbox("Enabled", &enabled);
    
    // Create a color picker
    static ImVec4 color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    ImGui::ColorEdit3("Color", (float*)&color);
    
    ImGui::End();
    
    // You can create multiple windows
    static bool show_demo = false;
    if (ImGui::Button("Show Demo Window")) {
        show_demo = !show_demo;
    }
    if (show_demo) {
        ImGui::ShowDemoWindow(&show_demo);
    }
}


