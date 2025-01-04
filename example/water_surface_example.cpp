// Copyright (c) [2025] [Initial_Equationor]
// [WaterSurface] is licensed under Mulan PubL v2.
// You can use this software according to the terms and conditions of the Mulan PubL v2.
// You may obtain a copy of Mulan PubL v2 at:
// http://license.coscl.org.cn/MulanPubL-2.0
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PubL v2 for more details.
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <stdio.h>
#include <SDL.h>
#include "water_surface.hpp"
#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

#if !SDL_VERSION_ATLEAST(2,0,17)
#error This backend requires SDL 2.0.17+ because of SDL_RenderGeometry() function
#endif

bool drawTriangles = true;
void RenderTriangles(SDL_Renderer* renderer, WaterSurface::TriangleNet const& net) {
    if (!drawTriangles) return;
    SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, 0xff);
    for (int i = 0; i < net.nVerts; ++i) {
        for (int j = 0; j < net.verts[i].nNeighbours; ++j) {
            int idx = net.verts[i].iNeighbours[j];
            SDL_RenderDrawLine(renderer, net.verts[i].position.x, net.verts[i].position.y,
                               net.verts[idx].position.x, net.verts[idx].position.y);
        }
    }
} // void RenderTriangles

float voronoiMargin = 2.5f;
bool drawVoronoi = true;
void RenderVoronoi(SDL_Renderer* renderer, WaterSurface::TriangleNet const& net) {
    if (!drawVoronoi) return;
    SDL_SetRenderDrawColor(renderer, 0, 0xff, 0, 0xff);
    auto voronoi = WaterSurface::CalculateVoronoi(net, voronoiMargin);
    for (int i = 0; i < voronoi.size(); ++i) {
        std::vector<SDL_FPoint> polygon;
        int nVerts = voronoi[i].nVerts;
        polygon.reserve(nVerts + 1);
        for (int j = 0; j < nVerts; ++j) {
            polygon.push_back({ voronoi[i].verts[j].x, voronoi[i].verts[j].y });
        }
        polygon.push_back({ voronoi[i].verts[0].x, voronoi[i].verts[0].y });
        SDL_RenderDrawLinesF(renderer, polygon.data(), polygon.size());
    }
} // void RenderVoronoi

bool drawBezier = true;
int bezierSegments = 15;
void RenderBezier(SDL_Renderer* renderer, WaterSurface::TriangleNet const& net) {
    if (!drawBezier) return;
    SDL_SetRenderDrawColor(renderer, 0, 0, 0xff, 0xff);
    SDL_SetRenderDrawColor(renderer, 0, 0xff, 0, 0xff);
    auto voronoi = WaterSurface::CalculateVoronoi(net, voronoiMargin);
    for (int i = 0; i < voronoi.size(); ++i) {
        auto curve = BezierCurve(voronoi[i], bezierSegments);
//        std::vector<SDL_FPoint> polygon;
        int nVerts = voronoi[i].nVerts * bezierSegments;
        std::vector<SDL_Vertex> verts;
        int indices[3 * nVerts];
        verts.reserve(nVerts + 1);
        const SDL_Color color = { 91, 200, 0xff, 0xff };
        verts.push_back({
            SDL_FPoint{ net.verts[i].position.x, net.verts[i].position.y },
            color, SDL_FPoint{ 0 }
        });
        for (int j = 0; j < nVerts; ++j) {
            verts.push_back({
                SDL_FPoint{ curve[j].x, curve[j].y },
                color, SDL_FPoint{ 0 }
            });
            indices[3 * j] = 0;
            indices[3 * j + 1] = j + 1;
            indices[3 * j + 2] = (j + 1) % nVerts + 1;
        }
        SDL_RenderGeometry(renderer, nullptr, verts.data(), nVerts + 1, indices, 3 * nVerts);
//        polygon.reserve(nVerts + 1);
//        polygon.push_back({ curve.back().x, curve.back().y });
//        for (int j = 0; j < nVerts; ++j) {
//            polygon.push_back({ curve[j].x, curve[j].y });
//        }
//        SDL_RenderDrawLinesF(renderer, polygon.data(), polygon.size());
    }
} // void RenderBezier

float elastic = 2.5;
void UpdatePhysics(WaterSurface::TriangleNet &net) {
    static Uint32 stopwatch = SDL_GetTicks();
    Uint32 now = SDL_GetTicks();
    float elapse = static_cast<float >(now - stopwatch) / 1000;
    stopwatch = now;
    if (elapse > 0.1)
        return;
    WaterSurface::UpdatePhysics(net, elapse, elastic);
}

// Main code
int main(int, char**)
{
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("water surface example",
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, window_flags);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return -1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr)
    {
        SDL_Log("Error creating SDL_Renderer!");
        return -1;
    }
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();
    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);
    // Load Fonts
    ImFont* font = io.Fonts->AddFontFromFileTTF("assets/AlibabaPuHuiTi-3-55-Regular.ttf", 22.0f, nullptr, io.Fonts->GetGlyphRangesChineseFull());
    IM_ASSERT(font != nullptr);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
//    ImVec4 clear_color = ImVec4(0.f, 0.f, 0.f, 1.00f);
    ImVec4 clear_color = ImVec4(1.f, 1.f, 1.f, 1.f);

    auto points = WaterSurface::RandomPoints({ 350, 50, WINDOW_HEIGHT-100, WINDOW_WIDTH-400 }, 64);
    WaterSurface::TriangleNet net = WaterSurface::Triangulate(points);
    WaterSurface::FirstMove(net);
    // Main loop
    bool done = false;
    while (!done)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                done = true;
            }
        }
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED)
        {
            SDL_Delay(10);
            continue;
        }
        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        {
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);
            ImGui::Begin("Water Surface Parameters");
            ImGui::BeginGroup();
            ImGui::Checkbox("draw triangles", &drawTriangles);
            ImGui::SliderFloat("voronoi margin", &voronoiMargin, 0, 15.f);
            ImGui::Checkbox("draw voronoi", &drawVoronoi);
            ImGui::Checkbox("draw bezier", &drawBezier);
            if (drawBezier) {
                ImGui::SliderInt("bezier segments", &bezierSegments, 1, 30);
            }
            ImGui::SliderFloat("elastic", &elastic, 0, 5.f);
            ImGui::EndGroup();
            ImGui::End();
        }
        UpdatePhysics(net);
        // Rendering
        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColor(renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
        SDL_RenderClear(renderer);
        RenderBezier(renderer, net);
        RenderTriangles(renderer, net);
        RenderVoronoi(renderer, net);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    // Cleanup
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
