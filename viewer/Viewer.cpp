#include "Grid.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <numbers>

bool DrawArrow(SDL_Renderer* renderer, MassMotion::Cell cell, SDL_FPoint center, float cellSize)
{
    if (cell == MassMotion::Cell::None)
    {
        return true;
    }

    constexpr float diagonal = 1.0f / std::numbers::sqrt2_v<float>;
    constexpr SDL_FPoint directions[] = {
        {0.0f, 0.0f},           // None
        {0.0f, -1.0f},          // North
        {diagonal, -diagonal},  // NorthEast
        {1.0f, 0.0f},           // East
        {diagonal, diagonal},   // SouthEast
        {0.0f, 1.0f},           // South
        {-diagonal, diagonal},  // SouthWest
        {-1.0f, 0.0f},          // West
        {-diagonal, -diagonal}, // NorthWest
    };
    const SDL_FPoint direction = directions[static_cast<std::size_t>(cell)];
    const float halfLength = cellSize * 0.26f;
    const float headLength = cellSize * 0.16f;
    const float headWidth = cellSize * 0.12f;
    const SDL_FPoint tip = {center.x + direction.x * halfLength, center.y + direction.y * halfLength};
    const SDL_FPoint headBase = {tip.x - direction.x * headLength, tip.y - direction.y * headLength};
    const SDL_FPoint points[] = {
        {center.x - direction.x * halfLength, center.y - direction.y * halfLength},   tip,
        {headBase.x - direction.y * headWidth, headBase.y + direction.x * headWidth}, tip,
        {headBase.x + direction.y * headWidth, headBase.y - direction.x * headWidth},
    };
    return SDL_RenderLines(renderer, points, 5);
}

bool DrawGrid(SDL_Renderer* renderer, const MassMotion::GridState& state, std::size_t goalX, std::size_t goalY)
{
    int outputWidth = 0;
    int outputHeight = 0;
    if (!SDL_GetRenderOutputSize(renderer, &outputWidth, &outputHeight))
    {
        return false;
    }

    constexpr float padding = 24.0f;
    const float cellSize =
        std::min((static_cast<float>(outputWidth) - padding * 2.0f) / static_cast<float>(state.width),
                 (static_cast<float>(outputHeight) - padding * 2.0f) / static_cast<float>(state.height));
    if (cellSize <= 0.0f)
    {
        return true;
    }

    const float gridWidth = cellSize * static_cast<float>(state.width);
    const float gridHeight = cellSize * static_cast<float>(state.height);
    const float originX = (static_cast<float>(outputWidth) - gridWidth) * 0.5f;
    const float originY = (static_cast<float>(outputHeight) - gridHeight) * 0.5f;
    const float inset = cellSize * 0.08f;
    const SDL_FRect goalRect = {
        originX + static_cast<float>(goalX) * cellSize + inset,
        originY + static_cast<float>(goalY) * cellSize + inset,
        cellSize - inset * 2.0f,
        cellSize - inset * 2.0f,
    };
    if (!SDL_SetRenderDrawColor(renderer, 50, 145, 100, 255) || !SDL_RenderFillRect(renderer, &goalRect) ||
        !SDL_SetRenderDrawColor(renderer, 58, 70, 84, 255))
    {
        return false;
    }
    for (std::size_t x = 0; x <= state.width; ++x)
    {
        const float lineX = originX + static_cast<float>(x) * cellSize;
        if (!SDL_RenderLine(renderer, lineX, originY, lineX, originY + gridHeight))
        {
            return false;
        }
    }
    for (std::size_t y = 0; y <= state.height; ++y)
    {
        const float lineY = originY + static_cast<float>(y) * cellSize;
        if (!SDL_RenderLine(renderer, originX, lineY, originX + gridWidth, lineY))
        {
            return false;
        }
    }
    if (!SDL_SetRenderDrawColor(renderer, 164, 205, 225, 255))
    {
        return false;
    }
    for (std::size_t y = 0; y < state.height; ++y)
    {
        for (std::size_t x = 0; x < state.width; ++x)
        {
            const SDL_FPoint center = {originX + (static_cast<float>(x) + 0.5f) * cellSize,
                                       originY + (static_cast<float>(y) + 0.5f) * cellSize};
            if (!DrawArrow(renderer, state.cells[y * state.width + x], center, cellSize))
            {
                return false;
            }
        }
    }
    return true;
}

int RunViewer()
{
    const std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
        SDL_CreateWindow("MassMotion Grid Viewer", 960, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY),
        SDL_DestroyWindow);
    if (!window)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create window: %s", SDL_GetError());
        return EXIT_FAILURE;
    }

    const std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer(
        SDL_CreateRenderer(window.get(), nullptr), SDL_DestroyRenderer);
    if (!renderer)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create renderer: %s", SDL_GetError());
        return EXIT_FAILURE;
    }

    MassMotion::Grid grid(20, 14);
    constexpr std::size_t goalX = 10;
    constexpr std::size_t goalY = 7;
    grid.SetGoal(goalX, goalY);
    const MassMotion::GridState state = grid.GetState();

    while (true)
    {
        if (!SDL_SetRenderDrawColor(renderer.get(), 24, 28, 34, 255) || !SDL_RenderClear(renderer.get()) ||
            !DrawGrid(renderer.get(), state, goalX, goalY) || !SDL_RenderPresent(renderer.get()))
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not render frame: %s", SDL_GetError());
            return EXIT_FAILURE;
        }

        // Redraw on events and sleep while idle.
        SDL_Event event;
        if (!SDL_WaitEvent(&event))
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not wait for events: %s", SDL_GetError());
            return EXIT_FAILURE;
        }

        if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED ||
            (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE))
        {
            return EXIT_SUCCESS;
        }
    }
}

int main(int, char**)
{
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not initialize SDL: %s", SDL_GetError());
        SDL_Quit();
        return EXIT_FAILURE;
    }

    const int result = RunViewer();
    SDL_Quit();
    return result;
}
