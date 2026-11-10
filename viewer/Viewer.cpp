#include "Agents.h"
#include "Grid.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <vector>

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

bool DrawGrid(SDL_Renderer* renderer, const MassMotion::GridState& state, std::size_t goalX, std::size_t goalY,
              SDL_FRect& gridRect)
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
    gridRect = {originX, originY, gridWidth, gridHeight};
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
    for (std::size_t y = 0; y < state.height; ++y)
    {
        for (std::size_t x = 0; x < state.width; ++x)
        {
            if (state.obstacles[y * state.width + x])
            {
                const SDL_FRect obstacleRect = {
                    originX + static_cast<float>(x) * cellSize + inset,
                    originY + static_cast<float>(y) * cellSize + inset,
                    cellSize - inset * 2.0f,
                    cellSize - inset * 2.0f,
                };
                if (!SDL_SetRenderDrawColor(renderer, 190, 100, 70, 255) ||
                    !SDL_RenderFillRect(renderer, &obstacleRect))
                {
                    return false;
                }
                continue;
            }

            const SDL_FPoint center = {originX + (static_cast<float>(x) + 0.5f) * cellSize,
                                       originY + (static_cast<float>(y) + 0.5f) * cellSize};
            if (!SDL_SetRenderDrawColor(renderer, 164, 205, 225, 255) ||
                !DrawArrow(renderer, state.cells[y * state.width + x], center, cellSize))
            {
                return false;
            }
        }
    }
    return true;
}

bool DrawAgent(SDL_Renderer* renderer, const MassMotion::Agents& agents, std::size_t agentIndex,
               const SDL_FRect& gridRect, std::size_t gridWidth)
{
    if (gridRect.w <= 0.0f)
    {
        return true;
    }

    const float cellSize = gridRect.w / static_cast<float>(gridWidth);
    const auto position = agents.GetPosition(agentIndex);
    const SDL_FPoint center = {gridRect.x + static_cast<float>(position.x) * cellSize,
                               gridRect.y + static_cast<float>(position.y) * cellSize};
    constexpr int segments = 32;
    constexpr SDL_FColor color = {1.0f, 0.8f, 0.25f, 1.0f};
    std::array<SDL_Vertex, segments + 1> vertices{};
    std::array<int, segments * 3> indices{};
    vertices[0] = {center, color, {}};
    for (int i = 0; i < segments; ++i)
    {
        const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / segments;
        vertices[i + 1] = {
            {center.x + std::cos(angle) * cellSize * 0.28f, center.y + std::sin(angle) * cellSize * 0.28f}, color, {}};
        indices[i * 3] = 0;
        indices[i * 3 + 1] = i + 1;
        indices[i * 3 + 2] = (i + 1) % segments + 1;
    }
    if (!SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()), indices.data(),
                            static_cast<int>(indices.size())))
    {
        return false;
    }

    const auto direction = agents.GetDirection(agentIndex);
    const SDL_FPoint forward{static_cast<float>(direction.x), static_cast<float>(direction.y)};
    const SDL_FPoint side{-forward.y, forward.x};
    const auto point = [&](float along, float across)
    {
        return SDL_FPoint{center.x + (forward.x * along + side.x * across) * cellSize,
                          center.y + (forward.y * along + side.y * across) * cellSize};
    };
    constexpr SDL_FColor light{1.0f, 0.95f, 0.78f, 1.0f};
    constexpr SDL_FColor dark{0.14f, 0.16f, 0.2f, 1.0f};
    const SDL_Vertex headingVertices[] = {
        {point(0.26f, -0.03f), light, {}}, {point(0.7f, -0.03f), light, {}},  {point(0.7f, 0.03f), light, {}},
        {point(0.26f, 0.03f), light, {}},  {point(-0.05f, -0.03f), dark, {}}, {point(0.26f, -0.03f), dark, {}},
        {point(0.26f, 0.03f), dark, {}},   {point(-0.05f, 0.03f), dark, {}},
    };
    constexpr int headingIndices[] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    return SDL_RenderGeometry(renderer, nullptr, headingVertices, 8, headingIndices, 12);
}

bool DrawAgents(SDL_Renderer* renderer, const MassMotion::Agents& agents, const SDL_FRect& gridRect,
                std::size_t gridWidth)
{
    for (std::size_t i = 0; i < agents.GetCount(); ++i)
    {
        if (!DrawAgent(renderer, agents, i, gridRect, gridWidth))
        {
            return false;
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
    std::size_t goalX = 10;
    std::size_t goalY = 7;
    grid.SetGoal(goalX, goalY);
    grid.SetObstacle(2, 5);
    grid.SetObstacle(3, 5);
    grid.SetObstacle(4, 5);
    MassMotion::GridState state = grid.GetState();
    MassMotion::Agents agents;
    agents.Add(1, 1);
    agents.Add(1, 7);
    agents.Add(1, 12);

    Uint64 previousTicks = SDL_GetTicksNS();

    while (true)
    {
        const Uint64 ticks = SDL_GetTicksNS();
        // Avoid a large jump after dragging the window or resuming the application.
        const double elapsedSeconds = std::min(static_cast<double>(ticks - previousTicks) / 1.0e9, 0.1);
        previousTicks = ticks;
        agents.Update(state, elapsedSeconds);

        SDL_FRect gridRect{};
        if (!SDL_SetRenderDrawColor(renderer.get(), 24, 28, 34, 255) || !SDL_RenderClear(renderer.get()) ||
            !DrawGrid(renderer.get(), state, goalX, goalY, gridRect) ||
            !DrawAgents(renderer.get(), agents, gridRect, state.width) || !SDL_RenderPresent(renderer.get()))
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not render frame: %s", SDL_GetError());
            return EXIT_FAILURE;
        }

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED ||
                (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE))
            {
                return EXIT_SUCCESS;
            }

            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
                (event.button.button == SDL_BUTTON_LEFT || event.button.button == SDL_BUTTON_RIGHT))
            {
                SDL_FPoint mouse{};
                if (!SDL_RenderCoordinatesFromWindow(renderer.get(), event.button.x, event.button.y, &mouse.x,
                                                     &mouse.y))
                {
                    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not convert mouse coordinates: %s",
                                 SDL_GetError());
                    return EXIT_FAILURE;
                }

                if (gridRect.w <= 0.0f || gridRect.h <= 0.0f || mouse.x < gridRect.x ||
                    mouse.x >= gridRect.x + gridRect.w || mouse.y < gridRect.y || mouse.y >= gridRect.y + gridRect.h)
                {
                    continue;
                }

                const float cellSize = gridRect.w / static_cast<float>(state.width);
                const std::size_t x =
                    std::min(static_cast<std::size_t>((mouse.x - gridRect.x) / cellSize), state.width - 1);
                const std::size_t y =
                    std::min(static_cast<std::size_t>((mouse.y - gridRect.y) / cellSize), state.height - 1);
                if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    if (x == goalX && y == goalY)
                    {
                        continue;
                    }
                    grid.SetObstacle(x, y, !state.obstacles[y * state.width + x]);
                }
                else
                {
                    if (state.obstacles[y * state.width + x])
                    {
                        continue;
                    }
                    grid.SetGoal(x, y);
                    goalX = x;
                    goalY = y;
                }
                state = grid.GetState();
            }
        }
        SDL_Delay(16);
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
