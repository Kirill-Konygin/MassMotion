#include "Agents.h"
#include "Grid.h"
#include "ThreadPool.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <vector>

constexpr float minimumDetailCellSize = 8.0f;

struct ViewerScene
{
    MassMotion::Grid grid;
    MassMotion::Agents agents;
    std::size_t goalX;
    std::size_t goalY;
    const char* title;
};

ViewerScene CreateSmallScene()
{
    ViewerScene scene{MassMotion::Grid(20, 14), {}, 10, 7, "MassMotion - 3 agents | 1: small, 2: crowd"};
    scene.grid.SetObstacle(2, 5);
    scene.grid.SetObstacle(3, 5);
    scene.grid.SetObstacle(4, 5);
    scene.grid.SetGoal(scene.goalX, scene.goalY);
    scene.agents.Add(1, 1);
    scene.agents.Add(1, 7);
    scene.agents.Add(1, 12);
    return scene;
}

ViewerScene CreateCrowdScene()
{
    constexpr std::size_t width = 512;
    constexpr std::size_t height = 512;
    constexpr std::size_t agentCount = 100000;
    ViewerScene scene{MassMotion::Grid(width, height),
                      {},
                      width / 2,
                      height / 2,
                      "MassMotion - 100000 agents to center | 1: small, 2: crowd"};
    const auto addSquareWall = [&scene](std::size_t inset, int gapOffset)
    {
        constexpr int halfGap = 20;
        constexpr int center = static_cast<int>(width / 2);
        const std::size_t end = width - inset - 1;
        const auto upperRightGap = static_cast<std::size_t>(center + gapOffset - halfGap);
        const auto lowerLeftGap = static_cast<std::size_t>(center - gapOffset - halfGap);
        for (std::size_t position = inset; position <= end; ++position)
        {
            if (position < upperRightGap || position >= upperRightGap + halfGap * 2)
            {
                scene.grid.SetObstacle(position, inset);
                scene.grid.SetObstacle(end, position);
            }
            if (position < lowerLeftGap || position >= lowerLeftGap + halfGap * 2)
            {
                scene.grid.SetObstacle(position, end);
                scene.grid.SetObstacle(inset, position);
            }
        }
    };
    addSquareWall(96, -56);
    addSquareWall(176, 36);
    scene.grid.SetGoal(scene.goalX, scene.goalY);

    scene.agents.Reserve(agentCount);
    for (std::size_t layer = 0; scene.agents.GetCount() < agentCount; ++layer)
    {
        const std::size_t first = 8 + layer;
        const std::size_t last = width - 9 - layer;
        for (std::size_t offset = 0; offset < last - first && scene.agents.GetCount() < agentCount; ++offset)
        {
            const std::size_t positions[][2] = {
                {first + offset, first},
                {last, first + offset},
                {last - offset, last},
                {first, last - offset},
            };
            for (const auto& position : positions)
            {
                if (scene.agents.GetCount() == agentCount)
                {
                    break;
                }
                scene.agents.Add(position[0], position[1], 8.0f);
            }
        }
    }
    return scene;
}

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
    const bool showDetails = cellSize >= minimumDetailCellSize;
    const float inset = cellSize * 0.08f;
    const float goalSize = std::max(cellSize - inset * 2.0f, 6.0f);
    const SDL_FRect goalRect = {
        originX + (static_cast<float>(goalX) + 0.5f) * cellSize - goalSize * 0.5f,
        originY + (static_cast<float>(goalY) + 0.5f) * cellSize - goalSize * 0.5f,
        goalSize,
        goalSize,
    };
    if (!SDL_SetRenderDrawColor(renderer, 50, 145, 100, 255) || !SDL_RenderFillRect(renderer, &goalRect) ||
        !SDL_SetRenderDrawColor(renderer, 58, 70, 84, 255))
    {
        return false;
    }
    if (showDetails)
    {
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
    }
    for (std::size_t y = 0; y < state.height; ++y)
    {
        for (std::size_t x = 0; x < state.width; ++x)
        {
            if (state.obstacles[y * state.width + x])
            {
                const float obstacleSize = std::max(showDetails ? cellSize - inset * 2.0f : cellSize, 2.0f);
                const SDL_FRect obstacleRect = {
                    originX + (static_cast<float>(x) + 0.5f) * cellSize - obstacleSize * 0.5f,
                    originY + (static_cast<float>(y) + 0.5f) * cellSize - obstacleSize * 0.5f,
                    obstacleSize,
                    obstacleSize,
                };
                if (!SDL_SetRenderDrawColor(renderer, 190, 100, 70, 255) ||
                    !SDL_RenderFillRect(renderer, &obstacleRect))
                {
                    return false;
                }
                continue;
            }

            if (showDetails)
            {
                const SDL_FPoint center = {originX + (static_cast<float>(x) + 0.5f) * cellSize,
                                           originY + (static_cast<float>(y) + 0.5f) * cellSize};
                if (!SDL_SetRenderDrawColor(renderer, 164, 205, 225, 255) ||
                    !DrawArrow(renderer, state.cells[y * state.width + x], center, cellSize))
                {
                    return false;
                }
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
                std::size_t gridWidth, std::vector<SDL_FPoint>& points)
{
    if (gridRect.w <= 0.0f || agents.GetCount() == 0)
    {
        return true;
    }
    const float cellSize = gridRect.w / static_cast<float>(gridWidth);
    if (cellSize < minimumDetailCellSize)
    {
        points.resize(agents.GetCount());
        for (std::size_t i = 0; i < agents.GetCount(); ++i)
        {
            const auto position = agents.GetPosition(i);
            points[i] = {gridRect.x + position.x * cellSize, gridRect.y + position.y * cellSize};
        }
        return SDL_SetRenderDrawColor(renderer, 255, 204, 64, 255) &&
               SDL_RenderPoints(renderer, points.data(), static_cast<int>(points.size()));
    }
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
    auto scene = CreateCrowdScene();
    const std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
        SDL_CreateWindow(scene.title, 960, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY),
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

    MassMotion::GridState state = scene.grid.GetState();
    MassMotion::ThreadPool threadPool;
    std::vector<SDL_FPoint> agentPoints;

    Uint64 previousTicks = SDL_GetTicksNS();

    while (true)
    {
        const Uint64 ticks = SDL_GetTicksNS();
        // Avoid a large jump after dragging the window or resuming the application.
        const double elapsedSeconds = std::min(static_cast<double>(ticks - previousTicks) / 1.0e9, 0.1);
        previousTicks = ticks;
        scene.agents.Update(state, elapsedSeconds, threadPool);

        SDL_FRect gridRect{};
        if (!SDL_SetRenderDrawColor(renderer.get(), 24, 28, 34, 255) || !SDL_RenderClear(renderer.get()) ||
            !DrawGrid(renderer.get(), state, scene.goalX, scene.goalY, gridRect) ||
            !DrawAgents(renderer.get(), scene.agents, gridRect, state.width, agentPoints) ||
            !SDL_RenderPresent(renderer.get()))
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

            if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat &&
                (event.key.key == SDLK_1 || event.key.key == SDLK_2))
            {
                scene = event.key.key == SDLK_1 ? CreateSmallScene() : CreateCrowdScene();
                state = scene.grid.GetState();
                if (!SDL_SetWindowTitle(window.get(), scene.title))
                {
                    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not set window title: %s", SDL_GetError());
                    return EXIT_FAILURE;
                }
                previousTicks = SDL_GetTicksNS();
                break;
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
                    if (x == scene.goalX && y == scene.goalY)
                    {
                        continue;
                    }
                    scene.grid.SetObstacle(x, y, !state.obstacles[y * state.width + x]);
                }
                else
                {
                    if (state.obstacles[y * state.width + x])
                    {
                        continue;
                    }
                    scene.grid.SetGoal(x, y);
                    scene.goalX = x;
                    scene.goalY = y;
                }
                state = scene.grid.GetState();
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
