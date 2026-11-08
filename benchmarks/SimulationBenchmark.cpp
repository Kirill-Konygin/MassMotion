#include "Agent.h"
#include "Grid.h"

#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <span>
#include <vector>
#include "ScopeTimer.h"

constexpr std::size_t gridWidth = 256;
constexpr std::size_t gridHeight = 256;
constexpr std::size_t agentCount = 512;
constexpr std::size_t stepCount = 3;
// Matching the integration limit makes UpdateAll execute exactly one SimulateStep.
constexpr double elapsedSeconds = 1.0 / 30.0;

MassMotion::Grid CreateGrid()
{
    MassMotion::Grid grid(gridWidth, gridHeight);
    constexpr std::size_t wallX = gridWidth / 2;
    constexpr std::size_t gapStart = gridHeight / 2 - 8;
    constexpr std::size_t gapEnd = gridHeight / 2 + 8;
    for (std::size_t y = 16; y < gridHeight - 16; ++y)
    {
        if (y < gapStart || y >= gapEnd)
        {
            grid.SetObstacle(wallX, y);
        }
    }
    grid.SetGoal(gridWidth - 16, gridHeight - 16);
    return grid;
}

std::vector<MassMotion::Agent> CreateAgents()
{
    constexpr std::size_t columns = 32;
    constexpr std::size_t origin = 8;
    constexpr std::size_t spacing = 2;
    std::vector<MassMotion::Agent> agents;
    agents.reserve(agentCount);
    for (std::size_t i = 0; i < agentCount; ++i)
    {
        agents.emplace_back(origin + i % columns * spacing, origin + i / columns * spacing);
    }
    return agents;
}

int main()
{
    const auto grid = CreateGrid();
    const auto state = grid.GetState();
    auto agents = CreateAgents();
    
    {
        MM_PROFILE_SCOPE("All");
        for (std::size_t step = 0; step < stepCount; ++step)
        {
            MassMotion::Agent::UpdateAll(agents, state, elapsedSeconds);
        }
    }

    return EXIT_SUCCESS;
}
