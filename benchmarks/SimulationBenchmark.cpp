#include "Agents.h"
#include "Grid.h"
#include "ThreadPool.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace
{

constexpr std::size_t warmupSteps = 30;
constexpr std::size_t measuredSteps = 120;
constexpr std::size_t repeatCount = 5;
// Matching the integration limit makes Update execute exactly one SimulateStep.
constexpr float elapsedSeconds = 1.0f / 30.0f;

struct Scenario
{
    std::string_view name;
    std::size_t width;
    std::size_t height;
    std::size_t agentCount;
    std::size_t columns;
    std::size_t spacing;
    std::size_t agentsPerCell;
    bool wall;
};

constexpr std::array scenarios = {
    Scenario{"sparse", 256, 256, 10000, 100, 2, 1, false},
    Scenario{"sparse", 1000, 1000, 100000, 240, 2, 1, false},
    Scenario{"dense", 1000, 1000, 100000, 240, 1, 4, false},
    Scenario{"wall-gap", 1000, 1000, 100000, 240, 2, 1, true},
};

MassMotion::Grid CreateGrid(const Scenario& scenario)
{
    MassMotion::Grid grid(scenario.width, scenario.height);
    if (scenario.wall)
    {
        const std::size_t wallX = scenario.width / 2;
        const std::size_t gapStart = scenario.height / 2 - 8;
        const std::size_t gapEnd = scenario.height / 2 + 8;
        for (std::size_t y = 16; y < scenario.height - 16; ++y)
        {
            if (y < gapStart || y >= gapEnd)
            {
                grid.SetObstacle(wallX, y);
            }
        }
    }
    grid.SetGoal(scenario.width - 16, scenario.height - 16);
    return grid;
}

MassMotion::Agents CreateAgents(const Scenario& scenario, const MassMotion::GridState& state)
{
    constexpr std::size_t origin = 8;
    MassMotion::Agents agents;
    agents.Reserve(scenario.agentCount);
    for (std::size_t i = 0; i < scenario.agentCount; ++i)
    {
        const std::size_t cell = i / scenario.agentsPerCell;
        const std::size_t x = origin + cell % scenario.columns * scenario.spacing;
        const std::size_t y = origin + cell / scenario.columns * scenario.spacing;
        if (x >= state.width || y >= state.height || state.obstacles[y * state.width + x])
        {
            std::cerr << "Invalid agent placement in " << scenario.name << '\n';
            std::exit(EXIT_FAILURE);
        }
        agents.Add(x, y);
    }
    return agents;
}

double RunBenchmark(const MassMotion::Agents& initialAgents, const MassMotion::GridState& state,
                    MassMotion::ThreadPool& threadPool)
{
    double totalMicroseconds = 0.0;
    for (std::size_t repeat = 0; repeat < repeatCount; ++repeat)
    {
        auto agents = initialAgents;
        for (std::size_t step = 0; step < warmupSteps; ++step)
        {
            agents.Update(state, elapsedSeconds, threadPool);
        }

        const auto start = std::chrono::steady_clock::now();
        for (std::size_t step = 0; step < measuredSteps; ++step)
        {
            agents.Update(state, elapsedSeconds, threadPool);
        }
        const auto finish = std::chrono::steady_clock::now();
        totalMicroseconds += std::chrono::duration<double, std::micro>(finish - start).count();
    }

    return totalMicroseconds / (repeatCount * measuredSteps);
}

} // namespace

int main()
{
    for (const auto& scenario : scenarios)
    {
        const auto grid = CreateGrid(scenario);
        const auto state = grid.GetState();
        const auto initialAgents = CreateAgents(scenario, state);
        MassMotion::ThreadPool threadPool;
        std::cout << scenario.name << " (" << initialAgents.GetCount() << " agents)\n" << std::flush;
        const double averageMicroseconds = RunBenchmark(initialAgents, state, threadPool);
        std::cout << "Average step: " << std::fixed << std::setprecision(2)
                  << averageMicroseconds << " us\n\n" << std::flush;
    }

    return EXIT_SUCCESS;
}
