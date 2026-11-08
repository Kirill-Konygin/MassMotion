#include "ProximitySearch.h"

#include "Agent.h"

#include <cmath>
#include <stdexcept>

namespace MassMotion
{

bool IsWithinRadius(AgentPosition first, AgentPosition second, double radiusSquared)
{
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return dx * dx + dy * dy < radiusSquared;
}

void ProximitySearch::Update(std::span<const Agent> agents, double radius)
{
    if (!std::isfinite(radius) || radius < 0.0)
    {
        throw std::invalid_argument("Proximity radius must be finite and nonnegative");
    }

    ResetNeighborLists(agents.size());
    FindNeighborPairs(agents, radius * radius);
}

void ProximitySearch::ResetNeighborLists(std::size_t agentCount)
{
    mNeighbors.resize(agentCount);
    for (auto& neighbors : mNeighbors)
    {
        neighbors.clear();
    }
}

void ProximitySearch::FindNeighborPairs(std::span<const Agent> agents, double radiusSquared)
{
    for (std::size_t i = 0; i < agents.size(); ++i)
    {
        const auto position = agents[i].GetPosition();
        for (std::size_t j = i + 1; j < agents.size(); ++j)
        {
            if (IsWithinRadius(position, agents[j].GetPosition(), radiusSquared))
            {
                AddNeighbor(i, j);
                AddNeighbor(j, i);
            }
        }
    }
}

} // namespace MassMotion
