#include "ProximitySearch.h"

#include "Agents.h"

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

void ProximitySearch::Update(const Agents& agents, double radius)
{
    if (!std::isfinite(radius) || radius < 0.0)
    {
        throw std::invalid_argument("Proximity radius must be finite and nonnegative");
    }

    ResetNeighborLists(agents.GetCount());
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

void ProximitySearch::FindNeighborPairs(const Agents& agents, double radiusSquared)
{
    const std::size_t agentCount = agents.GetCount();
    for (std::size_t i = 0; i < agentCount; ++i)
    {
        const auto position = agents.GetPosition(i);
        for (std::size_t j = i + 1; j < agentCount; ++j)
        {
            if (IsWithinRadius(position, agents.GetPosition(j), radiusSquared))
            {
                AddNeighbor(i, j);
                AddNeighbor(j, i);
            }
        }
    }
}

} // namespace MassMotion
