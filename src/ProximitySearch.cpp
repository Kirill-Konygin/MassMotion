#include "ProximitySearch.h"

#include "Agents.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace MassMotion
{

std::size_t GridDimension(std::size_t extent, double cellSize)
{
    const double count = std::max(1.0, std::ceil(static_cast<double>(extent) / cellSize));
    if (count >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
    {
        throw std::length_error("Proximity grid dimensions are too large");
    }
    return static_cast<std::size_t>(count);
}

bool IsWithinRadius(Vector2 first, Vector2 second, double radiusSquared)
{
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return dx * dx + dy * dy < radiusSquared;
}

ProximitySearch::ProximitySearch(double radius, std::size_t width, std::size_t height) : mRadiusSquared(radius * radius)
{
    if (!std::isfinite(radius) || radius < 0.0)
    {
        throw std::invalid_argument("Proximity radius must be finite and nonnegative");
    }
    if (width == 0 || height == 0)
    {
        throw std::invalid_argument("Proximity grid dimensions must be positive");
    }

    // Cover rounding in the float coordinate subtraction used by the narrow phase.
    mCellSize = std::max(1.0, radius * (1.0 + 2.0 * std::numeric_limits<float>::epsilon()));
    mGrid.resize(GridDimension(height, mCellSize));
    const std::size_t columns = GridDimension(width, mCellSize);
    for (auto& row : mGrid)
    {
        row.resize(columns);
    }
}

bool ProximitySearch::MatchesGridSize(std::size_t width, std::size_t height) const
{
    return width > 0 && height > 0 && !mGrid.empty() && mGrid.size() == GridDimension(height, mCellSize) &&
           mGrid.front().size() == GridDimension(width, mCellSize);
}

void ProximitySearch::Update(const Agents& agents)
{
    ResetNeighborLists(agents.GetCount());
    if (agents.GetCount() < 2 || mRadiusSquared == 0.0)
    {
        return;
    }

    UpdateGrid(agents);
    for (std::size_t y = 0; y < mGrid.size(); ++y)
    {
        for (std::size_t x = 0; x < mGrid[y].size(); ++x)
        {
            const auto& cell = mGrid[y][x];
            if (!cell.empty())
            {
                const auto candidates = BroadPhase(x, y);
                NarrowPhase(agents, cell, candidates);
            }
        }
    }

    // Preserve the separation force accumulation order.
    for (auto& neighbors : mNeighbors)
    {
        std::sort(neighbors.begin(), neighbors.end());
    }
}

void ProximitySearch::ResetNeighborLists(std::size_t agentCount)
{
    mNeighbors.resize(agentCount);
    for (auto& neighbors : mNeighbors)
    {
        neighbors.clear();
    }
}

void ProximitySearch::UpdateGrid(const Agents& agents)
{
    for (auto& row : mGrid)
    {
        for (auto& cell : row)
        {
            cell.clear();
        }
    }

    for (std::size_t i = 0; i < agents.GetCount(); ++i)
    {
        const auto position = agents.GetPosition(i);
        if (!std::isfinite(position.x) || !std::isfinite(position.y))
        {
            continue;
        }
        // Border cells also hold outside agents; exact distances still use their original positions.
        const auto x = static_cast<std::size_t>(std::clamp(static_cast<double>(position.x) / mCellSize, 0.0,
                                                           static_cast<double>(mGrid.front().size() - 1)));
        const auto y = static_cast<std::size_t>(
            std::clamp(static_cast<double>(position.y) / mCellSize, 0.0, static_cast<double>(mGrid.size() - 1)));
        mGrid[y][x].push_back(i);
    }
}

std::span<const std::size_t> ProximitySearch::BroadPhase(std::size_t cellX, std::size_t cellY)
{
    mCandidates.clear();
    const std::size_t firstX = cellX == 0 ? 0 : cellX - 1;
    const std::size_t firstY = cellY == 0 ? 0 : cellY - 1;
    const std::size_t lastX = std::min(cellX + 1, mGrid.front().size() - 1);
    const std::size_t lastY = std::min(cellY + 1, mGrid.size() - 1);
    for (std::size_t y = firstY; y <= lastY; ++y)
    {
        for (std::size_t x = firstX; x <= lastX; ++x)
        {
            const auto& cell = mGrid[y][x];
            mCandidates.insert(mCandidates.end(), cell.begin(), cell.end());
        }
    }
    return mCandidates;
}

void ProximitySearch::NarrowPhase(const Agents& agents, std::span<const std::size_t> cell,
                                  std::span<const std::size_t> candidates)
{
    for (const std::size_t i : cell)
    {
        const auto position = agents.GetPosition(i);
        for (const std::size_t j : candidates)
        {
            if (j > i && IsWithinRadius(position, agents.GetPosition(j), mRadiusSquared))
            {
                AddNeighbor(i, j);
                AddNeighbor(j, i);
            }
        }
    }
}

} // namespace MassMotion
