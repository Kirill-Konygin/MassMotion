#include "ProximitySearch.h"

#include "Agents.h"
#include "ThreadPool.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace MassMotion
{

constexpr std::size_t chunkSize = 128;

std::size_t GridDimension(std::size_t extent, double cellSize)
{
    const double count = std::max(1.0, std::ceil(static_cast<double>(extent) / cellSize));
    if (count >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
    {
        throw std::length_error("Proximity grid dimensions are too large");
    }
    return static_cast<std::size_t>(count);
}

std::size_t GridCoordinate(float position, double cellSize, std::size_t cellCount)
{
    return static_cast<std::size_t>(
        std::clamp(static_cast<double>(position) / cellSize, 0.0, static_cast<double>(cellCount - 1)));
}

bool IsWithinRadius(Vector2 first, Vector2 second, double radiusSquared)
{
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return dx * dx + dy * dy < radiusSquared;
}

ProximitySearch::ProximitySearch(double radius, std::size_t width, std::size_t height)
    : mRadiusSquared(radius * radius)
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

void ProximitySearch::Update(const Agents& agents, ThreadPool& threadPool)
{
    const std::size_t agentCount = agents.GetCount();
    ResetNeighborLists(agentCount);
    if (agentCount < 2 || mRadiusSquared == 0.0)
    {
        return;
    }

    UpdateGrid(agents);
    const std::size_t chunkCount = (agentCount + chunkSize - 1) / chunkSize;
    for (std::size_t chunk = 0; chunk < chunkCount; ++chunk)
    {
        threadPool.Enqueue([this, &agents, chunk]
        {
            thread_local WorkerContext context;
            const std::size_t begin = chunk * chunkSize;
            const std::size_t end = std::min(begin + chunkSize, agents.GetCount());
            ProcessChunk(agents, begin, end, context);
        });
    }
    threadPool.Wait();
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
        const std::size_t x = GridCoordinate(position.x, mCellSize, mGrid.front().size());
        const std::size_t y = GridCoordinate(position.y, mCellSize, mGrid.size());
        mGrid[y][x].push_back(i);
    }
}

void ProximitySearch::ProcessChunk(const Agents& agents, std::size_t begin, std::size_t end, WorkerContext& context)
{
    for (std::size_t i = begin; i < end; ++i)
    {
        const auto position = agents.GetPosition(i);
        if (!std::isfinite(position.x) || !std::isfinite(position.y))
        {
            continue;
        }

        const std::size_t x = GridCoordinate(position.x, mCellSize, mGrid.front().size());
        const std::size_t y = GridCoordinate(position.y, mCellSize, mGrid.size());
        const auto candidates = BroadPhase(x, y, context);
        NarrowPhase(agents, i, candidates);
    }
}

std::span<const std::size_t> ProximitySearch::BroadPhase(std::size_t cellX, std::size_t cellY,
                                                         WorkerContext& context) const
{
    auto& candidates = context.candidates;
    candidates.clear();
    const std::size_t firstX = cellX == 0 ? 0 : cellX - 1;
    const std::size_t firstY = cellY == 0 ? 0 : cellY - 1;
    const std::size_t lastX = std::min(cellX + 1, mGrid.front().size() - 1);
    const std::size_t lastY = std::min(cellY + 1, mGrid.size() - 1);
    for (std::size_t y = firstY; y <= lastY; ++y)
    {
        for (std::size_t x = firstX; x <= lastX; ++x)
        {
            const auto& cell = mGrid[y][x];
            candidates.insert(candidates.end(), cell.begin(), cell.end());
        }
    }
    return candidates;
}

void ProximitySearch::NarrowPhase(const Agents& agents, std::size_t agentIndex, std::span<const std::size_t> candidates)
{
    const auto position = agents.GetPosition(agentIndex);
    auto& neighbors = mNeighbors[agentIndex];
    for (const std::size_t j : candidates)
    {
        if (j != agentIndex && IsWithinRadius(position, agents.GetPosition(j), mRadiusSquared))
        {
            neighbors.push_back(j);
        }
    }

    // Preserve the separation force accumulation order.
    std::sort(neighbors.begin(), neighbors.end());
}

} // namespace MassMotion
