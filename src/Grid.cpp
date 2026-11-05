//
//  Grid.cpp
//  MassMotion
//
//  Created by Кирилл Коныгин on 02.11.2026.
//

#include "Grid.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>
#include <utility>

namespace MassMotion
{

Grid::Grid(std::size_t width, std::size_t height) : mWidth(width), mHeight(height)
{
    if (width == 0 || height == 0)
    {
        throw std::invalid_argument("Grid dimensions must be positive");
    }

    if (width > mCells.max_size() / height)
    {
        throw std::length_error("Grid dimensions are too large");
    }

    mCells.resize(width * height, Cell::None);
    mObstacles.resize(width * height, false);
}

void Grid::SetGoal(std::size_t x, std::size_t y)
{
    if (x >= mWidth || y >= mHeight)
    {
        throw std::out_of_range("Goal coordinates are outside the grid");
    }

    const std::size_t goal = y * mWidth + x;
    if (mObstacles[goal])
    {
        throw std::invalid_argument("Goal cell is blocked");
    }

    mGoal = goal;
    UpdateDirections();
}

void Grid::UpdateDirections()
{
    std::fill(mCells.begin(), mCells.end(), Cell::None);
    if (!mGoal)
    {
        return;
    }

    RunDijkstra(*mGoal);
}

void Grid::RunDijkstra(std::size_t goal)
{
    static constexpr Cell directions[3][3] = {
        {Cell::NorthWest, Cell::North, Cell::NorthEast},
        {Cell::West, Cell::None, Cell::East},
        {Cell::SouthWest, Cell::South, Cell::SouthEast},
    };

    std::vector<double> distances(mCells.size(), std::numeric_limits<double>::infinity());
    distances[goal] = 0.0;
    using FrontierEntry = std::pair<double, std::size_t>;
    std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, std::greater<>> frontier;
    frontier.emplace(0.0, goal);
    while (!frontier.empty())
    {
        const auto [distance, current] = frontier.top();
        frontier.pop();
        if (distance > distances[current])
        {
            continue;
        }
        const std::size_t x = current % mWidth;
        const std::size_t y = current / mWidth;
        for (int dy = -1; dy <= 1; ++dy)
        {
            if ((dy < 0 && y == 0) || (dy > 0 && y == mHeight - 1))
            {
                continue;
            }
            const std::size_t nextY = dy < 0 ? y - 1 : (dy > 0 ? y + 1 : y);
            for (int dx = -1; dx <= 1; ++dx)
            {
                if ((dx == 0 && dy == 0) || (dx < 0 && x == 0) || (dx > 0 && x == mWidth - 1))
                {
                    continue;
                }
                const std::size_t nextX = dx < 0 ? x - 1 : (dx > 0 ? x + 1 : x);
                const std::size_t next = nextY * mWidth + nextX;
                if (mObstacles[next])
                {
                    continue;
                }
                if (dx != 0 && dy != 0 && (mObstacles[y * mWidth + nextX] || mObstacles[nextY * mWidth + x]))
                {
                    continue;
                }

                const double nextDistance = distance + (dx != 0 && dy != 0 ? std::numbers::sqrt2_v<double> : 1.0);
                if (nextDistance >= distances[next])
                {
                    continue;
                }

                // Point back along the cheapest path found for this neighbor.
                distances[next] = nextDistance;
                mCells[next] = directions[1 - dy][1 - dx];
                frontier.emplace(nextDistance, next);
            }
        }
    }
}

void Grid::SetObstacle(std::size_t x, std::size_t y, bool blocked)
{
    if (x >= mWidth || y >= mHeight)
    {
        throw std::out_of_range("Obstacle coordinates are outside the grid");
    }

    const std::size_t cell = y * mWidth + x;
    if (blocked && mGoal && cell == *mGoal)
    {
        throw std::invalid_argument("Goal cell cannot be blocked");
    }
    if (mObstacles[cell] == blocked)
    {
        return;
    }

    mObstacles[cell] = blocked;
    UpdateDirections();
}

GridState Grid::GetState() const
{
    return {mWidth, mHeight, mCells, mObstacles};
}

} // namespace MassMotion
