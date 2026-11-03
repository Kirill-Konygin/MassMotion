//
//  Grid.cpp
//  MassMotion
//
//  Created by Кирилл Коныгин on 02.11.2026.
//

#include "Grid.h"

#include <stdexcept>

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
}

void Grid::SetGoal(std::size_t x, std::size_t y)
{
    if (x >= mWidth || y >= mHeight)
    {
        throw std::out_of_range("Goal coordinates are outside the grid");
    }

    static constexpr Cell directions[3][3] = {
        {Cell::NorthWest, Cell::North, Cell::NorthEast},
        {Cell::West, Cell::None, Cell::East},
        {Cell::SouthWest, Cell::South, Cell::SouthEast},
    };

    for (std::size_t cellY = 0; cellY < mHeight; ++cellY)
    {
        const int directionY = (cellY < y) - (cellY > y);
        const std::size_t rowOffset = cellY * mWidth;
        for (std::size_t cellX = 0; cellX < mWidth; ++cellX)
        {
            const int directionX = (cellX < x) - (cellX > x);
            mCells[rowOffset + cellX] = directions[directionY + 1][directionX + 1];
        }
    }
}

GridState Grid::GetState() const
{
    return {mWidth, mHeight, mCells};
}

} // namespace MassMotion
