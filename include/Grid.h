//
//  Grid.h
//  MassMotion
//
//  Created by Кирилл Коныгин on 02.11.2026.
//

#ifndef MASSMOTION_GRID_H
#define MASSMOTION_GRID_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace MassMotion
{

enum class Cell : std::uint8_t
{
    None,
    North,
    NorthEast,
    East,
    SouthEast,
    South,
    SouthWest,
    West,
    NorthWest
};

// Coordinates start at the top-left: x increases east, y increases south.
class Grid
{
  public:
    // Both dimensions must be positive. All cells initially contain Cell::None.
    Grid(std::size_t width, std::size_t height);

    // Updates every cell along a shortest path to the goal on an obstacle-free grid.
    // Eight movement directions are allowed; the goal itself contains Cell::None.
    // Invalid coordinates throw std::out_of_range without changing the grid.
    void SetGoal(std::size_t x, std::size_t y);

  private:
    std::size_t mWidth;
    std::size_t mHeight;
    std::vector<Cell> mCells;
};

} // namespace MassMotion

#endif // MASSMOTION_GRID_H
