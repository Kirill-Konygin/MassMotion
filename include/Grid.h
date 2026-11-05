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

// An owning snapshot. Cells and obstacles use row-major order: y * width + x.
struct GridState
{
    std::size_t width;
    std::size_t height;
    std::vector<Cell> cells;
    std::vector<bool> obstacles;
};

// Coordinates start at the top-left: x increases east, y increases south.
class Grid
{
  public:
    // Both dimensions must be positive. All cells initially contain Cell::None and no obstacles.
    Grid(std::size_t width, std::size_t height);

    // Updates every cell along a shortest path to the goal on an obstacle-free grid.
    // Eight movement directions are allowed; the goal itself contains Cell::None.
    // Invalid coordinates throw std::out_of_range without changing the grid.
    void SetGoal(std::size_t x, std::size_t y);

    // Sets or removes an obstacle independently of cell directions. SetGoal ignores obstacles.
    // Invalid coordinates throw std::out_of_range without changing the grid.
    void SetObstacle(std::size_t x, std::size_t y, bool blocked = true);

    // The snapshot remains valid and unchanged after the grid is updated or destroyed.
    [[nodiscard]] GridState GetState() const;

  private:
    std::size_t mWidth;
    std::size_t mHeight;
    std::vector<Cell> mCells;
    std::vector<bool> mObstacles;
};

} // namespace MassMotion

#endif // MASSMOTION_GRID_H
