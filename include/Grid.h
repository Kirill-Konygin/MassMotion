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
#include <optional>
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

    // Updates directions along shortest paths around obstacles. Straight moves cost 1, diagonals cost sqrt(2);
    // diagonal moves require both adjacent orthogonal cells to be free.
    // The goal, obstacles and unreachable cells contain Cell::None.
    // Invalid coordinates throw std::out_of_range without changing the grid.
    // A blocked goal throws std::invalid_argument without changing the grid.
    void SetGoal(std::size_t x, std::size_t y);

    // Sets or removes an obstacle and recalculates directions if a goal has been set.
    // Invalid coordinates throw std::out_of_range without changing the grid.
    // Blocking the current goal throws std::invalid_argument without changing the grid.
    void SetObstacle(std::size_t x, std::size_t y, bool blocked = true);

    // The snapshot remains valid and unchanged after the grid is updated or destroyed.
    [[nodiscard]] GridState GetState() const;

  private:
    void UpdateDirections();
    void RunDijkstra(std::size_t goal);

    std::size_t mWidth;
    std::size_t mHeight;
    std::vector<Cell> mCells;
    std::vector<bool> mObstacles;
    std::optional<std::size_t> mGoal;
};

} // namespace MassMotion

#endif // MASSMOTION_GRID_H
