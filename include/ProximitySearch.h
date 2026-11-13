#ifndef MASSMOTION_PROXIMITY_SEARCH_H
#define MASSMOTION_PROXIMITY_SEARCH_H

#include <cstddef>
#include <span>
#include <vector>

namespace MassMotion
{

class Agents;
class ThreadPool;

class ProximitySearch
{
  public:
    // Dimensions use the same units as agent positions and must be positive.
    // Radius must be finite and nonnegative.
    ProximitySearch(double radius, std::size_t width, std::size_t height);

    [[nodiscard]] bool MatchesGridSize(std::size_t width, std::size_t height) const;

    // Neighbors are strictly within radius, sorted by index; an agent never includes itself.
    // Rebuild after agent positions or order change.
    void Update(const Agents& agents, ThreadPool& threadPool);

    // Returns cached indices into the agents passed to Update, without repeating the search.
    // The view remains valid until the next Update or destruction of this object.
    // An invalid agent index throws std::out_of_range.
    [[nodiscard]] std::span<const std::size_t> GetNeighbors(std::size_t agentIndex) const
    {
        return mNeighbors.at(agentIndex);
    }

  private:
    using GridCell = std::vector<std::size_t>;

    struct WorkerContext
    {
        std::vector<std::size_t> candidates;
    };

    void ResetNeighborLists(std::size_t agentCount);
    void UpdateGrid(const Agents& agents);
    void ProcessChunk(const Agents& agents, std::size_t begin, std::size_t end, WorkerContext& context);
    [[nodiscard]] std::span<const std::size_t> BroadPhase(std::size_t cellX, std::size_t cellY,
                                                          WorkerContext& context) const;
    void NarrowPhase(const Agents& agents, std::size_t agentIndex, std::span<const std::size_t> candidates);

    double mRadiusSquared;
    double mCellSize;
    std::vector<std::vector<GridCell>> mGrid;
    std::vector<std::vector<std::size_t>> mNeighbors;
};

} // namespace MassMotion

#endif // MASSMOTION_PROXIMITY_SEARCH_H
