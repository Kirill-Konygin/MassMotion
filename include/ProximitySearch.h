#ifndef MASSMOTION_PROXIMITY_SEARCH_H
#define MASSMOTION_PROXIMITY_SEARCH_H

#include <cstddef>
#include <span>
#include <vector>

namespace MassMotion
{

class Agent;

class ProximitySearch
{
  public:
    // Rebuilds all neighbor lists with a naive O(n^2) search, checking each pair once.
    // Neighbors are strictly within radius; an agent never includes itself.
    // Radius must be finite and nonnegative. Rebuild after agent positions or order change.
    void Update(std::span<const Agent> agents, double radius);

    // Returns cached indices into the agents passed to Update, without repeating the search.
    // The view remains valid until the next Update or destruction of this object.
    // An invalid agent index throws std::out_of_range.
    [[nodiscard]] std::span<const std::size_t> GetNeighbors(std::size_t agentIndex) const
    {
        return mNeighbors.at(agentIndex);
    }

  private:
    void ResetNeighborLists(std::size_t agentCount);
    void FindNeighborPairs(std::span<const Agent> agents, double radiusSquared);

    void AddNeighbor(std::size_t agentIndex, std::size_t neighborIndex)
    {
        mNeighbors[agentIndex].push_back(neighborIndex);
    }

    std::vector<std::vector<std::size_t>> mNeighbors;
};

} // namespace MassMotion

#endif // MASSMOTION_PROXIMITY_SEARCH_H
