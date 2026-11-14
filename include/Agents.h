#ifndef MASSMOTION_AGENTS_H
#define MASSMOTION_AGENTS_H

#include "Grid.h"
#include "ProximitySearch.h"
#include "Vector2.h"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace MassMotion
{

class ThreadPool;

class Agents
{
  public:
    void Reserve(std::size_t capacity);

    // Speed is measured in grid cells per second.
    std::size_t Add(std::size_t x, std::size_t y, float maxSpeed = 3.0f);

    // Separation uses positions from the start of each simulation step.
    // Cell::None requests zero flow velocity.
    void Update(const GridState& state, float elapsedSeconds, ThreadPool& threadPool);

    [[nodiscard]] std::size_t GetCount() const
    {
        return mAgents.size();
    }

    // Grid-space position: the center of cell (0, 0) is (0.5, 0.5).
    [[nodiscard]] Vector2 GetPosition(std::size_t agentIndex) const
    {
        return mAgents[agentIndex].position;
    }

    // The last facing direction is kept while stopped.
    [[nodiscard]] Vector2 GetDirection(std::size_t agentIndex) const;

  private:
    struct State
    {
        Vector2 position;
        float maxSpeed;
        float heading = 0.0f;
        Vector2 velocity{};
        Vector2 steeringForce{};
    };

    void SimulateStep(const GridState& state, ProximitySearch& proximitySearch, float elapsedSeconds,
                      ThreadPool& threadPool);

    void CalculateSteering(const GridState& state, const ProximitySearch& proximitySearch,
                           std::size_t begin, std::size_t end);
    [[nodiscard]] static Vector2 GetDesiredFlowDirection(const State& agent, const GridState& state);
    [[nodiscard]] static Vector2 CalculateFlowFollowingForce(const State& agent, const GridState& state);
    [[nodiscard]] Vector2 CalculateSeparationForce(const State& agent, std::span<const std::size_t> neighbors,
                                                   float maxForce) const;
    [[nodiscard]] static Vector2 CalculateRepulsionFrom(const State& agent, const State& neighbor);

    void Integrate(const GridState& state, float elapsedSeconds, std::size_t begin, std::size_t end);
    void ApplySteering(const GridState& state, float elapsedSeconds, std::size_t begin, std::size_t end);
    void Move(const GridState& state, float elapsedSeconds, std::size_t begin, std::size_t end);
    static void SlideAlongObstacles(State& agent, const GridState& state, Vector2 next);
    void UpdateHeading(std::size_t begin, std::size_t end);

    std::vector<State> mAgents;
    std::optional<ProximitySearch> mProximitySearch;
};

} // namespace MassMotion

#endif // MASSMOTION_AGENTS_H
