#ifndef MASSMOTION_AGENTS_H
#define MASSMOTION_AGENTS_H

#include "Grid.h"

#include <cstddef>
#include <span>
#include <vector>

namespace MassMotion
{

struct AgentPosition
{
    float x;
    float y;
};

struct AgentDirection
{
    float x;
    float y;
};

class ProximitySearch;

class Agents
{
  public:
    void Reserve(std::size_t capacity);

    // Speed is measured in grid cells per second.
    std::size_t Add(std::size_t x, std::size_t y, float maxSpeed = 3.0f);

    // Separation uses positions from the start of each simulation step.
    // Cell::None requests zero flow velocity.
    void Update(const GridState& state, float elapsedSeconds);

    [[nodiscard]] std::size_t GetCount() const
    {
        return mAgents.size();
    }

    // Grid-space position: the center of cell (0, 0) is (0.5, 0.5).
    [[nodiscard]] AgentPosition GetPosition(std::size_t agentIndex) const
    {
        return mAgents[agentIndex].position;
    }

    // The last facing direction is kept while stopped.
    [[nodiscard]] AgentDirection GetDirection(std::size_t agentIndex) const;

  private:
    struct SteeringForce
    {
        float x = 0.0f;
        float y = 0.0f;
    };

    struct State
    {
        AgentPosition position;
        float maxSpeed;
        float velocityX = 0.0f;
        float velocityY = 0.0f;
        SteeringForce steeringForce{};
        float heading = 0.0f;
    };

    void SimulateStep(const GridState& state, ProximitySearch& proximitySearch, float elapsedSeconds);

    void CalculateSteering(const GridState& state, const ProximitySearch& proximitySearch);
    [[nodiscard]] static AgentDirection GetDesiredFlowDirection(const State& agent, const GridState& state);
    [[nodiscard]] static SteeringForce CalculateFlowFollowingForce(const State& agent, const GridState& state);
    [[nodiscard]] SteeringForce CalculateSeparationForce(const State& agent, std::span<const std::size_t> neighbors,
                                                         float maxForce) const;
    [[nodiscard]] static SteeringForce CalculateRepulsionFrom(const State& agent, const State& neighbor);

    void Integrate(const GridState& state, float elapsedSeconds);
    void ApplySteering(const GridState& state, float elapsedSeconds);
    void Move(const GridState& state, float elapsedSeconds);
    static void SlideAlongObstacles(State& agent, const GridState& state, AgentPosition next);
    void UpdateHeading();

    std::vector<State> mAgents;
};

} // namespace MassMotion

#endif // MASSMOTION_AGENTS_H
