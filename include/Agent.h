#ifndef MASSMOTION_AGENT_H
#define MASSMOTION_AGENT_H

#include "Grid.h"

#include <span>

namespace MassMotion
{

struct AgentPosition
{
    double x;
    double y;
};

struct AgentDirection
{
    double x;
    double y;
};

class ProximitySearch;

class Agent
{
  public:
    // Position is a cell coordinate; speed is the maximum speed in cells per second and must be positive and finite.
    Agent(std::size_t x, std::size_t y, double maxSpeed = 3.0);

    // Updates a single agent using flow-field steering. Cell::None requests zero velocity.
    // The state must be a snapshot from Grid. Elapsed time must be finite and nonnegative.
    void Update(const GridState& state, double elapsedSeconds);

    // Combines flow following and separation using cached neighbor lists.
    // Forces are calculated for all agents before any positions are updated.
    static void UpdateAll(std::span<Agent> agents, const GridState& state, double elapsedSeconds);

    // Grid-space position: the center of cell (0, 0) is (0.5, 0.5).
    [[nodiscard]] AgentPosition GetPosition() const
    {
        return mPosition;
    }

    // Unit facing vector aligned with velocity; the last direction is kept while stopped.
    [[nodiscard]] AgentDirection GetDirection() const;

  private:
    struct SteeringForce
    {
        double x = 0.0;
        double y = 0.0;
    };

    static void SimulateStep(std::span<Agent> agents, const GridState& state, ProximitySearch& proximitySearch,
                             double elapsedSeconds);

    void CalculateSteering(const GridState& state, std::span<const Agent> agents,
                           std::span<const std::size_t> neighbors);
    [[nodiscard]] AgentDirection GetDesiredFlowDirection(const GridState& state) const;
    [[nodiscard]] SteeringForce CalculateFlowFollowingForce(const GridState& state) const;
    [[nodiscard]] SteeringForce CalculateSeparationForce(std::span<const Agent> agents,
                                                         std::span<const std::size_t> neighbors, double maxForce) const;
    [[nodiscard]] SteeringForce CalculateRepulsionFrom(const Agent& neighbor) const;

    void Integrate(const GridState& state, double elapsedSeconds);
    void ApplySteering(double elapsedSeconds);
    void Move(const GridState& state, double elapsedSeconds);
    void SlideAlongObstacles(const GridState& state, AgentPosition next);
    void UpdateHeading();

    AgentPosition mPosition;
    double mMaxSpeed;
    double mVelocityX = 0.0;
    double mVelocityY = 0.0;
    SteeringForce mSteeringForce;
    double mHeading = 0.0;
};

} // namespace MassMotion

#endif // MASSMOTION_AGENT_H
