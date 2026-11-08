#include "Agent.h"
#include "ProximitySearch.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace MassMotion
{

constexpr double steeringResponseTime = 0.2;
constexpr double flowPredictionTime = 0.15;
constexpr double separationRadius = 1.0;
constexpr double separationWeight = 0.8;
constexpr double maxIntegrationStep = 1.0 / 30.0;
constexpr double coincidentDistanceSquared = 1.0e-12;
constexpr double minimumHeadingSpeed = 1.0e-6;

void Limit(double& x, double& y, double maximum)
{
    const double length = std::hypot(x, y);
    if (length > maximum)
    {
        x *= maximum / length;
        y *= maximum / length;
    }
}

bool IsFree(const GridState& state, AgentPosition position)
{
    return position.x >= 0.0 && position.y >= 0.0 && position.x < static_cast<double>(state.width) &&
           position.y < static_cast<double>(state.height) &&
           !state.obstacles[static_cast<std::size_t>(position.y) * state.width + static_cast<std::size_t>(position.x)];
}

bool CanMove(const GridState& state, AgentPosition from, AgentPosition to)
{
    if (!IsFree(state, from) || !IsFree(state, to))
    {
        return false;
    }
    const auto x = static_cast<std::size_t>(from.x);
    const auto y = static_cast<std::size_t>(from.y);
    const auto nextX = static_cast<std::size_t>(to.x);
    const auto nextY = static_cast<std::size_t>(to.y);
    return x == nextX || y == nextY ||
           (!state.obstacles[y * state.width + nextX] && !state.obstacles[nextY * state.width + x]);
}

AgentDirection FlowDirection(const GridState& state, AgentPosition position)
{
    constexpr double diagonal = 1.0 / std::numbers::sqrt2_v<double>;
    static constexpr AgentDirection directions[] = {
        {0.0, 0.0},
        {0.0, -1.0},
        {diagonal, -diagonal},
        {1.0, 0.0},
        {diagonal, diagonal},
        {0.0, 1.0},
        {-diagonal, diagonal},
        {-1.0, 0.0},
        {-diagonal, -diagonal},
    };
    const auto cell =
        state.cells[static_cast<std::size_t>(position.y) * state.width + static_cast<std::size_t>(position.x)];
    return directions[static_cast<std::size_t>(cell)];
}


Agent::Agent(std::size_t x, std::size_t y, double maxSpeed)
    : mPosition{static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5}, mMaxSpeed(maxSpeed)
{
    if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0)
    {
        throw std::invalid_argument("Agent speed must be positive and finite");
    }
}

void Agent::Update(const GridState& state, double elapsedSeconds)
{
    UpdateAll(std::span<Agent>(this, 1), state, elapsedSeconds);
}

void Agent::UpdateAll(std::span<Agent> agents, const GridState& state, double elapsedSeconds)
{
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
    {
        throw std::invalid_argument("Elapsed time must be finite and nonnegative");
    }
    if (agents.empty())
    {
        return;
    }

    ProximitySearch proximitySearch;
    while (elapsedSeconds > 0.0)
    {
        const double step = std::min(elapsedSeconds, maxIntegrationStep);
        SimulateStep(agents, state, proximitySearch, step);
        elapsedSeconds = std::max(0.0, elapsedSeconds - step);
    }
}

void Agent::SimulateStep(std::span<Agent> agents, const GridState& state, ProximitySearch& proximitySearch,
                         double elapsedSeconds)
{
    proximitySearch.Update(agents, separationRadius);
    for (std::size_t i = 0; i < agents.size(); ++i)
    {
        agents[i].CalculateSteering(state, agents, proximitySearch.GetNeighbors(i));
    }
    for (auto& agent : agents)
    {
        agent.Integrate(state, elapsedSeconds);
    }
}

void Agent::CalculateSteering(const GridState& state, std::span<const Agent> agents,
                              std::span<const std::size_t> neighbors)
{
    mSteeringForce = {};
    if (!IsFree(state, mPosition))
    {
        return;
    }

    const double maxForce = mMaxSpeed / steeringResponseTime;
    const auto flowForce = CalculateFlowFollowingForce(state);
    const auto separationForce = CalculateSeparationForce(agents, neighbors, maxForce);
    mSteeringForce = {flowForce.x + separationForce.x, flowForce.y + separationForce.y};
    Limit(mSteeringForce.x, mSteeringForce.y, maxForce);
}

AgentDirection Agent::GetDesiredFlowDirection(const GridState& state) const
{
    auto direction = FlowDirection(state, mPosition);
    const AgentPosition predicted{mPosition.x + mVelocityX * flowPredictionTime,
                                  mPosition.y + mVelocityY * flowPredictionTime};
    if ((direction.x != 0.0 || direction.y != 0.0) && CanMove(state, mPosition, predicted))
    {
        const auto predictedDirection = FlowDirection(state, predicted);
        // Enter the terminal cell before requesting a stop.
        if (predictedDirection.x != 0.0 || predictedDirection.y != 0.0)
        {
            direction = predictedDirection;
        }
    }
    return direction;
}

Agent::SteeringForce Agent::CalculateFlowFollowingForce(const GridState& state) const
{
    const auto direction = GetDesiredFlowDirection(state);
    return {(direction.x * mMaxSpeed - mVelocityX) / steeringResponseTime,
            (direction.y * mMaxSpeed - mVelocityY) / steeringResponseTime};
}

Agent::SteeringForce Agent::CalculateSeparationForce(std::span<const Agent> agents,
                                                     std::span<const std::size_t> neighbors, double maxForce) const
{
    SteeringForce repulsion;
    for (const std::size_t neighborIndex : neighbors)
    {
        const auto force = CalculateRepulsionFrom(agents[neighborIndex]);
        repulsion.x += force.x;
        repulsion.y += force.y;
    }
    return {repulsion.x * separationWeight * maxForce, repulsion.y * separationWeight * maxForce};
}

Agent::SteeringForce Agent::CalculateRepulsionFrom(const Agent& neighbor) const
{
    const double dx = mPosition.x - neighbor.mPosition.x;
    const double dy = mPosition.y - neighbor.mPosition.y;
    const double distanceSquared = dx * dx + dy * dy;
    if (distanceSquared < coincidentDistanceSquared)
    {
        // Give coincident pairs opposite forces without dividing by zero.
        return {this < &neighbor ? -1.0 : 1.0, 0.0};
    }
    // Normalized offset weighted by 1 / distance, as in Reynolds' separation.
    return {dx / distanceSquared, dy / distanceSquared};
}

void Agent::Integrate(const GridState& state, double elapsedSeconds)
{
    if (!IsFree(state, mPosition))
    {
        mVelocityX = 0.0;
        mVelocityY = 0.0;
        return;
    }

    ApplySteering(elapsedSeconds);
    Move(state, elapsedSeconds);
    UpdateHeading();
}

void Agent::ApplySteering(double elapsedSeconds)
{
    // Unit mass: acceleration equals the combined, limited steering force.
    mVelocityX += mSteeringForce.x * elapsedSeconds;
    mVelocityY += mSteeringForce.y * elapsedSeconds;
    Limit(mVelocityX, mVelocityY, mMaxSpeed);
}

void Agent::Move(const GridState& state, double elapsedSeconds)
{
    const AgentPosition next{mPosition.x + mVelocityX * elapsedSeconds, mPosition.y + mVelocityY * elapsedSeconds};
    if (CanMove(state, mPosition, next))
    {
        mPosition = next;
    }
    else
    {
        SlideAlongObstacles(state, next);
    }
}

void Agent::SlideAlongObstacles(const GridState& state, AgentPosition next)
{
    const AgentPosition nextX{next.x, mPosition.y};
    if (CanMove(state, mPosition, nextX))
    {
        mPosition.x = next.x;
    }
    else
    {
        mVelocityX = 0.0;
    }
    const AgentPosition nextY{mPosition.x, next.y};
    if (CanMove(state, mPosition, nextY))
    {
        mPosition.y = next.y;
    }
    else
    {
        mVelocityY = 0.0;
    }
}

void Agent::UpdateHeading()
{
    if (std::hypot(mVelocityX, mVelocityY) > minimumHeadingSpeed)
    {
        mHeading = std::atan2(mVelocityY, mVelocityX);
    }
}

AgentDirection Agent::GetDirection() const
{
    return {std::cos(mHeading), std::sin(mHeading)};
}

} // namespace MassMotion
