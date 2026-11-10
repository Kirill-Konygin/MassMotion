#include "Agents.h"
#include "ProximitySearch.h"
#include "ScopeTimer.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace MassMotion
{

constexpr float steeringResponseTime = 0.2f;
constexpr float flowPredictionTime = 0.15f;
constexpr float separationRadius = 1.0f;
constexpr float separationWeight = 0.8f;
constexpr float maxIntegrationStep = 1.0f / 30.0f;
constexpr float coincidentDistanceSquared = 1.0e-12f;
constexpr float minimumHeadingSpeed = 1.0e-6f;

void Limit(float& x, float& y, float maximum)
{
    const float length = std::hypot(x, y);
    if (length > maximum)
    {
        x *= maximum / length;
        y *= maximum / length;
    }
}

bool IsFree(const GridState& state, AgentPosition position)
{
    return position.x >= 0.0f && position.y >= 0.0f && position.x < static_cast<float>(state.width) &&
           position.y < static_cast<float>(state.height) &&
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
    constexpr float diagonal = 1.0f / std::numbers::sqrt2_v<float>;
    static constexpr AgentDirection directions[] = {
        {0.0f, 0.0f}, {0.0f, -1.0f},         {diagonal, -diagonal}, {1.0f, 0.0f},           {diagonal, diagonal},
        {0.0f, 1.0f}, {-diagonal, diagonal}, {-1.0f, 0.0f},         {-diagonal, -diagonal},
    };
    const auto cell =
        state.cells[static_cast<std::size_t>(position.y) * state.width + static_cast<std::size_t>(position.x)];
    return directions[static_cast<std::size_t>(cell)];
}

void Agents::Reserve(std::size_t capacity)
{
    mAgents.reserve(capacity);
}

std::size_t Agents::Add(std::size_t x, std::size_t y, float maxSpeed)
{
    if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0f)
    {
        throw std::invalid_argument("Agent speed must be positive and finite");
    }

    const std::size_t agentIndex = mAgents.size();
    mAgents.push_back({{static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f}, maxSpeed});
    return agentIndex;
}

void Agents::Update(const GridState& state, float elapsedSeconds)
{
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0f)
    {
        throw std::invalid_argument("Elapsed time must be finite and nonnegative");
    }
    if (mAgents.empty())
    {
        return;
    }

    ProximitySearch proximitySearch;
    while (elapsedSeconds > 0.0f)
    {
        const float step = std::min(elapsedSeconds, maxIntegrationStep);
        SimulateStep(state, proximitySearch, step);
        elapsedSeconds = std::max(0.0f, elapsedSeconds - step);
    }
}

void Agents::SimulateStep(const GridState& state, ProximitySearch& proximitySearch, float elapsedSeconds)
{
    proximitySearch.Update(*this, separationRadius);
    MM_PROFILE_SCOPE("Agents::SimulateStep");
    CalculateSteering(state, proximitySearch);
    Integrate(state, elapsedSeconds);
}

void Agents::CalculateSteering(const GridState& state, const ProximitySearch& proximitySearch)
{
    for (std::size_t i = 0; i < mAgents.size(); ++i)
    {
        auto& agent = mAgents[i];
        agent.steeringForce = {};
        if (!IsFree(state, agent.position))
        {
            continue;
        }

        const float maxForce = agent.maxSpeed / steeringResponseTime;
        const auto flowForce = CalculateFlowFollowingForce(agent, state);
        const auto separationForce = CalculateSeparationForce(agent, proximitySearch.GetNeighbors(i), maxForce);
        agent.steeringForce = {flowForce.x + separationForce.x, flowForce.y + separationForce.y};
        Limit(agent.steeringForce.x, agent.steeringForce.y, maxForce);
    }
}

AgentDirection Agents::GetDesiredFlowDirection(const State& agent, const GridState& state)
{
    auto direction = FlowDirection(state, agent.position);
    const AgentPosition predicted{agent.position.x + agent.velocityX * flowPredictionTime,
                                  agent.position.y + agent.velocityY * flowPredictionTime};
    if ((direction.x != 0.0f || direction.y != 0.0f) && CanMove(state, agent.position, predicted))
    {
        const auto predictedDirection = FlowDirection(state, predicted);
        // Enter the terminal cell before requesting a stop.
        if (predictedDirection.x != 0.0f || predictedDirection.y != 0.0f)
        {
            direction = predictedDirection;
        }
    }
    return direction;
}

Agents::SteeringForce Agents::CalculateFlowFollowingForce(const State& agent, const GridState& state)
{
    const auto direction = GetDesiredFlowDirection(agent, state);
    return {(direction.x * agent.maxSpeed - agent.velocityX) / steeringResponseTime,
            (direction.y * agent.maxSpeed - agent.velocityY) / steeringResponseTime};
}

Agents::SteeringForce Agents::CalculateSeparationForce(const State& agent, std::span<const std::size_t> neighbors,
                                                       float maxForce) const
{
    SteeringForce repulsion;
    for (const std::size_t neighborIndex : neighbors)
    {
        const auto force = CalculateRepulsionFrom(agent, mAgents[neighborIndex]);
        repulsion.x += force.x;
        repulsion.y += force.y;
    }
    return {repulsion.x * separationWeight * maxForce, repulsion.y * separationWeight * maxForce};
}

Agents::SteeringForce Agents::CalculateRepulsionFrom(const State& agent, const State& neighbor)
{
    const float dx = agent.position.x - neighbor.position.x;
    const float dy = agent.position.y - neighbor.position.y;
    const float distanceSquared = dx * dx + dy * dy;
    if (distanceSquared < coincidentDistanceSquared)
    {
        // Give coincident pairs opposite forces without dividing by zero.
        return {&agent < &neighbor ? -1.0f : 1.0f, 0.0f};
    }
    // Normalized offset weighted by 1 / distance, as in Reynolds' separation.
    return {dx / distanceSquared, dy / distanceSquared};
}

void Agents::Integrate(const GridState& state, float elapsedSeconds)
{
    ApplySteering(state, elapsedSeconds);
    Move(state, elapsedSeconds);
    UpdateHeading();
}

void Agents::ApplySteering(const GridState& state, float elapsedSeconds)
{
    for (auto& agent : mAgents)
    {
        if (!IsFree(state, agent.position))
        {
            agent.velocityX = 0.0f;
            agent.velocityY = 0.0f;
            continue;
        }

        agent.velocityX += agent.steeringForce.x * elapsedSeconds;
        agent.velocityY += agent.steeringForce.y * elapsedSeconds;
        Limit(agent.velocityX, agent.velocityY, agent.maxSpeed);
    }
}

void Agents::Move(const GridState& state, float elapsedSeconds)
{
    for (auto& agent : mAgents)
    {
        if (!IsFree(state, agent.position))
        {
            continue;
        }

        const AgentPosition next{agent.position.x + agent.velocityX * elapsedSeconds,
                                 agent.position.y + agent.velocityY * elapsedSeconds};
        if (CanMove(state, agent.position, next))
        {
            agent.position = next;
        }
        else
        {
            SlideAlongObstacles(agent, state, next);
        }
    }
}

void Agents::SlideAlongObstacles(State& agent, const GridState& state, AgentPosition next)
{
    const AgentPosition nextX{next.x, agent.position.y};
    if (CanMove(state, agent.position, nextX))
    {
        agent.position.x = next.x;
    }
    else
    {
        agent.velocityX = 0.0f;
    }
    const AgentPosition nextY{agent.position.x, next.y};
    if (CanMove(state, agent.position, nextY))
    {
        agent.position.y = next.y;
    }
    else
    {
        agent.velocityY = 0.0f;
    }
}

void Agents::UpdateHeading()
{
    for (auto& agent : mAgents)
    {
        if (std::hypot(agent.velocityX, agent.velocityY) > minimumHeadingSpeed)
        {
            agent.heading = std::atan2(agent.velocityY, agent.velocityX);
        }
    }
}

AgentDirection Agents::GetDirection(std::size_t agentIndex) const
{
    const float heading = mAgents[agentIndex].heading;
    return {std::cos(heading), std::sin(heading)};
}

} // namespace MassMotion
