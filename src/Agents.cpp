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

void Limit(Vector2& vector, float maximum)
{
    const float length = std::hypot(vector.x, vector.y);
    if (length > maximum)
    {
        vector.x *= maximum / length;
        vector.y *= maximum / length;
    }
}

bool IsFree(const GridState& state, Vector2 position)
{
    return position.x >= 0.0f && position.y >= 0.0f && position.x < static_cast<float>(state.width) &&
           position.y < static_cast<float>(state.height) &&
           !state.obstacles[static_cast<std::size_t>(position.y) * state.width + static_cast<std::size_t>(position.x)];
}

bool CanMove(const GridState& state, Vector2 from, Vector2 to)
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

Vector2 FlowDirection(const GridState& state, Vector2 position)
{
    constexpr float diagonal = 1.0f / std::numbers::sqrt2_v<float>;
    static constexpr Vector2 directions[] = {
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
    if (mAgents.empty() || elapsedSeconds == 0.0f)
    {
        return;
    }

    if (!mProximitySearch || !mProximitySearch->MatchesGridSize(state.width, state.height))
    {
        mProximitySearch.emplace(separationRadius, state.width, state.height);
    }
    while (elapsedSeconds > 0.0f)
    {
        const float step = std::min(elapsedSeconds, maxIntegrationStep);
        SimulateStep(state, *mProximitySearch, step);
        elapsedSeconds = std::max(0.0f, elapsedSeconds - step);
    }
}

void Agents::SimulateStep(const GridState& state, ProximitySearch& proximitySearch, float elapsedSeconds)
{
    MM_PROFILE_SCOPE("Agents::SimulateStep");
    proximitySearch.Update(*this);

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
        Limit(agent.steeringForce, maxForce);
    }
}

Vector2 Agents::GetDesiredFlowDirection(const State& agent, const GridState& state)
{
    auto direction = FlowDirection(state, agent.position);
    const Vector2 predicted{agent.position.x + agent.velocity.x * flowPredictionTime,
                            agent.position.y + agent.velocity.y * flowPredictionTime};
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

Vector2 Agents::CalculateFlowFollowingForce(const State& agent, const GridState& state)
{
    const auto direction = GetDesiredFlowDirection(agent, state);
    return {(direction.x * agent.maxSpeed - agent.velocity.x) / steeringResponseTime,
            (direction.y * agent.maxSpeed - agent.velocity.y) / steeringResponseTime};
}

Vector2 Agents::CalculateSeparationForce(const State& agent, std::span<const std::size_t> neighbors,
                                         float maxForce) const
{
    Vector2 repulsion;
    for (const std::size_t neighborIndex : neighbors)
    {
        const auto force = CalculateRepulsionFrom(agent, mAgents[neighborIndex]);
        repulsion.x += force.x;
        repulsion.y += force.y;
    }
    return {repulsion.x * separationWeight * maxForce, repulsion.y * separationWeight * maxForce};
}

Vector2 Agents::CalculateRepulsionFrom(const State& agent, const State& neighbor)
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
            agent.velocity = {};
            continue;
        }

        agent.velocity.x += agent.steeringForce.x * elapsedSeconds;
        agent.velocity.y += agent.steeringForce.y * elapsedSeconds;
        Limit(agent.velocity, agent.maxSpeed);
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

        const Vector2 next{agent.position.x + agent.velocity.x * elapsedSeconds,
                           agent.position.y + agent.velocity.y * elapsedSeconds};
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

void Agents::SlideAlongObstacles(State& agent, const GridState& state, Vector2 next)
{
    const Vector2 nextX{next.x, agent.position.y};
    if (CanMove(state, agent.position, nextX))
    {
        agent.position.x = next.x;
    }
    else
    {
        agent.velocity.x = 0.0f;
    }
    const Vector2 nextY{agent.position.x, next.y};
    if (CanMove(state, agent.position, nextY))
    {
        agent.position.y = next.y;
    }
    else
    {
        agent.velocity.y = 0.0f;
    }
}

void Agents::UpdateHeading()
{
    for (auto& agent : mAgents)
    {
        if (std::hypot(agent.velocity.x, agent.velocity.y) > minimumHeadingSpeed)
        {
            agent.heading = std::atan2(agent.velocity.y, agent.velocity.x);
        }
    }
}

Vector2 Agents::GetDirection(std::size_t agentIndex) const
{
    const float heading = mAgents[agentIndex].heading;
    return {std::cos(heading), std::sin(heading)};
}

} // namespace MassMotion
