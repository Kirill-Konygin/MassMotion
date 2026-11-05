#ifndef MASSMOTION_AGENT_H
#define MASSMOTION_AGENT_H

#include "Grid.h"

#include <array>

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

class Agent
{
  public:
    // Position is a cell coordinate; speed is in cells per second and must be positive and finite.
    Agent(std::size_t x, std::size_t y, double speed = 3.0);

    // Follows arrows between cell centers with rounded turns, stopping at Cell::None or a blocked step.
    // New directions take effect at the next cell center. Elapsed time must be finite and nonnegative.
    // The state must be a snapshot from Grid. If an active step becomes blocked, return to its start.
    void Update(const GridState& state, double elapsedSeconds);

    // Grid-space position: the center of cell (0, 0) is (0.5, 0.5).
    [[nodiscard]] AgentPosition GetPosition() const;

    // Unit facing vector. Rotation is limited to one full turn per second.
    [[nodiscard]] AgentDirection GetDirection() const;

  private:
    void PrepareCurve(const GridState& state);
    void UpdateHeading(double elapsedSeconds);
    [[nodiscard]] double CurveParameter() const;
    [[nodiscard]] AgentPosition EvaluateCurve(double parameter) const;

    static constexpr std::size_t CurveSamples = 128;
    std::size_t mX;
    std::size_t mY;
    std::size_t mTargetX;
    std::size_t mTargetY;
    double mSpeed;
    double mProgress = 0.0;
    double mLength = 1.0;
    double mHeading = 0.0;
    bool mHasHeading = false;
    bool mStraight = true;
    AgentDirection mEndTangent{1.0, 0.0};
    AgentPosition mControl1{};
    AgentPosition mControl2{};
    std::array<double, CurveSamples + 1> mDistances{};
};

} // namespace MassMotion

#endif // MASSMOTION_AGENT_H
