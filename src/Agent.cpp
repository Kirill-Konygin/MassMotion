#include "Agent.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace MassMotion
{

bool CanStep(const GridState& state, std::size_t x, std::size_t y, std::size_t targetX, std::size_t targetY)
{
    if (x >= state.width || y >= state.height || targetX >= state.width || targetY >= state.height ||
        state.obstacles[y * state.width + x] || state.obstacles[targetY * state.width + targetX])
    {
        return false;
    }
    return x == targetX || y == targetY ||
           (!state.obstacles[y * state.width + targetX] && !state.obstacles[targetY * state.width + x]);
}

std::pair<int, int> CellOffset(Cell cell)
{
    static constexpr std::pair<int, int> offsets[] = {
        {0, 0}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
    };
    return offsets[static_cast<std::size_t>(cell)];
}

Agent::Agent(std::size_t x, std::size_t y, double speed) : mX(x), mY(y), mTargetX(x), mTargetY(y), mSpeed(speed)
{
    if (!std::isfinite(speed) || speed <= 0.0)
    {
        throw std::invalid_argument("Agent speed must be positive and finite");
    }
}

void Agent::Update(const GridState& state, double elapsedSeconds)
{
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
    {
        throw std::invalid_argument("Elapsed time must be finite and nonnegative");
    }

    while (true)
    {
        if (!CanStep(state, mX, mY, mTargetX, mTargetY))
        {
            mTargetX = mX;
            mTargetY = mY;
            mProgress = 0.0;
            return;
        }

        if (mX == mTargetX && mY == mTargetY)
        {
            const auto [dx, dy] = CellOffset(state.cells[mY * state.width + mX]);
            if (dx == 0 && dy == 0)
            {
                return;
            }

            if ((dx < 0 && mX == 0) || (dy < 0 && mY == 0) || (dx > 0 && mX == state.width - 1) ||
                (dy > 0 && mY == state.height - 1))
            {
                return;
            }
            const std::size_t targetX = dx < 0 ? mX - 1 : (dx > 0 ? mX + 1 : mX);
            const std::size_t targetY = dy < 0 ? mY - 1 : (dy > 0 ? mY + 1 : mY);
            if (!CanStep(state, mX, mY, targetX, targetY))
            {
                return;
            }
            mTargetX = targetX;
            mTargetY = targetY;
            PrepareCurve(state);
        }

        const double secondsToTarget = (mLength - mProgress) / mSpeed;
        if (elapsedSeconds < secondsToTarget)
        {
            mProgress += mSpeed * elapsedSeconds;
            UpdateHeading(elapsedSeconds);
            return;
        }

        mProgress = mLength;
        UpdateHeading(secondsToTarget);
        elapsedSeconds = std::max(0.0, elapsedSeconds - secondsToTarget);
        mX = mTargetX;
        mY = mTargetY;
        mProgress = 0.0;
        if (elapsedSeconds == 0.0)
        {
            return;
        }
    }
}

void Agent::PrepareCurve(const GridState& state)
{
    const double dx = static_cast<double>(mTargetX) - static_cast<double>(mX);
    const double dy = static_cast<double>(mTargetY) - static_cast<double>(mY);
    mLength = std::hypot(dx, dy);
    const AgentDirection forward{dx / mLength, dy / mLength};
    const AgentDirection startTangent = mHasHeading ? mEndTangent : forward;
    mEndTangent = forward;

    const auto [nextDX, nextDY] = CellOffset(state.cells[mTargetY * state.width + mTargetX]);
    if ((nextDX >= 0 || mTargetX > 0) && (nextDY >= 0 || mTargetY > 0) && (nextDX <= 0 || mTargetX < state.width - 1) &&
        (nextDY <= 0 || mTargetY < state.height - 1))
    {
        const std::size_t nextX = nextDX < 0 ? mTargetX - 1 : (nextDX > 0 ? mTargetX + 1 : mTargetX);
        const std::size_t nextY = nextDY < 0 ? mTargetY - 1 : (nextDY > 0 ? mTargetY + 1 : mTargetY);
        if ((nextDX != 0 || nextDY != 0) && CanStep(state, mTargetX, mTargetY, nextX, nextY))
        {
            const double nextLength = std::hypot(nextDX, nextDY);
            const double tangentX = forward.x + nextDX / nextLength;
            const double tangentY = forward.y + nextDY / nextLength;
            const double tangentLength = std::hypot(tangentX, tangentY);
            if (tangentLength > 1.0e-9)
            {
                mEndTangent = {tangentX / tangentLength, tangentY / tangentLength};
            }
        }
    }

    if (!mHasHeading)
    {
        mHeading = std::atan2(forward.y, forward.x);
        mHasHeading = true;
    }
    mStraight = std::hypot(startTangent.x - forward.x, startTangent.y - forward.y) < 1.0e-9 &&
                std::hypot(mEndTangent.x - forward.x, mEndTangent.y - forward.y) < 1.0e-9;
    // All control points stay within 0.2 cells of the centers. Together with the
    // viewer's 0.28-cell radius, the curve fits inside the step's free cells.
    constexpr double handleLength = 0.2;
    mControl1 = {static_cast<double>(mX) + 0.5 + startTangent.x * handleLength,
                 static_cast<double>(mY) + 0.5 + startTangent.y * handleLength};
    mControl2 = {static_cast<double>(mTargetX) + 0.5 - mEndTangent.x * handleLength,
                 static_cast<double>(mTargetY) + 0.5 - mEndTangent.y * handleLength};
    if (!mStraight)
    {
        // Approximate arc length so curved movement keeps the same speed.
        mDistances[0] = 0.0;
        auto previous = EvaluateCurve(0.0);
        for (std::size_t i = 1; i <= CurveSamples; ++i)
        {
            const auto point = EvaluateCurve(static_cast<double>(i) / CurveSamples);
            mDistances[i] = mDistances[i - 1] + std::hypot(point.x - previous.x, point.y - previous.y);
            previous = point;
        }
        mLength = mDistances.back();
    }
}

double Agent::CurveParameter() const
{
    if (mStraight)
    {
        return std::clamp(mProgress / mLength, 0.0, 1.0);
    }
    const auto upper = std::upper_bound(mDistances.begin(), mDistances.end(), mProgress);
    if (upper == mDistances.end())
    {
        return 1.0;
    }
    const auto index = static_cast<std::size_t>(upper - mDistances.begin());
    const double fraction = (mProgress - mDistances[index - 1]) / (mDistances[index] - mDistances[index - 1]);
    return (static_cast<double>(index - 1) + fraction) / CurveSamples;
}

AgentPosition Agent::EvaluateCurve(double t) const
{
    const double startX = static_cast<double>(mX) + 0.5;
    const double startY = static_cast<double>(mY) + 0.5;
    const double endX = static_cast<double>(mTargetX) + 0.5;
    const double endY = static_cast<double>(mTargetY) + 0.5;
    if (mStraight)
    {
        return {startX + (endX - startX) * t, startY + (endY - startY) * t};
    }
    const double u = 1.0 - t;
    return {u * u * u * startX + 3.0 * u * u * t * mControl1.x + 3.0 * u * t * t * mControl2.x + t * t * t * endX,
            u * u * u * startY + 3.0 * u * u * t * mControl1.y + 3.0 * u * t * t * mControl2.y + t * t * t * endY};
}

void Agent::UpdateHeading(double elapsedSeconds)
{
    const double t = CurveParameter();
    const double u = 1.0 - t;
    double dx = static_cast<double>(mTargetX) - static_cast<double>(mX);
    double dy = static_cast<double>(mTargetY) - static_cast<double>(mY);
    if (!mStraight)
    {
        dx = u * u * (mControl1.x - static_cast<double>(mX) - 0.5) + 2.0 * u * t * (mControl2.x - mControl1.x) +
             t * t * (static_cast<double>(mTargetX) + 0.5 - mControl2.x);
        dy = u * u * (mControl1.y - static_cast<double>(mY) - 0.5) + 2.0 * u * t * (mControl2.y - mControl1.y) +
             t * t * (static_cast<double>(mTargetY) + 0.5 - mControl2.y);
    }
    if (std::hypot(dx, dy) > 1.0e-9)
    {
        constexpr double fullTurn = 2.0 * std::numbers::pi_v<double>;
        const double difference = std::remainder(std::atan2(dy, dx) - mHeading, fullTurn);
        const double maxTurn = fullTurn * elapsedSeconds;
        mHeading = std::remainder(mHeading + std::clamp(difference, -maxTurn, maxTurn), fullTurn);
    }
}

AgentPosition Agent::GetPosition() const
{
    if (mX == mTargetX && mY == mTargetY)
    {
        return {static_cast<double>(mX) + 0.5, static_cast<double>(mY) + 0.5};
    }
    return EvaluateCurve(CurveParameter());
}

AgentDirection Agent::GetDirection() const
{
    return {std::cos(mHeading), std::sin(mHeading)};
}

} // namespace MassMotion
