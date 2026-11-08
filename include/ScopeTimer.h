#ifndef MASSMOTION_SCOPE_TIMER_H
#define MASSMOTION_SCOPE_TIMER_H

#ifdef MASSMOTION_ENABLE_PROFILING

#include <chrono>
#include <cstdio>

namespace MassMotion
{

class ScopeTimer
{
  public:
    // The name must remain valid until the measured scope ends; string literals are suitable.
    explicit ScopeTimer(const char* name) : mName(name), mStart(std::chrono::steady_clock::now())
    {
    }

    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;

    ~ScopeTimer()
    {
        const auto elapsed = std::chrono::steady_clock::now() - mStart;
        const double microseconds = std::chrono::duration<double, std::micro>(elapsed).count();
        std::fprintf(stderr, "[profile] %s: %.3f us\n", mName, microseconds);
    }

  private:
    const char* mName;
    std::chrono::steady_clock::time_point mStart;
};

} // namespace MassMotion

// Use a separate block for each measured scope.
#define MM_PROFILE_SCOPE(name) ::MassMotion::ScopeTimer scopeTimer(name)

#else

#define MM_PROFILE_SCOPE(name)

#endif

#endif // MASSMOTION_SCOPE_TIMER_H
