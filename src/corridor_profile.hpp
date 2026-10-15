#pragma once
#include "corridor_planner.hpp"
#include <chrono>

namespace corridor::detail {
using ProfileClock = std::chrono::steady_clock;
struct ProfileContext;
inline thread_local ProfileContext *activeProfile = nullptr;
struct ProfileContext {
    PlanningTimings *timings;
    ProfileContext *previous;
    PlanningStage stage = PlanningStage::Other;
    ProfileClock::time_point mark = ProfileClock::now();
    explicit ProfileContext(PlanningTimings *value) : timings(value), previous(activeProfile) {
        activeProfile = value ? this : nullptr;
    }
    void enter(PlanningStage next) {
        const auto now = ProfileClock::now();
        if (timings)
            timings->stage_seconds[static_cast<size_t>(stage)] +=
                std::chrono::duration<double>(now - mark).count();
        mark = now;
        stage = next;
    }
    ~ProfileContext() {
        enter(PlanningStage::Other);
        activeProfile = previous;
    }
};
struct StageScope {
    ProfileContext *context = activeProfile;
    PlanningStage previous = context ? context->stage : PlanningStage::Other;
    bool changed;
    explicit StageScope(PlanningStage next) : changed(context && next != previous) {
        if (changed)
            context->enter(next);
    }
    ~StageScope() {
        if (changed)
            context->enter(previous);
    }
};
inline PlanningTimings *profile() {
    return activeProfile ? activeProfile->timings : nullptr;
}
} // namespace corridor::detail
