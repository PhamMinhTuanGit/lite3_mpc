#pragma once

constexpr bool PlannedContactFromSwingState(float swing_state) noexcept
{
    return !(swing_state > 0.0f);
}
static_assert(!PlannedContactFromSwingState(0.25f), "positive swing state must be swing");
static_assert(PlannedContactFromSwingState(0.0f), "zero swing state must be stance");
