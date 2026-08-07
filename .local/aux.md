Current coupling: everything hangs off one line — aux_global_position_control.cpp:108-111:


if (!ekf._fc.agp[_slot].intended()) { _state = State::kStopped; return true; }
intended() is just enabled && available (common.h:287-306). If false, nothing downstream runs at all — no origin check, no reset, no fusion. GPS has the identical pattern (gps_control.cpp:46), so this isn't AGP-specific, but you only asked about AGP.

What already exists that you can reuse, unmodified:

Ekf::setEkfGlobalOrigin (ekf_helper.cpp:88-126) — sets only the origin, doesn't move the position estimate. This is goal 1, cleanly.
Ekf::resetHorizontalPositionTo (used by GPS's resetHorizontalPositionToGnss, gps_control.cpp:454-462) — resets position from a sample given an existing origin, without touching the origin itself. This is goal 2.
isOtherSourceOfHorizontalPositionAidingThan(false) (estimator_interface.cpp:633-666) — already the exact "is anything else providing horizontal position aiding" check GPS/EV/OF all use for their own reset-gating. This is goal 2's condition.
Today AGP conflates goals 1+2 into one call, resetGlobalPositionTo (ekf_helper.cpp:155-199 = origin-if-unset + position reset in one shot), invoked only from inside the kStarting/no-origin branch that's already past the intended() gate.
The one real gap: there's no state that means "did a one-time bootstrap correction but is not continuously fusing." Every reset in the current code (lines 146, 158, 177) is immediately followed by _state = kActive, and kActive is isFusing(), which is what feeds _control_status_flags.aux_gpos (the flag everything else checks for "is AGP aiding"). If goals 1/2 fire unconditionally while disabled, and you route them through the existing state machine, you'd flip aux_gpos true even though goal 3 says fusion must stay off — that would misrepresent AGP as an active aiding source to the rest of the filter and to arming checks.


# Decision
Move AGP toggling, and other fusion flags logic, into a dedicated class.
AGP will be turned on if:
* NPT_FUSE_AGP0 == 1
* Global origin is not set or
* There is no other aiding source active.
