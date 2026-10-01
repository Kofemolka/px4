#include "spoofing_profiles.hpp"

#include <mathlib/mathlib.h>

#include <cfloat>

namespace GnssSpoofing
{

// h(u) = 10 * u^3 - 15 * u^4 + 6 * u^5
float rampQuintic(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const float progress3 = progress * progress * progress;
	const float progress4 = progress3 * progress;
	const float progress5 = progress4 * progress;
	return 10.f * progress3 - 15.f * progress4 + 6.f * progress5;
}

// h'(u) = 30 * u^2 * (1 - u)^2
float rampQuinticDerivative(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const float progress2 = progress * progress;
	const float remaining = 1.f - progress;
	const float remaining2 = remaining * remaining;
	return 30.f * progress2 * remaining2;
}

// h(u) = u - 6u^3 + 8u^4 - 3u^5
float rampQuinticCarryOffInitial(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const float progress3 = progress * progress * progress;
	const float progress4 = progress3 * progress;
	const float progress5 = progress4 * progress;
	return progress - 6.f * progress3 + 8.f * progress4 - 3.f * progress5;
}

// h'(u) = 1 - 18u^2 + 32u^3 - 15u^4
float rampQuinticDerivativeCarryOffInitial(const float u)
{
	const float progress = math::constrain(u, 0.f, 1.f);
	const float progress2 = progress * progress;
	const float progress3 = progress2 * progress;
	const float progress4 = progress3 * progress;
	return 1.f - 18.f * progress2 + 32.f * progress3 - 15.f * progress4;
}

GradualOffsetSample gradualOffset(const matrix::Vector3f& target_offset_ned,
				  const float maximum_speed_m_s,
				  const float current_progress,
				  const float dt_s)
{
	const float distance_m = target_offset_ned.norm();

	if (distance_m <= 0.f || maximum_speed_m_s <= 0.f || dt_s < 0.f)
	{
		return {};
	}

	const float duration_s = kMaximumRampSlope * distance_m / maximum_speed_m_s;
	const float progress = math::min(current_progress + dt_s / duration_s, 1.f);
	const float position_fraction = rampQuintic(progress);
	const float velocity_fraction = rampQuinticDerivative(progress);

	return GradualOffsetSample
	{
		.position_offset_ned = target_offset_ned * position_fraction,
		.velocity_offset_ned = target_offset_ned * velocity_fraction / duration_s,
		.progress = progress,
		.duration_s = duration_s
	};
}

GradualCarryOffSample gradualCarryOff(const matrix::Vector3f& start_position_ned,
				      const matrix::Vector3f& start_velocity_ned,
				      const matrix::Vector3f& target_position_ned,
				      const float maximum_speed_m_s,
				      const float current_progress,
				      const float dt_s)
{
	const matrix::Vector3f path_displacement = target_position_ned - start_position_ned;
	const float distance_m = path_displacement.norm();
	const float available_added_speed_m_s = maximum_speed_m_s - start_velocity_ned.norm();

	if (distance_m <= FLT_EPSILON || maximum_speed_m_s <= 0.f
	    || available_added_speed_m_s <= 0.f || dt_s < 0.f)
	{
		return {};
	}

	const float duration_s = kMaximumRampSlope * distance_m / available_added_speed_m_s;
	const float progress = math::min(current_progress + dt_s / duration_s, 1.f);
	const float position_fraction = rampQuintic(progress);
	const float velocity_fraction = rampQuinticDerivative(progress);
	const float initial_velocity_fraction = rampQuinticCarryOffInitial(progress);
	const float initial_velocity_rate_fraction = rampQuinticDerivativeCarryOffInitial(progress);

	return GradualCarryOffSample
	{
		.position_ned = start_position_ned
			+ path_displacement * position_fraction
			+ start_velocity_ned * (duration_s * initial_velocity_fraction),
		.velocity_ned = path_displacement * (velocity_fraction / duration_s)
			+ start_velocity_ned * initial_velocity_rate_fraction,
		.progress = progress,
		.duration_s = duration_s
	};
}

} // namespace GnssSpoofing
