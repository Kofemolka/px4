#ifndef GNSS_SPOOFING_PROFILES_HPP
#define GNSS_SPOOFING_PROFILES_HPP

#include <matrix/Vector3.hpp>

namespace GnssSpoofing
{

constexpr float kMaximumRampSlope = 1.875f;

float rampQuintic(float progress);
float rampQuinticDerivative(float progress);
float rampQuinticCarryOffInitial(float progress);
float rampQuinticDerivativeCarryOffInitial(float progress);

struct GradualOffsetSample
{
	matrix::Vector3f position_offset_ned{};
	matrix::Vector3f velocity_offset_ned{};
	float progress{0.f};
	float duration_s{0.f};
};

GradualOffsetSample gradualOffset(const matrix::Vector3f& target_offset_ned,
				  float maximum_speed_m_s,
				  float current_progress,
				  float dt_s);

struct GradualCarryOffSample {
	matrix::Vector3f position_ned{};
	matrix::Vector3f velocity_ned{};
	float progress{0.f};
	float duration_s{0.f};
};

GradualCarryOffSample gradualCarryOff(const matrix::Vector3f& start_position_ned,
				      const matrix::Vector3f& start_velocity_ned,
				      const matrix::Vector3f& target_position_ned,
				      float maximum_speed_m_s,
				      float current_progress,
				      float dt_s);

} // namespace GnssSpoofing

#endif // GNSS_SPOOFING_PROFILES_HPP
