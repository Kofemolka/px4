#include "optical_flow.hpp"

#include <float.h>

using matrix::Dcmf;
using matrix::Vector2f;
using matrix::Vector3f;

namespace
{
// TODO: move to params (NPT_OF_*) once the approach is validated
constexpr float kMinFlowRate = 0.05f;   ///< rad/s, min gyro-compensated flow magnitude to trust its direction
constexpr float kMinGroundSpeed = 0.5f; ///< m/s, min EKF horizontal speed to trust its direction
constexpr float kFakeVelVar = 10.f;      ///< (m/s)^2, conservative fixed variance for the fake velocity observation
}

OpticalFlow::OpticalFlow()
{

}

void OpticalFlow::update(Ekf& _ekf)
{
	// TODO: custom toggle

	vehicle_optical_flow_s optical_flow;

	if (_vehicle_optical_flow_sub.update(&optical_flow)) {

		// if (!_ekf.getFusionControlHandle()->of.intended()) {
		// 	return;
		// }

		const float dt = 1e-6f * (float)optical_flow.integration_timespan_us;

		if (dt <= FLT_EPSILON) {
			return;
		}

		// NOTE: same sign convention as Navputer::UpdateFlowSample(): the EKF assumes positive LOS rate
		// is produced by a RH rotation of the image about the sensor axis.
		const Vector2f flow_rate = Vector2f(-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]) / dt;
		const Vector3f gyro_rate = Vector3f(-optical_flow.delta_angle[0], -optical_flow.delta_angle[1],
						     -optical_flow.delta_angle[2]) / dt;

		// remove rotation-induced flow, leaving the translation-induced component
		const Vector2f flow_compensated = flow_rate - gyro_rate.xy();

		// check min magnitude
		if (!flow_compensated.isAllFinite() || flow_compensated.norm() < kMinFlowRate) {
			return;
		}

		// get IMU velocity vector
		const Vector2f vel_ne{_ekf.getVelocity().xy()};
		const float speed = vel_ne.norm();

		if (speed < kMinGroundSpeed) {
			return;
		}

		// get OF normal vector - gyro compensated, rotated body -> NED (assumes near-level camera)
		const Vector2f dir_body = Vector2f(-flow_compensated(1), flow_compensated(0)).normalized();
		const Dcmf R{_ekf.getQuaternion()};
		const Vector2f dir_ne = Vector2f(R(0, 0) * dir_body(0) + R(0, 1) * dir_body(1),
						  R(1, 0) * dir_body(0) + R(1, 1) * dir_body(1)).normalized();

		// produce OF velocity vector: direction from flow, magnitude from the EKF's own estimate
		const Vector2f vel_fake = dir_ne * speed;

		// fuse velocity observation
		const auxVelSample sample {
			.time_us = optical_flow.timestamp_sample - optical_flow.integration_timespan_us / 2,
			.vel = vel_fake,
			.velVar = Vector2f(kFakeVelVar, kFakeVelVar),
		};

		_ekf.setAuxVelData(sample);
	}
}
