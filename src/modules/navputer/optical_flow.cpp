#include "optical_flow.hpp"

#include <float.h>

using matrix::Dcmf;
using matrix::Vector2f;
using matrix::Vector3f;

namespace
{
// TODO: move to params (NPT_OF_*) once the approach is validated
constexpr float kMinFlowRate = 0.05f; ///< rad/s, min gyro-compensated flow magnitude to trust its direction
constexpr float kPerpVelVar = 1.f;    ///< (m/s)^2, tolerance on velocity perpendicular to the flow heading
}

OpticalFlow::OpticalFlow(ModuleParams *parent) :
	ModuleParams(parent)
{

}

void OpticalFlow::update(Ekf& _ekf)
{
	if (!_param_npt_fuse_ofh.get()) {
		return;
	}

	vehicle_optical_flow_s optical_flow;

	if (_vehicle_optical_flow_sub.update(&optical_flow)) {

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

		// get OF normal vector - gyro compensated, rotated body -> NED (assumes near-level camera)
		const Vector2f dir_body = Vector2f(-flow_compensated(1), flow_compensated(0)).normalized();
		const Dcmf R{_ekf.getQuaternion()};
		const Vector2f dir_ne{R(0, 0) * dir_body(0) + R(0, 1) * dir_body(1),
				      R(1, 0) * dir_body(0) + R(1, 1) * dir_body(1)};

		if (!dir_ne.isAllFinite() || dir_ne.norm() < FLT_EPSILON) {
			return;
		}

		// fuse velocity observation: constrain heading only, leave magnitude to the IMU
		// (avoids feeding the EKF's own speed estimate back into itself as a "measurement")
		const float heading_rad = atan2f(dir_ne(1), dir_ne(0));
		_ekf.fuseOpticalFlowHeading(heading_rad, kPerpVelVar);
	}
}
