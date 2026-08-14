#include "optical_flow.hpp"

#include <float.h>

using matrix::Dcmf;
using matrix::Vector2f;
using matrix::Vector3f;

namespace
{
// TODO: move to params (NPT_OF_*) once the approach is validated
constexpr float kMinFlowRate = 0.05f; ///< rad/s, min gyro-compensated flow magnitude to trust its direction
constexpr float kPosVar = 0.25f;      ///< m^2, tolerance on the fused position target
}

OpticalFlow::OpticalFlow(ModuleParams *parent) :
	ModuleParams(parent)
{

}

void OpticalFlow::update(Ekf& ekf)
{
	if (!_param_npt_fuse_ofh.get()) {
		return;
	}

	if(!ekf.global_origin_valid()) {
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
		const Dcmf R{ekf.getQuaternion()};
		const Vector2f dir_ne = Vector2f(R(0, 0) * dir_body(0) + R(0, 1) * dir_body(1),
						  R(1, 0) * dir_body(0) + R(1, 1) * dir_body(1)).normalized();

		if (!dir_ne.isAllFinite()) {
			return;
		}

		// produce an X/Y point along the flow vector: advance the current position by
		// dir_ne * speed * dt and fuse that as a position target, not a velocity observation.
		// This still uses the EKF's own speed for the step length, but fusing it as position
		// shrinks the EKF's position uncertainty directly rather than repeatedly asserting
		// confidence in its own velocity (the source of the earlier runaway).
		const Vector2f vel_ne{ekf.getVelocity().xy()};
		const float speed = vel_ne.norm();

		const Vector2f pos_target = Vector2f(ekf.getPosition().xy()) + dir_ne * speed * dt;

		double lat, lon;
		ekf.global_origin().reproject(pos_target(0), pos_target(1), lat, lon);

		// ekf.fuseOpticalFlowPosition(pos_target(0), pos_target(1), kPosVar);
		aux_global_position_s agp{};
		agp.timestamp_sample = optical_flow.timestamp_sample;
		agp.id = 112;
		agp.source = aux_global_position_s::SOURCE_VISION;
		agp.lat = lat;
		agp.lon = lon;
		agp.alt = NAN;
		agp.eph = optical_flow.quality / 3; // TODO: need param
		agp.epv = NAN;
		agp.lat_lon_reset_counter = 0;
		agp.timestamp = hrt_absolute_time();
		_aux_global_position_pub.publish(agp);
	}
}
