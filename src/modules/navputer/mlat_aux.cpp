#include "mlat_aux.hpp"

#include <cfloat>
#include <cmath>
#include <utility>

using namespace time_literals;

namespace
{

// solve cadence and beacon freshness
constexpr hrt_abstime kUpdateInterval = 500_ms;
constexpr hrt_abstime kMaxBeaconAge = 1_s;

// geometry gate
constexpr float kMaxHdop = 5.0f;

// MLAT solver tuning (ported from lion::filters::MLAT / ArduPilot AP_NavEKF3_RngBcnFusion SolveMlat)
constexpr float kLocalAnchorOffset = 1000.f;           // m
constexpr float kBasinRadius2 = 50.f * 50.f;           // m^2
constexpr float kResidualEpsilon = 10.f;               // m^2
constexpr float kMinBeaconSeparation2 = 100.f * 100.f; // m^2
constexpr float kMaxResidual2 = 255.f * 255.f;         // m^2
constexpr float kLearningRate = 0.1f;
constexpr int kMaxIterations = 300;
constexpr float kTolerance = 1.0f; // m

// AGP publication: matches EKF2_AGP0_ID in the 10046_navput_quadx airframe, i.e. the
// already-enabled slot 0 (Navputer::Navputer() hardcodes _fc.agp[0].enabled = true).
constexpr uint8_t kAgpId = 111;

} // namespace

float MlatAux::squaredNorm(const matrix::Vector2f &v)
{
	return v(0) * v(0) + v(1) * v(1);
}

int MlatAux::countDistinctBeacons(const BeaconInput *inputs, int num_inputs)
{
	bool counted[kMaxBeacons] {};
	int distinct = 0;

	for (int i = 0; i < num_inputs; i++) {
		if (counted[i]) {
			continue;
		}

		counted[i] = true;
		distinct++;

		for (int j = i + 1; j < num_inputs; j++) {
			if (!counted[j] && squaredNorm(inputs[i].pos - inputs[j].pos) < kMinBeaconSeparation2) {
				counted[j] = true;
			}
		}
	}

	return distinct;
}

bool MlatAux::computeBeaconCentroid(double &lat, double &lon)
{
	double lat_sum = 0.0;
	double lon_sum = 0.0;
	int count = 0;

	for (int i = 0; i < kMaxBeacons; i++) {
		if (_beacons[i].valid) {
			lat_sum += _beacons[i].lat;
			lon_sum += _beacons[i].lon;
			count++;
		}
	}

	if (count == 0) {
		return false;
	}

	lat = lat_sum / count;
	lon = lon_sum / count;
	return true;
}

float MlatAux::bearingRad(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg)
{
	const double lat1 = math::radians(lat1_deg);
	const double lat2 = math::radians(lat2_deg);
	const double dlon = math::radians(lon2_deg - lon1_deg);

	const double y = sin(dlon) * cos(lat2);
	const double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);

	return (float)atan2(y, x);
}

float MlatAux::directionalEarthRadius(double center_lat_deg, double center_lon_deg,
				       double target_lat_deg, double target_lon_deg)
{
	const double lat_rad = math::radians(center_lat_deg);
	const double sin_lat = sin(lat_rad);
	const double w = 1.0 - LatLonAlt::Wgs84::eccentricity2 * sin_lat * sin_lat;
	const double n = LatLonAlt::Wgs84::equatorial_radius / sqrt(w); // prime-vertical (E-W) radius
	const double m = LatLonAlt::Wgs84::meridian_radius_of_curvature_numerator / (w * sqrt(w)); // meridian (N-S) radius

	const bool same_point = fabs(target_lat_deg - center_lat_deg) < 1e-9 && fabs(target_lon_deg - center_lon_deg) < 1e-9;

	if (same_point) {
		return (float)sqrt(m * n);
	}

	const double bearing = (double)bearingRad(center_lat_deg, center_lon_deg, target_lat_deg, target_lon_deg);
	const double cos_b = cos(bearing);
	const double sin_b = sin(bearing);

	return (float)((m * n) / (n * cos_b * cos_b + m * sin_b * sin_b));
}

float MlatAux::distortionFactor(double lat, double lon) const
{
	const double ref_lat = _projection.getProjectionReferenceLat();
	const double ref_lon = _projection.getProjectionReferenceLon();
	const double directional_radius = (double)directionalEarthRadius(ref_lat, ref_lon, lat, lon);

	return (float)(CONSTANTS_RADIUS_OF_EARTH / directional_radius);
}

bool MlatAux::selectProjectionCenter(const navput_local_position_s &local_pos, matrix::Vector2d &center_lat_lon,
				      bool &centered_on_known_point)
{
	if (local_pos.xy_valid && local_pos.xy_global) {
		MapProjection ref_projection;
		ref_projection.initReference(local_pos.ref_lat, local_pos.ref_lon);

		double lat;
		double lon;
		ref_projection.reproject(local_pos.x, local_pos.y, lat, lon);

		center_lat_lon = matrix::Vector2d(lat, lon);
		centered_on_known_point = true;
		return true;
	}

	if (_has_last_solution) {
		center_lat_lon = _last_solution_lat_lon;
		centered_on_known_point = true;
		return true;
	}

	double lat;
	double lon;

	if (!computeBeaconCentroid(lat, lon)) {
		return false;
	}

	center_lat_lon = matrix::Vector2d(lat, lon);
	centered_on_known_point = false;
	return true;
}

bool MlatAux::calcHdop(const matrix::Vector2f &from,
			const BeaconInput *inputs,
			int num_inputs,
			float &hdop,
			matrix::SquareMatrix<float, 2> *geometry_covariance)
{
	float hxx = 0.f;
	float hyy = 0.f;
	float hxy = 0.f;
	int valid = 0;

	for (int i = 0; i < num_inputs; i++) {
		const matrix::Vector2f delta = inputs[i].pos - from;
		const float norm = delta.norm();

		if (norm < 0.1f) {
			continue;
		}

		const float ux = delta(0) / norm;
		const float uy = delta(1) / norm;
		hxx += ux * ux;
		hyy += uy * uy;
		hxy += ux * uy;
		valid++;
	}

	if (valid < 3) {
		return false;
	}

	const float det = hxx * hyy - hxy * hxy;

	if (det < 1e-6f) {
		return false;
	}

	const float q_nn = hyy / det;
	const float q_ee = hxx / det;
	const float q_ne = -hxy / det;

	hdop = sqrtf(q_nn + q_ee);

	if (geometry_covariance) {
		(*geometry_covariance)(0, 0) = q_nn;
		(*geometry_covariance)(0, 1) = q_ne;
		(*geometry_covariance)(1, 0) = q_ne;
		(*geometry_covariance)(1, 1) = q_ee;
	}

	return true;
}

void MlatAux::solveCandidate(Candidate &candidate, const BeaconInput *inputs, int num_inputs)
{
	matrix::Vector2f solution = candidate.anchor;
	float residual = 1e10f;

	for (int iter = 0; iter < kMaxIterations; iter++) {
		float gx = 0.f;
		float gy = 0.f;
		float sum_sq_err = 0.f;
		int used = 0;

		for (int i = 0; i < num_inputs; i++) {
			const matrix::Vector2f delta = solution - inputs[i].pos;
			const float estimated = delta.norm();

			if (estimated > 0.f) {
				const float err = estimated - inputs[i].range;
				const float err_deriv = 2.f * err;
				gx += err_deriv * delta(0) / estimated;
				gy += err_deriv * delta(1) / estimated;
				sum_sq_err += err * err;
				used++;
			}
		}

		if (used == 0) {
			break;
		}

		residual = sum_sq_err / used;

		const matrix::Vector2f next(solution(0) - kLearningRate * gx, solution(1) - kLearningRate * gy);
		const bool converged = fabsf(next(0) - solution(0)) < kTolerance && fabsf(next(1) - solution(1)) < kTolerance;
		solution = next;

		if (converged) {
			break;
		}
	}

	candidate.residual = fsqrt(residual);

	if (residual > kMaxResidual2) {
		candidate.valid = false;
		candidate.votes = 0;
		return;
	}

	candidate.pos = solution;
	candidate.valid = true;
	candidate.votes = 1;
}

void MlatAux::buildLocalAnchors(Candidate (&candidates)[kNumSeeds], const matrix::Vector2f &last_pos)
{
	candidates[0].anchor = last_pos;
	candidates[1].anchor = last_pos + matrix::Vector2f(kLocalAnchorOffset, 0.f); // N
	candidates[2].anchor = last_pos + matrix::Vector2f(0.f, kLocalAnchorOffset); // E
	candidates[3].anchor = last_pos + matrix::Vector2f(-kLocalAnchorOffset, 0.f); // S
	candidates[4].anchor = last_pos + matrix::Vector2f(0.f, -kLocalAnchorOffset); // W
}

void MlatAux::buildGlobalAnchors(Candidate (&candidates)[kNumSeeds], const BeaconInput *inputs, int num_inputs)
{
	float min_n = FLT_MAX;
	float max_n = -FLT_MAX;
	float min_e = FLT_MAX;
	float max_e = -FLT_MAX;

	for (int i = 0; i < num_inputs; i++) {
		const float n = inputs[i].pos(0);
		const float e = inputs[i].pos(1);
		const float r = inputs[i].range;
		min_n = fminf(min_n, n - r);
		max_n = fmaxf(max_n, n + r);
		min_e = fminf(min_e, e - r);
		max_e = fmaxf(max_e, e + r);
	}

	candidates[0].anchor = matrix::Vector2f(min_n, min_e);
	candidates[1].anchor = matrix::Vector2f(min_n, max_e);
	candidates[2].anchor = matrix::Vector2f(max_n, min_e);
	candidates[3].anchor = matrix::Vector2f(max_n, max_e);
	candidates[4].anchor = matrix::Vector2f((min_n + max_n) * 0.5f, (min_e + max_e) * 0.5f);
}

void MlatAux::mergeBasins(Candidate (&candidates)[kNumSeeds])
{
	for (int i = 0; i < kNumSeeds; i++) {
		if (!candidates[i].valid) {
			continue;
		}

		for (int j = 0; j < kNumSeeds; j++) {
			if (i == j || !candidates[j].valid) {
				continue;
			}

			if (squaredNorm(candidates[i].pos - candidates[j].pos) < kBasinRadius2) {
				const uint8_t votes = candidates[i].votes + candidates[j].votes;

				if (candidates[i].residual < candidates[j].residual) {
					candidates[j] = candidates[i];
				}

				candidates[j].votes = votes;
				candidates[i].valid = false;
				break;
			}
		}
	}
}

bool MlatAux::evaluate(Candidate (&candidates)[kNumSeeds], bool have_last_pos, const matrix::Vector2f &last_pos,
			Candidate &solution)
{
	mergeBasins(candidates);

	int best = -1;
	int second = -1;

	for (int i = 0; i < kNumSeeds; i++) {
		if (!candidates[i].valid) {
			continue;
		}

		if (best < 0 || candidates[i].residual < candidates[best].residual) {
			second = best;
			best = i;

		} else if (second < 0 || candidates[i].residual < candidates[second].residual) {
			second = i;
		}
	}

	if (best < 0) {
		return false;
	}

	if (second < 0) {
		solution = candidates[best];
		return true;
	}

	if (candidates[second].votes > candidates[best].votes + 1) {
		std::swap(best, second);
	}

	if (fabsf(candidates[best].residual - candidates[second].residual) <= kResidualEpsilon) {
		if (!have_last_pos) {
			return false; // ambiguous cold-start fix, reject rather than guess
		}

		const float dist_best = squaredNorm(candidates[best].pos - last_pos);
		const float dist_second = squaredNorm(candidates[second].pos - last_pos);
		solution = (dist_second < dist_best) ? candidates[second] : candidates[best];
		return true;
	}

	solution = candidates[best];
	return true;
}

void MlatAux::updateBeaconStore()
{
	ranging_beacon_s beacon;

	if (!_ranging_beacon_sub.update(&beacon)) {
		return;
	}

	int slot = -1;
	int oldest = 0;

	for (int i = 0; i < kMaxBeacons; i++) {
		if (_beacons[i].valid && _beacons[i].beacon_id == beacon.beacon_id) {
			slot = i;
			break;
		}

		if (!_beacons[i].valid && slot < 0) {
			slot = i;
		}

		if (_beacons[i].timestamp_sample < _beacons[oldest].timestamp_sample) {
			oldest = i;
		}
	}

	if (slot < 0) {
		slot = oldest; // table full and beacon_id unseen: evict the oldest entry
	}

	_beacons[slot].valid = true;
	_beacons[slot].beacon_id = beacon.beacon_id;
	_beacons[slot].lat = beacon.lat;
	_beacons[slot].lon = beacon.lon;
	_beacons[slot].alt = beacon.alt;
	_beacons[slot].range = beacon.range;
	_beacons[slot].range_accuracy = beacon.range_accuracy;
	_beacons[slot].timestamp_sample = beacon.timestamp_sample;
}

bool MlatAux::solve()
{
	navput_local_position_s local_pos;

	if (!_local_position_sub.copy(&local_pos)) {
		return false;
	}

	matrix::Vector2d center_lat_lon;
	bool have_last_pos = false;

	if (!selectProjectionCenter(local_pos, center_lat_lon, have_last_pos)) {
		return false; // no beacons seen yet, nothing to center the projection on
	}

	// re-centered every cycle so that distortionFactor() stays
	// an accurate proxy for the true vehicle-to-beacon geometry
	_projection.initReference(center_lat_lon(0), center_lat_lon(1), hrt_absolute_time());

	const matrix::Vector2f last_pos(0.f, 0.f); // _projection is centered exactly on last_pos this cycle
	const bool have_own_alt = local_pos.z_global;
	const float own_alt = local_pos.ref_alt - local_pos.z;

	const hrt_abstime now = hrt_absolute_time();

	BeaconInput inputs[kMaxBeacons];
	int num_inputs = 0;
	uint64_t latest_timestamp_sample = 0;

	for (int i = 0; i < kMaxBeacons; i++) {
		const BeaconEntry &beacon = _beacons[i];

		if (!beacon.valid || (now - beacon.timestamp_sample) > kMaxBeaconAge) {
			continue;
		}

		BeaconInput &input = inputs[num_inputs];
		float x;
		float y;
		_projection.project(beacon.lat, beacon.lon, x, y);
		input.pos = matrix::Vector2f(x, y);

		float range = beacon.range;

		if (have_own_alt) {
			const float dz = own_alt - beacon.alt;

			if (fabsf(dz) < beacon.range) {
				range = sqrtf(beacon.range * beacon.range - dz * dz);
			}
		}

		const float distortion = distortionFactor(beacon.lat, beacon.lon);
		range *= distortion;

		input.range = range;
		input.range_accuracy = beacon.range_accuracy;
		input.timestamp_sample = beacon.timestamp_sample;
		latest_timestamp_sample = math::max(latest_timestamp_sample, beacon.timestamp_sample);
		num_inputs++;
	}

	const int min_required = have_last_pos ? 2 : 3;

	if (countDistinctBeacons(inputs, num_inputs) < min_required) {
		return false;
	}

	const matrix::Vector2f best_guess = have_last_pos ? last_pos : (inputs[0].pos + inputs[num_inputs - 1].pos) * 0.5f;

	float hdop = kMaxHdop;

	if (!calcHdop(best_guess, inputs, num_inputs, hdop) || hdop > kMaxHdop) {
		return false;
	}

	Candidate candidates[kNumSeeds];

	if (have_last_pos) {
		buildLocalAnchors(candidates, last_pos);

	} else {
		buildGlobalAnchors(candidates, inputs, num_inputs);
	}

	for (int i = 0; i < kNumSeeds; i++) {
		solveCandidate(candidates[i], inputs, num_inputs);
	}

	Candidate solution;

	if (!evaluate(candidates, have_last_pos, last_pos, solution)) {
		return false;
	}

	float solution_hdop = kMaxHdop;
	matrix::SquareMatrix<float, 2> geometry_covariance{};
	if (!calcHdop(solution.pos, inputs, num_inputs, solution_hdop, &geometry_covariance))
	{
		return false;
	}

	double lat;
	double lon;
	_projection.reproject(solution.pos(0), solution.pos(1), lat, lon);

	_has_last_solution = true;
	_last_solution_lat_lon = matrix::Vector2d(lat, lon);

	const float eph = calculateMlatEph(inputs, num_inputs, solution.pos, geometry_covariance);

	aux_global_position_s agp{};
	agp.timestamp_sample = latest_timestamp_sample;
	agp.id = kAgpId;
	agp.source = aux_global_position_s::SOURCE_PSEUDOLITES;
	agp.lat = lat;
	agp.lon = lon;
	agp.alt = have_own_alt ? own_alt : NAN;
	agp.eph = eph;
	agp.epv = have_own_alt ? local_pos.epv : NAN;
	agp.lat_lon_reset_counter = 0;
	agp.timestamp = hrt_absolute_time();

	_aux_global_position_pub.publish(agp);

	return true;
}

float MlatAux::calculateMlatEph(const BeaconInput* const inputs,
				const int num_inputs,
				const matrix::Vector2f& solution_pos,
				const matrix::SquareMatrix<float, 2>& geometry_covariance) const
{
	// Calculate stddev for mlat
	float sum_sq_residual = 0.f;
	int used_ranges = 0;

	for (int i = 0; i < num_inputs; ++i)
	{
		const matrix::Vector2f delta = solution_pos - inputs[i].pos;
		const float estimated_range = delta.norm();

		if (estimated_range <= 0.f)
		{
			continue;
		}

		const float residual = estimated_range - inputs[i].range;

		sum_sq_residual += residual * residual;
		used_ranges++;
	}

	// conservative sigma_beac
	float sigma_beac = 0.f;
	for (int i = 0; i < num_inputs; ++i)
	{
		if (PX4_ISFINITE(inputs[i].range_accuracy)
			&& inputs[i].range_accuracy > 0.f)
		{
			sigma_beac = math::max(sigma_beac, inputs[i].range_accuracy);
		}
	}

	const int used_axis = 2; // North and East
	const int degrees_of_freedom = used_ranges - used_axis;

	float fit_variance = 0.f;
	const float expected_variance = math::sq(sigma_beac);
	if (degrees_of_freedom > 0)
	{
		fit_variance = sum_sq_residual / degrees_of_freedom;
	}
	const float range_variance = math::max(expected_variance, fit_variance);
	const float sigma_dyn = sqrtf(range_variance);

	const float q_nn = geometry_covariance(0, 0);
	const float q_ee = geometry_covariance(1, 1);

	const float sigma_n = sigma_dyn * sqrtf(q_nn);
	const float sigma_e = sigma_dyn * sqrtf(q_ee);
	const float eph = sqrtf(sigma_n * sigma_n + sigma_e * sigma_e);

	PX4_DEBUG("MLAT: SSR=%.1f dof=%d sigma_range=%.1f "
		"sigma_n=%.1f sigma_e=%.1f eph=%.1f",
		static_cast<double>(sum_sq_residual),
		degrees_of_freedom,
		static_cast<double>(sigma_dyn),
		static_cast<double>(sigma_n),
		static_cast<double>(sigma_e),
		static_cast<double>(eph));


	return eph;
}

void MlatAux::update()
{
	updateBeaconStore();

	if (hrt_elapsed_time(&_last_run) < kUpdateInterval) {
		return;
	}

	_last_run = hrt_absolute_time();

	solve();
}
