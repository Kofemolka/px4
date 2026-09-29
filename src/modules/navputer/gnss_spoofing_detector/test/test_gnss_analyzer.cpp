#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>

#include <mathlib/mathlib.h>

#include "gnss_analyzer.hpp"

namespace
{

constexpr uint64_t kImuPeriodUs = 4'000;
constexpr uint64_t kGnssPeriodUs = 125'000;
constexpr uint64_t kAuxPositionPeriodUs = 544'000;
constexpr uint64_t kStartTimeUs = 1'000'000;
constexpr uint64_t kWarmupDurationUs = 10'000'000;
constexpr float kNominalFigureEightPeriodS = 40.f;
constexpr float kAggressiveFigureEightPeriodS = 15.f;
constexpr float kExtremeFigureEightPeriodS = 10.f;
constexpr float kFigureEightCycleCount = 5.f;

// From vehicle_gps_position in Navputer SITL ULogs:
// eph = 0.9 m, epv = 1.78 m, s_variance_m_s = 0.4 m/s.
constexpr float kGnssHorizontalPositionStdDevM = 0.9f;
constexpr float kGnssVerticalPositionStdDevM = 1.78f;
constexpr float kGnssVelocityStdDevMS = 0.4f;

// Representative aux_global_position eph from the same logs is 57.1 m.
// Production splits its total horizontal variance equally between north and east.
constexpr float kAuxHorizontalPositionStdDevM = 57.1f / 1.41421356237f;

// Ekf::immediateLatestDeltaVelocity() reports an isotropic variance of
// (EKF2_ACC_NOISE² + max(accel bias variance)) * dt². EKF2_ACC_NOISE is 0.35 m/s²,
// and the median max accel-bias standard deviation in the 2026-09-25 ULog is 0.059 m/s².
constexpr float kImuAccelerationNoiseStdDevMS2 = 0.35f;
constexpr float kImuAccelerationBiasStdDevMS2 = 0.059f;

struct TruthSample {
	matrix::Vector3f position_ned{};
	matrix::Vector3f velocity_ned{};
	matrix::Vector3f acceleration_ned{};
};

using Trajectory = TruthSample (*)(float time_s);

class GnssAnalyzerTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_analyzer.reset(true);
		_random_generator.seed(0x4E415650);

		if (const char *trace_path = getenv("GNSS_ANALYZER_TRACE")) {
			openTraceStream(trace_path);
		}
	}

	void TearDown() override
	{
		_trace.close();
	}

	void openTraceStream(const char *trace_path)
	{
		std::string output_path{trace_path};
		const std::string test_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
		const size_t extension_position = output_path.find_last_of('.');
		const size_t directory_position = output_path.find_last_of("/\\");
		const size_t suffix_position = extension_position != std::string::npos
			&& (directory_position == std::string::npos || extension_position > directory_position)
			? extension_position : output_path.size();
		output_path.insert(suffix_position, "_" + test_name);

		_trace.open(output_path);
		ASSERT_TRUE(_trace.is_open()) << "Failed to open GNSS analyzer trace " << output_path;
		_trace << "time_s,north_m,east_m,down_m,velocity_n_m_s,velocity_e_m_s,velocity_d_m_s,"
		       << "speed_m_s,acceleration_n_m_s2,acceleration_e_m_s2,acceleration_d_m_s2,"
		       << "gnss_north_m,gnss_east_m,gnss_down_m,gnss_velocity_n_m_s,gnss_velocity_e_m_s,"
		       << "gnss_velocity_d_m_s,gnss_kf_north_m,gnss_kf_east_m,gnss_kf_down_m,"
		       << "gnss_kf_velocity_n_m_s,gnss_kf_velocity_e_m_s,gnss_kf_velocity_d_m_s,"
		       << "raw_gnss_delta_velocity_n_m_s,raw_gnss_delta_velocity_e_m_s,"
		       << "gnss_kf_delta_velocity_n_m_s,gnss_kf_delta_velocity_e_m_s,"
		       << "imu_delta_velocity_n_m_s,imu_delta_velocity_e_m_s,"
		       << "imu_gnss_residual_n_m_s,imu_gnss_residual_e_m_s,"
		       << "imu_gnss_residual_variance_n,imu_gnss_residual_variance_e,"
		       << "imu_gnss_normalized_error,gnss_kf_acceleration_noise_density_squared,"
		       << "imu_velocity_suspicion,gnss_velocity_consistency_suspicion,position_suspicion,state\n";
		_trace << std::fixed << std::setprecision(6);
	}

	void maybeTraceIteration(const float time_s,
		const TruthSample& truth,
		const matrix::Vector3f& gnss_position_ned,
		const matrix::Vector3f& gnss_velocity_ned,
		const GnssAnalyzerTypes::GnssAnalyzerExtendedState& state)
	{
		if (_trace.is_open())
		{
			_trace << time_s << ','
			       << truth.position_ned(0) << ',' << truth.position_ned(1) << ',' << truth.position_ned(2) << ','
			       << truth.velocity_ned(0) << ',' << truth.velocity_ned(1) << ',' << truth.velocity_ned(2) << ','
			       << truth.velocity_ned.norm() << ','
			       << truth.acceleration_ned(0) << ',' << truth.acceleration_ned(1) << ','
			       << truth.acceleration_ned(2) << ','
			       << gnss_position_ned(0) << ',' << gnss_position_ned(1) << ',' << gnss_position_ned(2) << ','
			       << gnss_velocity_ned(0) << ',' << gnss_velocity_ned(1) << ',' << gnss_velocity_ned(2) << ','
			       << state.gnss_kf_snapshot.position_ned(0) << ','
			       << state.gnss_kf_snapshot.position_ned(1) << ','
			       << state.gnss_kf_snapshot.position_ned(2) << ','
			       << state.gnss_kf_snapshot.velocity_ned(0) << ','
			       << state.gnss_kf_snapshot.velocity_ned(1) << ','
			       << state.gnss_kf_snapshot.velocity_ned(2) << ','
			       << state.imu_velocity_diagnostics.raw_gnss_delta_velocity(0) << ','
			       << state.imu_velocity_diagnostics.raw_gnss_delta_velocity(1) << ','
			       << state.imu_velocity_diagnostics.filtered_gnss_delta_velocity(0) << ','
			       << state.imu_velocity_diagnostics.filtered_gnss_delta_velocity(1) << ','
			       << state.imu_velocity_diagnostics.imu_delta_velocity(0) << ','
			       << state.imu_velocity_diagnostics.imu_delta_velocity(1) << ','
			       << state.imu_velocity_diagnostics.residual(0) << ','
			       << state.imu_velocity_diagnostics.residual(1) << ','
			       << state.imu_velocity_diagnostics.residual_variance(0) << ','
			       << state.imu_velocity_diagnostics.residual_variance(1) << ','
			       << state.imu_velocity_diagnostics.normalized_error << ','
			       << state.gnss_kf_acceleration_noise_density_squared << ','
			       << state.imu_velocity_suspicion << ','
			       << state.gnss_velocity_consistency_suspicion << ','
			       << state.position_suspicion << ','
			       << static_cast<int>(state.state) << '\n';
		}
	}

	float sampleGaussian(const float standard_deviation)
	{
		std::normal_distribution<float> distribution{0.f, standard_deviation};
		return distribution(_random_generator);
	}

	void makeGnssMeasurement(const TruthSample &truth, const uint64_t time_us,
				 matrix::Vector3f &position_ned, matrix::Vector3f &velocity_ned)
	{
		position_ned = truth.position_ned;
		velocity_ned = truth.velocity_ned;

		const float time_s = static_cast<float>(time_us - kStartTimeUs) * 1e-6f;
		constexpr float horizontal_frequencies_rad_s[]{0.2f, 0.4f, 0.63f};
		constexpr float vertical_frequencies_rad_s[]{0.1f, 0.2f, 0.32f};
		constexpr float component_count = 3.f;
		const float horizontal_amplitude = kGnssHorizontalPositionStdDevM * sqrtf(2.f / component_count);
		const float vertical_amplitude = kGnssVerticalPositionStdDevM * sqrtf(2.f / component_count);

		for (size_t axis = 0; axis < 3; ++axis) {
			const float *frequencies = axis < 2 ? horizontal_frequencies_rad_s : vertical_frequencies_rad_s;
			const float amplitude = axis < 2 ? horizontal_amplitude : vertical_amplitude;

			for (size_t component = 0; component < 3; ++component) {
				const float phase = 0.37f + static_cast<float>(axis) * 1.1f
						    + static_cast<float>(component) * 1.7f;
				const float angle = frequencies[component] * time_s + phase;
				position_ned(axis) += amplitude * sinf(angle);
				velocity_ned(axis) += amplitude * frequencies[component] * cosf(angle);
			}
		}
	}

	void runTrajectory(const Trajectory trajectory, const uint64_t duration_us)
	{
		const uint64_t monitoring_start_us = math::min(kWarmupDurationUs, duration_us / 2);
		uint64_t next_gnss_time_us = kStartTimeUs;
		uint64_t next_aux_position_time_us = kStartTimeUs;

		for (uint64_t time_us = kStartTimeUs; time_us <= kStartTimeUs + duration_us; time_us += kImuPeriodUs) {
			const float time_s = static_cast<float>(time_us - kStartTimeUs) * 1e-6f;
			const TruthSample truth = trajectory(time_s);

			const float imu_dt = static_cast<float>(kImuPeriodUs) * 1e-6f;
			const float imu_white_noise_delta_velocity_std_dev = kImuAccelerationNoiseStdDevMS2 * imu_dt;
			matrix::Vector3f imu_delta_velocity_ned = truth.acceleration_ned * imu_dt;

			for (size_t axis = 0; axis < 3; ++axis) {
				imu_delta_velocity_ned(axis) += sampleGaussian(imu_white_noise_delta_velocity_std_dev);
			}

			_analyzer.pushImu({
				.time_us = time_us,
				.delta_velocity_ned = imu_delta_velocity_ned,
				.delta_velocity_white_noise_variance_ned = {
					math::sq(kImuAccelerationNoiseStdDevMS2 * imu_dt),
					math::sq(kImuAccelerationNoiseStdDevMS2 * imu_dt),
					math::sq(kImuAccelerationNoiseStdDevMS2 * imu_dt)
				},
				.acceleration_bias_variance_ned = {
					math::sq(kImuAccelerationBiasStdDevMS2),
					math::sq(kImuAccelerationBiasStdDevMS2),
					math::sq(kImuAccelerationBiasStdDevMS2)
				},
				.dt = imu_dt,
			});

			if (time_us < next_gnss_time_us) {
				continue;
			}

			matrix::Vector3f gnss_position_ned;
			matrix::Vector3f gnss_velocity_ned;
			makeGnssMeasurement(truth, time_us, gnss_position_ned, gnss_velocity_ned);

			_analyzer.pushGnss({
				.time_us = time_us,
				.pos_ned = gnss_position_ned,
				.vel_ned = gnss_velocity_ned,
				.pos_var = {
					kGnssHorizontalPositionStdDevM * kGnssHorizontalPositionStdDevM,
					kGnssHorizontalPositionStdDevM * kGnssHorizontalPositionStdDevM,
					kGnssVerticalPositionStdDevM * kGnssVerticalPositionStdDevM
				},
				.vel_var = {
					kGnssVelocityStdDevMS * kGnssVelocityStdDevMS,
					kGnssVelocityStdDevMS * kGnssVelocityStdDevMS,
					kGnssVelocityStdDevMS * kGnssVelocityStdDevMS
				},
			});
			next_gnss_time_us += kGnssPeriodUs;

			// Aux timestamps need a newer GNSS endpoint so that the analyzer can interpolate it.
			while (next_aux_position_time_us <= time_us) {
				const float aux_time_s = static_cast<float>(next_aux_position_time_us - kStartTimeUs) * 1e-6f;
				const TruthSample aux_truth = trajectory(aux_time_s);

				matrix::Vector2f aux_position_ne{aux_truth.position_ned(0), aux_truth.position_ned(1)};
				aux_position_ne(0) += sampleGaussian(kAuxHorizontalPositionStdDevM);
				aux_position_ne(1) += sampleGaussian(kAuxHorizontalPositionStdDevM);

				_analyzer.pushAuxPosition({
					.time_us = next_aux_position_time_us,
					.position_ne = aux_position_ne,
					.position_variance_ne = {
						kAuxHorizontalPositionStdDevM * kAuxHorizontalPositionStdDevM,
						kAuxHorizontalPositionStdDevM * kAuxHorizontalPositionStdDevM
					},
				});
				next_aux_position_time_us += kAuxPositionPeriodUs;
			}

			const auto state = _analyzer.extendedState();

			maybeTraceIteration(time_s,
				truth,
				gnss_position_ned,
				gnss_velocity_ned,
				state);

			if (time_us - kStartTimeUs >= monitoring_start_us) {
				_max_imu_velocity_suspicion = math::max(_max_imu_velocity_suspicion,
									       static_cast<float>(state.imu_velocity_suspicion));
			}
		}
	}

	GnssAnalyzer _analyzer;
	std::mt19937 _random_generator;
	std::ofstream _trace;
	float _max_imu_velocity_suspicion{0.f};
};

TruthSample figureEightTrajectory(const float time_s, const float period_s)
{
	constexpr float north_amplitude_m = 30.f;
	constexpr float east_amplitude_m = 15.f;
	const float angular_rate_rad_s = 2.f * M_PI_F / period_s;

	const float phase = angular_rate_rad_s * time_s;
	const float twice_phase = 2.f * phase;

	TruthSample sample{};
	sample.position_ned = {
		north_amplitude_m * sinf(phase),
		east_amplitude_m * sinf(twice_phase),
		0.f
	};
	sample.velocity_ned = {
		north_amplitude_m * angular_rate_rad_s * cosf(phase),
		2.f * east_amplitude_m * angular_rate_rad_s * cosf(twice_phase),
		0.f
	};
	sample.acceleration_ned = {
		-north_amplitude_m * angular_rate_rad_s * angular_rate_rad_s * sinf(phase),
		-4.f * east_amplitude_m * angular_rate_rad_s * angular_rate_rad_s * sinf(twice_phase),
		0.f
	};

	return sample;
}

TruthSample nominalFigureEightTrajectory(const float time_s)
{
	return figureEightTrajectory(time_s, kNominalFigureEightPeriodS);
}

TruthSample aggressiveFigureEightTrajectory(const float time_s)
{
	return figureEightTrajectory(time_s, kAggressiveFigureEightPeriodS);
}

TruthSample extremeFigureEightTrajectory(const float time_s)
{
	return figureEightTrajectory(time_s, kExtremeFigureEightPeriodS);
}

TEST_F(GnssAnalyzerTest, NominalFigureEightRemainsTrusted)
{
	runTrajectory(nominalFigureEightTrajectory,
		      static_cast<uint64_t>(kFigureEightCycleCount * kNominalFigureEightPeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, AggressiveFigureEightRemainsTrusted)
{
	runTrajectory(aggressiveFigureEightTrajectory,
		      static_cast<uint64_t>(kFigureEightCycleCount * kAggressiveFigureEightPeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, ExtremeFigureEightRemainsTrusted)
{
	runTrajectory(extremeFigureEightTrajectory,
		      static_cast<uint64_t>(kFigureEightCycleCount * kExtremeFigureEightPeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

} // namespace
