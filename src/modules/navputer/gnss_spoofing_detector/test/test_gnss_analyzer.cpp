#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>

#include <mathlib/mathlib.h>

#include "gnss_analyzer.hpp"
#include "spoofing_profiles.hpp"

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

constexpr float kNominalLoopedTrianglePeriodS = 64.f;
constexpr float kAggressiveLoopedTrianglePeriodS = 24.f;
constexpr float kExtremeLoopedTrianglePeriodS = 16.f;
constexpr float kLoopedTriangleCycleCount = 5.f;

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

struct TruthSample
{
	matrix::Vector3f position_ned{};
	matrix::Vector3f velocity_ned{};
	matrix::Vector3f acceleration_ned{};
};

enum class SpoofProfile
{
	GradualOffset,
	GradualCarryOff
};

struct SpoofConfig
{
	SpoofProfile profile{SpoofProfile::GradualOffset};
	float start_time_s{0.f};
	matrix::Vector3f target_offset_ned{};
	float maximum_speed_m_s{0.f};
};

struct SpoofRuntime
{
	uint64_t last_gnss_time_us{0};
	float progress{0.f};
	matrix::Vector3f carry_off_start_position_ned{};
	matrix::Vector3f carry_off_start_velocity_ned{};
	matrix::Vector3f carry_off_target_position_ned{};
	bool carry_off_initialized{false};
};

using Trajectory = TruthSample (*)(float time_s);

TruthSample nominalFigureEightTrajectory(float time_s);

class GnssAnalyzerTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_analyzer.reset(true);
		_random_generator.seed(0x4E415650);

		// Additional traces for the visualization. If user launches the test this way:
		// "GNSS_ANALYZER_TRACE=/tmp/test_route.csv ./unit-test_gnss_analyzer"
		// It will create a bunch of test_route_<TESTNAME>.csv files. They can be visualized later.
		if (const char *trace_path = getenv("GNSS_ANALYZER_TRACE"))
		{
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
		       << "spoof_active,spoof_progress,spoof_position_n_m,spoof_position_e_m,spoof_position_d_m,"
		       << "spoof_velocity_n_m_s,spoof_velocity_e_m_s,spoof_velocity_d_m_s,"
		       << "imu_velocity_suspicion,gnss_velocity_consistency_suspicion,position_suspicion,state\n";
		_trace << std::fixed << std::setprecision(6);
	}

	void maybeTraceIteration(const float time_s,
		const TruthSample& truth,
		const matrix::Vector3f& gnss_position_ned,
		const matrix::Vector3f& gnss_velocity_ned,
		const bool spoof_active,
		const GnssSpoofing::GradualOffsetSample &spoof,
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
			       << spoof_active << ',' << spoof.progress << ','
			       << spoof.position_offset_ned(0) << ',' << spoof.position_offset_ned(1) << ','
			       << spoof.position_offset_ned(2) << ','
			       << spoof.velocity_offset_ned(0) << ',' << spoof.velocity_offset_ned(1) << ','
			       << spoof.velocity_offset_ned(2) << ','
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

	void makeGnssMeasurement(const TruthSample& truth,
			const uint64_t time_us,
			matrix::Vector3f& position_ned,
			matrix::Vector3f& velocity_ned)
	{
		position_ned = truth.position_ned;
		velocity_ned = truth.velocity_ned;

		const float time_s = static_cast<float>(time_us - kStartTimeUs) * 1e-6f;
		constexpr float horizontal_frequencies_rad_s[]{0.2f, 0.4f, 0.63f};
		constexpr float vertical_frequencies_rad_s[]{0.1f, 0.2f, 0.32f};
		constexpr float component_count = 3.f;
		const float horizontal_amplitude = kGnssHorizontalPositionStdDevM * sqrtf(2.f / component_count);
		const float vertical_amplitude = kGnssVerticalPositionStdDevM * sqrtf(2.f / component_count);

		for (size_t axis = 0; axis < 3; ++axis)
		{
			const float *frequencies = axis < 2 ? horizontal_frequencies_rad_s : vertical_frequencies_rad_s;
			const float amplitude = axis < 2 ? horizontal_amplitude : vertical_amplitude;

			for (size_t component = 0; component < 3; ++component)
			{
				const float phase = 0.37f + static_cast<float>(axis) * 1.1f
						    + static_cast<float>(component) * 1.7f;
				const float angle = frequencies[component] * time_s + phase;
				position_ned(axis) += amplitude * sinf(angle);
				velocity_ned(axis) += amplitude * frequencies[component] * cosf(angle);
			}
		}
	}

	void generateImuSample(const uint64_t time_us, const TruthSample& truth)
	{
		const float imu_dt = static_cast<float>(kImuPeriodUs) * 1e-6f;
		const float imu_white_noise_delta_velocity_std_dev = kImuAccelerationNoiseStdDevMS2 * imu_dt;
		matrix::Vector3f imu_delta_velocity_ned = truth.acceleration_ned * imu_dt;

		for (size_t axis = 0; axis < 3; ++axis)
		{
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

	}

	void generateGnssSample(const uint64_t time_us,
			const TruthSample& truth,
			SpoofRuntime &spoof_runtime,
			const SpoofConfig *spoof_config = nullptr)
	{
		const float time_s = static_cast<float>(time_us - kStartTimeUs) * 1e-6f;
		matrix::Vector3f gnss_position_ned;
		matrix::Vector3f gnss_velocity_ned;
		makeGnssMeasurement(truth, time_us, gnss_position_ned, gnss_velocity_ned);

		GnssSpoofing::GradualOffsetSample spoof{};
		const bool spoof_active = spoof_config != nullptr && time_s >= spoof_config->start_time_s;

		if (spoof_active)
		{
			const float spoof_dt = spoof_runtime.last_gnss_time_us == 0
				? time_s - spoof_config->start_time_s
				: static_cast<float>(time_us - spoof_runtime.last_gnss_time_us) * 1e-6f;

			if (spoof_config->profile == SpoofProfile::GradualOffset)
			{
				spoof = GnssSpoofing::gradualOffset(
						spoof_config->target_offset_ned,
						spoof_config->maximum_speed_m_s,
						spoof_runtime.progress,
						spoof_dt);
				gnss_position_ned += spoof.position_offset_ned;
				gnss_velocity_ned += spoof.velocity_offset_ned;
			}
			else
			{
				if (!spoof_runtime.carry_off_initialized)
				{
					spoof_runtime.carry_off_start_position_ned = {gnss_position_ned(0), gnss_position_ned(1), 0.f};
					spoof_runtime.carry_off_start_velocity_ned = {gnss_velocity_ned(0), gnss_velocity_ned(1), 0.f};
					spoof_runtime.carry_off_target_position_ned =
						spoof_runtime.carry_off_start_position_ned + spoof_config->target_offset_ned;
					spoof_runtime.carry_off_initialized = true;
				}

				const matrix::Vector3f original_position_ned = gnss_position_ned;
				const matrix::Vector3f original_velocity_ned = gnss_velocity_ned;
				const auto carry_off = GnssSpoofing::gradualCarryOff(
						spoof_runtime.carry_off_start_position_ned,
						spoof_runtime.carry_off_start_velocity_ned,
						spoof_runtime.carry_off_target_position_ned,
						spoof_config->maximum_speed_m_s,
						spoof_runtime.progress,
						spoof_dt);

				gnss_position_ned(0) = carry_off.position_ned(0);
				gnss_position_ned(1) = carry_off.position_ned(1);
				gnss_velocity_ned(0) = carry_off.velocity_ned(0);
				gnss_velocity_ned(1) = carry_off.velocity_ned(1);
				spoof.position_offset_ned = gnss_position_ned - original_position_ned;
				spoof.velocity_offset_ned = gnss_velocity_ned - original_velocity_ned;
				spoof.progress = carry_off.progress;
				spoof.duration_s = carry_off.duration_s;
			}

			spoof_runtime.progress = spoof.progress;
			spoof_runtime.last_gnss_time_us = time_us;
		}

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

		const auto state = _analyzer.extendedState();

		maybeTraceIteration(time_s,
			truth,
			gnss_position_ned,
			gnss_velocity_ned,
			spoof_active,
			spoof,
			state);
	}

	void generateAuxSample(const uint64_t time_us, uint64_t& next_aux_position_time_us, const Trajectory& trajectory)
	{
		while (next_aux_position_time_us <= time_us)
		{
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
	}

	void runTrajectory(const Trajectory trajectory,
			const uint64_t duration_us,
			const SpoofConfig *spoof_config = nullptr)
	{
		const uint64_t monitoring_start_us = math::min(kWarmupDurationUs, duration_us / 2);
		uint64_t next_gnss_time_us = kStartTimeUs;
		uint64_t next_aux_position_time_us = kStartTimeUs;
		SpoofRuntime spoof_runtime{};

		for (uint64_t time_us = kStartTimeUs; time_us <= kStartTimeUs + duration_us; time_us += kImuPeriodUs)
		{
			const float time_s = static_cast<float>(time_us - kStartTimeUs) * 1e-6f;
			const TruthSample truth = trajectory(time_s);

			generateImuSample(time_us, truth);

			if (time_us < next_gnss_time_us)
			{
				continue;
			}

			const bool spoof_active = spoof_config != nullptr && time_s >= spoof_config->start_time_s;

			if (spoof_active && !_spoof_started)
			{
				_spoof_started = true;
				_trusted_before_spoof = _analyzer.state() == GnssSpoofingState::Trusted;
			}

			generateGnssSample(time_us, truth, spoof_runtime, spoof_config);

			if (spoof_active)
			{
				_final_spoof_progress = spoof_runtime.progress;
			}

			next_gnss_time_us += kGnssPeriodUs;

			generateAuxSample(time_us, next_aux_position_time_us, trajectory);

			const auto state = _analyzer.extendedState();

			if (spoof_active && state.state == GnssSpoofingState::Untrusted)
			{
				_spoof_detected = true;
			}

			if (time_us - kStartTimeUs >= monitoring_start_us)
			{
				_max_imu_velocity_suspicion = math::max(_max_imu_velocity_suspicion,
								       static_cast<float>(state.imu_velocity_suspicion));
				_max_gnss_velocity_consistency_suspicion = math::max(_max_gnss_velocity_consistency_suspicion,
								       static_cast<float>(state.gnss_velocity_consistency_suspicion));
				_max_aux_position_suspicion = math::max(_max_aux_position_suspicion,
								       static_cast<float>(state.position_suspicion));
			}
		}
	}

	void expectGradualCarryOffDetected(const float maximum_speed_m_s, const float observation_duration_s)
	{
		constexpr float attack_start_time_s = 30.f;
		const SpoofConfig spoof_config
		{
			.profile = SpoofProfile::GradualCarryOff,
			.start_time_s = attack_start_time_s,
			.target_offset_ned = {200.f, 0.f, 0.f},
			.maximum_speed_m_s = maximum_speed_m_s,
		};

		runTrajectory(nominalFigureEightTrajectory,
			      static_cast<uint64_t>((attack_start_time_s + observation_duration_s) * 1e6f),
			      &spoof_config);

		EXPECT_TRUE(_spoof_started);
		EXPECT_TRUE(_trusted_before_spoof);
		EXPECT_TRUE(_spoof_detected);
		EXPECT_FLOAT_EQ(_final_spoof_progress, 1.f);
		EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Untrusted);
	}

	void expectGradualOffsetDetected(const float maximum_speed_m_s)
	{
		constexpr float attack_start_time_s = 30.f;
		constexpr float target_offset_m = 200.f;
		constexpr float post_attack_observation_duration_s = 20.f;

		const SpoofConfig spoof_config
		{
			.profile = SpoofProfile::GradualOffset,
			.start_time_s = attack_start_time_s,
			.target_offset_ned = {target_offset_m, 0.f, 0.f},
			.maximum_speed_m_s = maximum_speed_m_s,
		};
		const float attack_duration_s = GnssSpoofing::kMaximumRampSlope * target_offset_m / maximum_speed_m_s;

		runTrajectory(nominalFigureEightTrajectory,
			      static_cast<uint64_t>((attack_start_time_s + attack_duration_s
					     + post_attack_observation_duration_s) * 1e6f),
			      &spoof_config);

		EXPECT_TRUE(_spoof_started);
		EXPECT_TRUE(_trusted_before_spoof);
		EXPECT_TRUE(_spoof_detected);
		EXPECT_FLOAT_EQ(_final_spoof_progress, 1.f);
		EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Untrusted);
	}

	GnssAnalyzer _analyzer;
	std::mt19937 _random_generator;
	std::ofstream _trace;
	float _max_imu_velocity_suspicion{0.f};
	float _max_gnss_velocity_consistency_suspicion{0.f};
	float _max_aux_position_suspicion{0.f};
	bool _spoof_started{false};
	bool _trusted_before_spoof{false};
	bool _spoof_detected{false};
	float _final_spoof_progress{0.f};
};

// Figure "Eight" trajectory:
//             ╭───────╮
//           ╭─╯       ╰─╮
//          ╱             ╲
//         ╱               ╲
//         ╲               ╱
//          ╲             ╱
//           ╲           ╱
//            ╲         ╱
//             ╲       ╱
//              ╲     ╱
//               ╲   ╱
//                ╲ ╱
//                 ╳
//                ╱ ╲
//               ╱   ╲
//              ╱     ╲
//             ╱       ╲
//            ╱         ╲
//           ╱           ╲
//          ╱             ╲
//         ╱               ╲
//         ╲               ╱
//          ╲             ╱
//           ╰─╮       ╭─╯
//             ╰───────╯
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

// Triangle with small loops trajectory:
//                        ╭───╮
//                       ╱     ╲
//                       ╲     ╱
//                        ╲   ╱
//                         ╲ ╱
//                          ╳
//                         ╱ ╲
//                        ╱   ╲
//                       ╱     ╲
//                      ╱       ╲
//                     ╱         ╲
//                    ╱           ╲
//           ╭───╮   ╱             ╲   ╭───╮
//          ╱     ╲ ╱               ╲ ╱     ╲
//          ╲     ╱───────────────────╲     ╱       
//           ╰───╯                     ╰───╯
TruthSample loopedTriangleTrajectory(const float time_s, const float period_s)
{
	constexpr float radius_m = 30.f;
	constexpr float loop_ratio = 1.15f;
	const float angular_rate_rad_s = 2.f * M_PI_F / period_s;

	const float phase = angular_rate_rad_s * time_s;
	const float twice_phase = 2.f * phase;

	TruthSample sample{};
	sample.position_ned = {
		radius_m * (2.f * cosf(phase) + loop_ratio * cosf(twice_phase)),
		radius_m * (2.f * sinf(phase) - loop_ratio * sinf(twice_phase)),
		0.f
	};
	sample.velocity_ned = {
		radius_m * angular_rate_rad_s * (-2.f * sinf(phase) - 2.f * loop_ratio * sinf(twice_phase)),
		radius_m * angular_rate_rad_s * (2.f * cosf(phase) - 2.f * loop_ratio * cosf(twice_phase)),
		0.f
	};
	sample.acceleration_ned = {
		radius_m * angular_rate_rad_s * angular_rate_rad_s
		* (-2.f * cosf(phase) - 4.f * loop_ratio * cosf(twice_phase)),
		radius_m * angular_rate_rad_s * angular_rate_rad_s
		* (-2.f * sinf(phase) + 4.f * loop_ratio * sinf(twice_phase)),
		0.f
	};

	return sample;
}

TruthSample nominalLoopedTriangleTrajectory(const float time_s)
{
	return loopedTriangleTrajectory(time_s, kNominalLoopedTrianglePeriodS);
}

TruthSample aggressiveLoopedTriangleTrajectory(const float time_s)
{
	return loopedTriangleTrajectory(time_s, kAggressiveLoopedTrianglePeriodS);
}

TruthSample extremeLoopedTriangleTrajectory(const float time_s)
{
	return loopedTriangleTrajectory(time_s, kExtremeLoopedTrianglePeriodS);
}

TEST_F(GnssAnalyzerTest, NominalFigureEightBecomesTrusted)
{
	runTrajectory(nominalFigureEightTrajectory,
		      static_cast<uint64_t>(kFigureEightCycleCount * kNominalFigureEightPeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_LT(_max_gnss_velocity_consistency_suspicion, 0.8f);
	EXPECT_LT(_max_aux_position_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, AggressiveFigureEightBecomesTrusted)
{
	runTrajectory(aggressiveFigureEightTrajectory,
		      static_cast<uint64_t>(kFigureEightCycleCount * kAggressiveFigureEightPeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_LT(_max_gnss_velocity_consistency_suspicion, 0.8f);
	EXPECT_LT(_max_aux_position_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, ExtremeFigureEightBecomesTrusted)
{
	runTrajectory(extremeFigureEightTrajectory,
		      static_cast<uint64_t>(kFigureEightCycleCount * kExtremeFigureEightPeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_LT(_max_gnss_velocity_consistency_suspicion, 0.8f);
	EXPECT_LT(_max_aux_position_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, NominalLoopedTriangleBecomesTrusted)
{
	runTrajectory(nominalLoopedTriangleTrajectory,
		      static_cast<uint64_t>(kLoopedTriangleCycleCount * kNominalLoopedTrianglePeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_LT(_max_gnss_velocity_consistency_suspicion, 0.8f);
	EXPECT_LT(_max_aux_position_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, AggressiveLoopedTriangleBecomesTrusted)
{
	runTrajectory(aggressiveLoopedTriangleTrajectory,
		      static_cast<uint64_t>(kLoopedTriangleCycleCount * kAggressiveLoopedTrianglePeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_LT(_max_gnss_velocity_consistency_suspicion, 0.8f);
	EXPECT_LT(_max_aux_position_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, ExtremeLoopedTriangleBecomesTrusted)
{
	runTrajectory(extremeLoopedTriangleTrajectory,
		      static_cast<uint64_t>(kLoopedTriangleCycleCount * kExtremeLoopedTrianglePeriodS * 1e6f));

	EXPECT_LT(_max_imu_velocity_suspicion, 0.8f);
	EXPECT_LT(_max_gnss_velocity_consistency_suspicion, 0.8f);
	EXPECT_LT(_max_aux_position_suspicion, 0.8f);
	EXPECT_EQ(_analyzer.state(), GnssSpoofingState::Trusted);
}

TEST_F(GnssAnalyzerTest, SlowGradualCarryOffIsDetected)
{
	expectGradualCarryOffDetected(10.f, 150.f);
}

TEST_F(GnssAnalyzerTest, AverageGradualCarryOffIsDetected)
{
	expectGradualCarryOffDetected(25.f, 50.f);
}

TEST_F(GnssAnalyzerTest, FastGradualCarryOffIsDetected)
{
	expectGradualCarryOffDetected(50.f, 30.f);
}

TEST_F(GnssAnalyzerTest, SlowGradualOffsetIsDetected)
{
	expectGradualOffsetDetected(2.f);
}

TEST_F(GnssAnalyzerTest, AverageGradualOffsetIsDetected)
{
	expectGradualOffsetDetected(10.f);
}

TEST_F(GnssAnalyzerTest, FastGradualOffsetIsDetected)
{
	expectGradualOffsetDetected(50.f);
}

} // namespace
