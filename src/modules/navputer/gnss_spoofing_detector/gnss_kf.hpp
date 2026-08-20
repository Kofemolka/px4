/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gnss_kf.hpp
 * Implementation of the GnssKalmanFilter.
 *
 * @author
 */

#ifndef GNSS_KF_HPP
#define GNSS_KF_HPP

#include <cstdint>

#include <matrix/SquareMatrix.hpp>
#include <matrix/Vector.hpp>
#include <matrix/Vector3.hpp>

class GnssKalmanFilter
{
public:
	struct Measurement
	{
		uint64_t time_us;
		matrix::Vector3f pos_ned;
		matrix::Vector3f vel_ned;
		matrix::Vector3f pos_var;
		matrix::Vector3f vel_var;
	};
public:
	using Matrix6f = matrix::SquareMatrix<float, 6>;
	using Vector6f = matrix::Vector<float, 6>;

	void reset();
	bool process(const Measurement &sample);

	const Vector6f& state() const;
	const Matrix6f& covariance() const;
private:
	bool isMeasurementValid(const Measurement& sample);
	void initialize(const Measurement& sample);
	bool calcTimeDelta(uint64_t sample_ts_us, float& dt);
	void predict(const float dt);
	bool update(const Measurement& sample);

	Matrix6f buildMotionMatrixF(const float dt);
	Matrix6f buildProcessNoiseMatrixQ(const float dt);
	Matrix6f buildMeasurementNoiseMatrixR(const Measurement& sample);
private:
	Vector6f _state{};
	Matrix6f _P{};
	uint64_t _time_us{0};
	bool _initialized{false};
};

#endif
