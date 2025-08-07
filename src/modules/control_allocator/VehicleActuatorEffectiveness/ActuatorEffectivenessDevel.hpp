/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/tilt_forward.h>

class ActuatorEffectivenessDevel : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessDevel(ModuleParams *parent);
	virtual ~ActuatorEffectivenessDevel() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	const char *name() const override { return "Devel"; }

private:
	float _tilt_base{math::radians(0.0f)}; ///< base position of the front rotors, 0 - directly up, pi/2 - directly forward
	matrix::Vector3f _tilt_axis{0.0f, -1.0f, 0.0f}; ///< axis of front rotors tilt

	uORB::SubscriptionData<tilt_forward_s> _tilt_forward_sub{ORB_ID(tilt_forward)};

	static constexpr float _min_tilt_angle{math::radians(-10.0f)}; ///< minimum tilt angle for the front rotors, in radians
	static constexpr float _max_tilt_angle{math::radians(90.0f)}; ///< maximum tilt angle for the front rotors, in radians
	static constexpr float _tilt_base_deadzone{math::radians(3.0f)}; ///< deadzone for tilt base updates, in radians
};
