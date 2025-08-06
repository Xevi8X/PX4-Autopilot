#include <gtest/gtest.h>
#include "ActuatorEffectivenessDevel.hpp"

using namespace matrix;

TEST(ActuatorEffectivenessDevel, EffectivenessMatrix)
{
	ActuatorEffectivenessDevel::Geometry geometry = {};

	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	ActuatorEffectivenessDevel::computeEffectivenessMatrix(geometry, effectiveness);

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{-1.0f,   1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 1.0f,  -1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.05f,  0.05f, -0.05f, -0.05f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-1.0f,  -1.0f,  -1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	ActuatorEffectiveness::EffectivenessMatrix effectiveness_expected(expected);

	EXPECT_EQ(effectiveness, effectiveness_expected);
}
