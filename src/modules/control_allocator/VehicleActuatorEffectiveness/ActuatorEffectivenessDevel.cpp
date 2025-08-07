#include "ActuatorEffectivenessDevel.hpp"

ActuatorEffectivenessDevel::ActuatorEffectivenessDevel(ModuleParams * parent)
	: ModuleParams(parent)
{
}

bool ActuatorEffectivenessDevel::getEffectivenessMatrix(Configuration & configuration, EffectivenessUpdateReason external_update)
{
	bool should_update = external_update != EffectivenessUpdateReason::NO_EXTERNAL_UPDATE;

	if (_tilt_forward_sub.update()) {
		const tilt_forward_s &tilt_forward = _tilt_forward_sub.get();
		float tilt_base_candidate = tilt_forward.tilt_base > 0.0f ? tilt_forward.tilt_base * _max_tilt_angle : tilt_forward.tilt_base * _min_tilt_angle;

		if (fabsf(tilt_base_candidate - _tilt_base) > _tilt_base_deadzone) {
			_tilt_base = tilt_base_candidate; // update the tilt base angle
			PX4_INFO("Tilt base updated to: %.2f rad", (double)_tilt_base);
			should_update = true;
		}
	}

	if (!should_update) {
		return false; // no update needed
	}

	// TODO: load parameters from parameters

	float ct = 5.0f; // thrust coefficient, T = ct * omega^2
	float cm = 0.0f; // moment coefficient, M = cm * omega^2
	auto up_axis = matrix::Vector3f(0.0f, 0.0f, -1.0f); // up axis of the vehicle
	matrix::Vector3f tilted_forward_axis = matrix::Dcmf(matrix::AxisAnglef(_tilt_axis, _tilt_base)) * up_axis; // forward axis of the front rotors, tilted by the tilt angle

	// Motors:
	// 0 - Front left, tilted forward
	// 1 - Front right, tilted forward
	// 2 - Tail rotor, tilted laterally
	int motor_count = 3;

	matrix::Vector3f motor_positions[3] = {
		matrix::Vector3f( 0.09f, -0.3f, 0.0f), // Front left
		matrix::Vector3f( 0.09f,  0.3f, 0.0f), // Front right
		matrix::Vector3f(-0.21f,  0.0f, 0.0f)  // Tail rotor
	};

	// Motors:
	auto &effectiveness_matrix = configuration.effectiveness_matrices[configuration.selected_matrix];

	// TODO: cm sign in moments
	effectiveness_matrix.slice<3, 1>(0, 0) = ct * motor_positions[0].cross(tilted_forward_axis) - cm * tilted_forward_axis; // motor 0 moment
	effectiveness_matrix.slice<3, 1>(3, 0) = ct * tilted_forward_axis; // motor 0 thrust

	effectiveness_matrix.slice<3, 1>(0, 1) = ct * motor_positions[1].cross(tilted_forward_axis) + cm * tilted_forward_axis; // motor 1 moment
	effectiveness_matrix.slice<3, 1>(3, 1) = ct * tilted_forward_axis; // motor 1 thrust

	effectiveness_matrix.slice<3, 1>(0, 2) = ct * motor_positions[2].cross(up_axis) - cm * up_axis; // motor 2 moment
	effectiveness_matrix.slice<3, 1>(3, 2) = ct * up_axis; // motor 2 thrust

	effectiveness_matrix(3, 0) = 0.0f; // Thrust x
	effectiveness_matrix(3, 1) = 0.0f; // Thrust x

	configuration.actuatorsAdded(ActuatorType::MOTORS, motor_count);

	// Servos:
	// 0 - Front left, tilted forward
	// 1 - Front right, tilted forward
	// 2 - Tail rotor, tilted laterally
	int servo_count = 2;

	effectiveness_matrix.slice<3, 1>(0, 3) = motor_positions[0].cross(_tilt_axis.cross(tilted_forward_axis)); // servo 0 moment
	effectiveness_matrix.slice<3, 1>(3, 3) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // servo 0 thrust

	effectiveness_matrix.slice<3, 1>(0, 4) = motor_positions[1].cross(_tilt_axis.cross(tilted_forward_axis)); // servo 1 moment
	effectiveness_matrix.slice<3, 1>(3, 4) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // servo 1 thrust

	// effectiveness_matrix.slice<3, 1>(0, 5) = matrix::Vector3f(0.0f, 0.0f, 1.0f); // servo 2 moment
	// effectiveness_matrix.slice<3, 1>(3, 5) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // servo 2 thrust

	configuration.actuatorsAdded(ActuatorType::SERVOS, servo_count);
	return true;
}

void ActuatorEffectivenessDevel::updateSetpoint(const matrix::Vector<float,NUM_AXES>& control_sp, int matrix_index, ActuatorVector & actuator_sp, const ActuatorVector & actuator_min, const ActuatorVector & actuator_max)
{
	auto tilt_base_sp = 2.0f * (_tilt_base - _min_tilt_angle) / (_max_tilt_angle - _min_tilt_angle) - 1.0f;

	actuator_sp(3 + 0) += tilt_base_sp;
	actuator_sp(3 + 1) += tilt_base_sp;

	if (actuator_sp(3 + 0) > actuator_max(3 + 0)) {
		actuator_sp(3 + 0) = actuator_max(3 + 0);
		// TODO: add desaturation logic
	}

	if (actuator_sp(3 + 1) > actuator_max(3 + 1)) {
		actuator_sp(3 + 1) = actuator_max(3 + 1);
		// TODO: add desaturation logic
	}

	if (actuator_sp(3 + 0) < actuator_min(3 + 0)) {
		actuator_sp(3 + 0) = actuator_min(3 + 0);
		// TODO: add desaturation logic
	}

	if (actuator_sp(3 + 1) < actuator_min(3 + 1)) {
		actuator_sp(3 + 1) = actuator_min(3 + 1);
		// TODO: add desaturation logic
	}

	if (actuator_sp(3 + 2) > actuator_max(3 + 2)) {
		actuator_sp(3 + 2) = actuator_max(3 + 2);
		// TODO: add desaturation logic
	}

	if (actuator_sp(3 + 2) < actuator_min(3 + 2)) {
		actuator_sp(3 + 2) = actuator_min(3 + 2);
		// TODO: add desaturation logic
	}
}
