#include "ActuatorEffectivenessDevel.hpp"

ActuatorEffectivenessDevel::ActuatorEffectivenessDevel(ModuleParams * parent)
	: ModuleParams(parent)
{
}

bool ActuatorEffectivenessDevel::getEffectivenessMatrix(Configuration & configuration, EffectivenessUpdateReason external_update)
{
	// TODO: load parameters from parameters

	float ct = 0.1f; // thrust coefficient, T = ct * omega^2
	float cm = 0.1f; // moment coefficient, M = cm * omega^2
	auto up_axis = matrix::Vector3f(0.0f, 0.0f, -1.0f); // up axis of the vehicle
	matrix::Vector3f tilted_forward_axis = matrix::Dcmf(matrix::AxisAnglef(_tilt_axis, _tilt_base)) * up_axis; // forward axis of the front rotors, tilted by the tilt angle

	// Motors:
	// 0 - Front left, tilted forward
	// 1 - Front right, tilted forward
	// 2 - Tail rotor, tilted laterally
	int motor_count = 3;

	matrix::Vector3f motor_positions[3] = {
		matrix::Vector3f( 0.5f, -0.5f, 0.0f), // Front left
		matrix::Vector3f( 0.5f,  0.5f, 0.0f), // Front right
		matrix::Vector3f(-0.5f,  0.0f, 0.0f)  // Tail rotor
	};

	// Motors:
	auto &effectiveness_matrix = configuration.effectiveness_matrices[configuration.selected_matrix];

	// TODO: cm sign in moments
	effectiveness_matrix.slice<3, 1>(0, 0) = ct * motor_positions[0].cross(tilted_forward_axis) + cm * tilted_forward_axis; // motor 0 moment
	effectiveness_matrix.slice<3, 1>(3, 0) = ct * tilted_forward_axis; // motor 0 thrust

	effectiveness_matrix.slice<3, 1>(0, 1) = ct * motor_positions[1].cross(tilted_forward_axis) + cm * tilted_forward_axis; // motor 1 moment
	effectiveness_matrix.slice<3, 1>(3, 1) = ct * tilted_forward_axis; // motor 1 thrust

	effectiveness_matrix.slice<3, 1>(0, 2) = ct * motor_positions[2].cross(up_axis) + cm * up_axis; // motor 2 moment
	effectiveness_matrix.slice<3, 1>(3, 2) = ct * up_axis; // motor 2 thrust

	configuration.actuatorsAdded(ActuatorType::MOTORS, motor_count);

	// Servos:
	// 0 - Front left, tilted forward
	// 1 - Front right, tilted forward
	// 2 - Tail rotor, tilted laterally
	int servo_count = 3;

	effectiveness_matrix.slice<3, 1>(0, 3) = motor_positions[0].cross(tilted_forward_axis.cross(tilted_forward_axis)); // servo 0 moment
	effectiveness_matrix.slice<3, 1>(3, 3) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // servo 0 thrust

	effectiveness_matrix.slice<3, 1>(0, 4) = motor_positions[1].cross(tilted_forward_axis.cross(tilted_forward_axis)); // servo 1 moment
	effectiveness_matrix.slice<3, 1>(3, 4) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // servo 1 thrust

	effectiveness_matrix.slice<3, 1>(0, 5) = matrix::Vector3f(0.0f, 0.0f, 1.0f); // servo 2 moment
	effectiveness_matrix.slice<3, 1>(3, 5) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // servo 2 thrust

	configuration.actuatorsAdded(ActuatorType::SERVOS, servo_count);
	return true;
}

void ActuatorEffectivenessDevel::updateSetpoint(const matrix::Vector<float,NUM_AXES>& control_sp, int matrix_index, ActuatorVector & actuator_sp, const ActuatorVector & actuator_min, const ActuatorVector & actuator_max)
{
	actuator_sp(3 + 0) += _tilt_base / _max_tilt_angle;
	actuator_sp(3 + 1) += _tilt_base / _max_tilt_angle;

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
}
