/*
 * current_controllers.cpp
 *
 *  Created on: Aug 1, 2021
 *      Author: LoganRosenmayer
 */
#include "Current_Controller.h"

int32_t Current_Controller::update(int32_t iq_target, int32_t iq_act, int32_t id_target,int32_t id_act,int32_t speed_mech,int32_t Ts, int32_t *vq, int32_t *vd){

}
/*
 * Class:	Current_Controller::update
 * ----------------------
 * computers the power invariant DQZ transform using a LUT for sin and cos and fixed point numbers
 * input: Target value for system
 * feedback: Measurement of current system state
 * ts: sample time (microseconds)
 *
 * returns:nothing
 */
int32_t update(int32_t iq_target, int32_t iq_act, int32_t id_target,int32_t id_act,int32_t speed_mech,int32_t Ts, int32_t *vq, int32_t *vd){
	int32_t speed_mili_erads = speed_mech*motor.polepairs*105;
	int32_t vd_forward = motor.rs*id_target-*motor*iq_target
}
Current_Controller::Current_Controller(pi_settings iq_settigns, pi_settings id_settigns,motor_parameters _motor_parameters){
	PI_controller iq_PI(iq_settigns.ki, iq_settigns.kp,iq_settigns.min_intergrator,iq_settigns.max_intergrator,iq_settigns.min_output,iq_settigns.max_output,iq_settigns.slew);
	PI_controller id_PI(id_settigns.ki, id_settigns.kp,id_settigns.min_intergrator,id_settigns.max_intergrator,id_settigns.min_output,id_settigns.max_output,id_settigns.slew);
	motor = _motor_parameters;
}
