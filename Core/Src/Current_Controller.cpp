/*
 * current_controllers.cpp
 *
 *  Created on: Aug 1, 2021
 *      Author: LoganRosenmayer
 */
#include "Current_Controller.h"

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
void Current_Controller::update_vqvd(float iq_target, float iq_act, float id_target,float id_act,float speed_mech,float Ts, float &vq, float &vd){
	float speed_erads = speed_mech*motor.polepairs*0.1047;
	float vq_forward = motor.rs*iq_target-speed_erads*motor.ld*id_target+speed_erads*motor.lambda_m;
	float vd_forward = motor.rs*id_target-speed_erads*motor.lq*iq_target;
	float vq_adj = iq_PI.update(iq_target,iq_act,Ts);
	float vd_adj = id_PI.update(id_target,id_act,Ts);
	vq=vq_adj+vq_forward;
	vd=vd_adj+vd_forward;
}
Current_Controller::Current_Controller(pi_settings iq_settigns, pi_settings id_settigns,motor_parameters _motor_parameters){
	PI_controller _iq_PI(iq_settigns.ki, iq_settigns.kp,iq_settigns.min_intergrator,iq_settigns.max_intergrator,iq_settigns.min_output,iq_settigns.max_output,iq_settigns.slew);
	PI_controller _id_PI(id_settigns.ki, id_settigns.kp,id_settigns.min_intergrator,id_settigns.max_intergrator,id_settigns.min_output,id_settigns.max_output,id_settigns.slew);
	id_PI=_id_PI;
	iq_PI=_iq_PI;
	motor = _motor_parameters;
}
