/*
 * current_controllers.h
 *
 *  Created on: Aug 1, 2021
 *      Author: LoganRosenmayer
 */
#include <stm32h7xx_hal.h>
#include "PI_controller.h"
#ifndef INC_CURRENT_CONTROLLER_H_
#define INC_CURRENT_CONTROLLER_H_
//made just to centralize pi loop settings for readability
struct pi_settings{
	float ki;
	float kp;
	float min_intergrator;
	float max_intergrator;
	float min_output;
	float max_output;
	float slew;
};
struct motor_parameters{
	float ld;
	float lq;
	float lambda_m;
	float polepairs;
	float rs;
};
class Current_Controller
{
private:
	motor_parameters motor;// Used for feed forward
	PI_controller id_PI,iq_PI;//stored at 1000x for precision
public:
	Current_Controller(pi_settings iq_settigns, pi_settings id_settigns,motor_parameters _motor_parameters);
	~Current_Controller();
	void update_vqvd(float iq_target, float iq_act, float id_target,float id_act,float speed_mech,float Ts, float &vq, float &vd);
};


#endif /* INC_CURRENT_CONTROLLER_H_ */
