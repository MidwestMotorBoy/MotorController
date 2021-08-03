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
	int32_t ki;
	int32_t kp;
	int32_t min_intergrator;
	int32_t max_intergrator;
	int32_t min_output;
	int32_t max_output;
	int32_t slew;
};
struct motor_parameters{
	int32_t ld;//nH
	int32_t lq;//nH
	int32_t lambda_m;//mWb
	int32_t polepairs;
	int32_t rs;//µΩ
};
class Current_Controller
{
private:
	motor_parameters motor;// Used for feed forward
	PI_controller id_PI,iq_PI;//stored at 1000x for precision
public:
	Current_Controller(pi_settings iq_settigns, pi_settings id_settigns,motor_parameters _motor_parameters);
	int32_t update(int32_t iq_target, int32_t iq_act, int32_t id_target,int32_t id_act,int32_t speed_mech,int32_t Ts, int32_t *vq, int32_t *vd);
};


#endif /* INC_CURRENT_CONTROLLER_H_ */
