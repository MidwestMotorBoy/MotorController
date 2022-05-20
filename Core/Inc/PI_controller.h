/*
 * controllers.cpp
 *
 *  Created on: Jun 26, 2021
 *      Author: LoganRosenmayer
 */
#include <stm32h7xx_hal.h>
#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H
class PI_controller
{
private:
	float ki,kp;
    float error_intergration;
    float min_intergrator,max_intergrator;
	float min_output,max_output;
	float slew;
	float old_ref;
public:
	PI_controller();
	PI_controller(float _ki, float _kp,float _min_intergrator,float _max_intergrator);
	PI_controller(float _ki, float _kp,float _min_output,float _max_output,float _slew);
	PI_controller(float _ki, float _kp,float _min_intergrator,float _max_intergrator,
			float _min_output,float _max_output);
	PI_controller(float _ki, float _kp,float _min_intergrator,float _max_intergrator,
			float _min_output,float _max_output,float _slew);
	~PI_controller();
	float update(float input,float feedback,float Ts);
};

#endif



