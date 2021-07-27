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
    int32_t ki=0,kp=0;
    int32_t error_intergration=0;
	int32_t min_intergrator=0,max_intergrator=0;
	int32_t min_output=0,max_output=0;
public:
	PI_controller();
	PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator);
	PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator,
		int32_t _min_output,int32_t _max_output);
	~PI_controller();
};

#endif



