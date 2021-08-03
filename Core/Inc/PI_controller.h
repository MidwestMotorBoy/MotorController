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
    int32_t ki,kp;//stored at 1000x for precision
    int32_t error_intergration;//stored at 1000x for precision
	int32_t min_intergrator,max_intergrator;
	int32_t min_output,max_output;
	uint32_t slew;
public:
	PI_controller();
	PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator);
	PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator,
		int32_t _min_output,int32_t _max_output);
	PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator,
			int32_t _min_output,int32_t _max_output,int32_t _slew);
	~PI_controller();
	int32_t update(int32_t input,int32_t feedback,int32_t Ts);
};

#endif



