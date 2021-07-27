/*
 * controllers.cpp
 *
 *  Created on: Jun 26, 2021
 *      Author: LoganRosenmayer
 */
#include "PI_controller.h"
PI_controller::PI_controller(){

}
PI_controller::PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator){
	ki = _ki;
	kp = _kp;
	min_intergrator = _min_intergrator;
	max_intergrator = _max_intergrator;
	min_output = _min_intergrator;
	max_output = _max_intergrator;
}
PI_controller::PI_controller(int32_t _ki, int32_t _kp,int32_t _min_intergrator,int32_t _max_intergrator,
		int32_t _min_output,int32_t _max_output){
	ki = _ki;
	kp = _kp;
	min_intergrator = _min_intergrator;
	max_intergrator = _max_intergrator;
	min_output = _min_intergrator;
	max_output = _max_intergrator;
}
PI_controller::~PI_controller(){


}


