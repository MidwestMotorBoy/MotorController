/*
 * controllers.cpp
 *
 *  Created on: Jun 26, 2021
 *      Author: LoganRosenmayer
 */
#include "PI_controller.h"
/*
 * Class:	PI_controller
 * ----------------------
 * computers the power invariant DQZ transform using a LUT for sin and cos and fixed point numbers
 * input: Target value for system
 * feedback: Measurement of current system state
 * ts: sample time (microseconds)
 *
 * returns:nothing
 */
PI_controller::PI_controller(){
	ki = 0;
	kp = 0;
	min_intergrator = 0;
	max_intergrator = 0;
	min_output = 0;
	max_output = 0;
	error_intergration=0;
	old_ref=0;
	slew=999999;
}
PI_controller::PI_controller(float _ki, float _kp,float _min_output,float _max_output){
	ki = _ki;
	kp = _kp;
	min_intergrator = _min_output/_ki;
	max_intergrator = _max_output/_ki;
	min_output = _min_output;
	max_output = _max_output;
	error_intergration=0;
	old_ref=0;
	slew=999999;
}
PI_controller::PI_controller(float _ki, float _kp,float _min_output,float _max_output,float _slew){
	ki = _ki;
	kp = _kp;
	min_intergrator = _min_output/_ki;
	max_intergrator = _max_output/_ki;
	min_output = _min_output;
	max_output = _max_output;
	error_intergration=0;
	old_ref=0;
	slew=_slew;
}
PI_controller::PI_controller(float _ki, float _kp,float _min_intergrator,float _max_intergrator,
		float _min_output,float _max_output){
	ki = _ki;
	kp = _kp;
	min_intergrator = _min_intergrator;
	max_intergrator = _max_intergrator;
	min_output = _min_output;
	max_output = _max_output;
	error_intergration=0;
	old_ref=0;
	slew=999999;
}
PI_controller::PI_controller(float _ki, float _kp,float _min_intergrator,float _max_intergrator,
		float _min_output,float _max_output,float _slew){
	ki = _ki;
	kp = _kp;
	min_intergrator = _min_intergrator;
	max_intergrator = _max_intergrator;
	min_output = _min_output;
	max_output = _max_output;
	error_intergration=0;
	old_ref=0;
	slew=_slew;
}
PI_controller::~PI_controller(){


}
/*
 * Function:	PI_controller::update
 * ----------------------
 * Updates discrete PI controller
 * input: Target value for system
 * feedback: Measurement of current system state
 * ts: sample time (microseconds)
 *
 * returns:nothing
 */
float PI_controller::update(float ref,float feedback,float ts){
		float change_in_ref = ref-old_ref;
		float max_change = slew*ts;
		if(change_in_ref>max_change){
			ref = old_ref+max_change;
		}
		else if(change_in_ref<-max_change){
			ref = old_ref-max_change;
		}
		old_ref=ref;
		float error=ref-feedback;
		error_intergration=error_intergration+error*ts;
		if(min_intergrator > error_intergration){
			error_intergration=min_intergrator;
		}
		if(max_intergrator < error_intergration){
			error_intergration=max_intergrator;
		}
		float output = ki*error_intergration+error*kp;
		if(min_output > output){
			output = min_output;
		}
		if(max_output < output){
			output = max_output;
		}
		return(output);
}
//float PI_controller::update(float input,float feedback,float ts){
//		float error=input-feedback;
//		this->error_intergration+=error*ts/1000;
//		if(this->min_intergrator > this->error_intergration/1000){
//			this->error_intergration=this->min_intergrator*1000;
//		}
//		if(this->max_intergrator < this->error_intergration/1000){
//			this->error_intergration=this->max_intergrator*1000;
//		}
//		float output = this->ki*this->error_intergration/1000+error*this->kp;
//		if(this->min_output > output){
//			output = this->min_output;
//		}
//		if(this->max_output < output){
//			output = this->max_output;
//		}
//		return(output);
//}

