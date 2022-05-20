/*
 * FOC.h
 *
 *  Created on: Oct 1, 2019
 *      Author: LoganRosenmayer
 */

#ifndef FOC_H_
#define FOC_H_

#include "stm32h7xx_hal.h"


void rawdata_to_angle(int32_t rawdata,float &mech_angle,int32_t &electrical_angle,int32_t debug_offset,int32_t pole_pairs);
void dqz(float Ia,float Ib,float Ic,int32_t theta,float &Iq,float &Id );
void inv_dqz(float &Va,float &Vb,float &Vc,int32_t theta,float Vq,float Vd );
#endif /* FOC_H_ */
