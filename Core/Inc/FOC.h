/*
 * FOC.h
 *
 *  Created on: Oct 1, 2019
 *      Author: LoganRosenmayer
 */

#ifndef FOC_H_
#define FOC_H_

#include "stm32h7xx_hal.h"


int32_t rawdata_to_angle(int32_t rawdata);
void dqz(int32_t Ia,int32_t Ib,int32_t Ic,int32_t theta,int32_t *Iq,int32_t *Id );
void inv_dqz(int32_t *Va,int32_t *Vb,int32_t *Vc,int32_t theta,int32_t Vq,int32_t Vd );
#endif /* FOC_H_ */
