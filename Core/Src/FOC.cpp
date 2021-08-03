/*
 * FOC.c
 *
 *  Created on: Oct 1, 2019
 *      Author: LoganRosenmayer
 */
#include "FOC.h"



int32_t sin_lut_table[360] = {0,2,3,5,7,9,10,12,14,16,17,19,21,22,24,26,28,29,31,33,34,36,37,39,41,42,44,45,47,48,50,52,53,54,56,57,59,60,62,63,64,
		66,67,68,69,71,72,73,74,75,77,78,79,80,81,82,83,84,85,86,87,87,88,89,90,91,91,92,93,93,94,95,95,96,96,97,97,97,98,98,98,99,99,99,99,100,100,
		100,100,100,100,100,100,100,100,100,99,99,99,99,98,98,98,97,97,97,96,96,95,95,94,93,93,92,91,91,90,89,88,87,87,86,85,84,83,82,81,80,79,78,77,
		75,74,73,72,71,69,68,67,66,64,63,62,60,59,57,56,54,53,52,50,48,47,45,44,42,41,39,37,36,34,33,31,29,28,26,24,22,21,19,17,16,14,12,10,9,7,5,3,
		2,0,-2,-3,-5,-7,-9,-10,-12,-14,-16,-17,-19,-21,-22,-24,-26,-28,-29,-31,-33,-34,-36,-37,-39,-41,-42,-44,-45,-47,-48,-50,-52,-53,-54,-56,-57,
		-59,-60,-62,-63,-64,-66,-67,-68,-69,-71,-72,-73,-74,-75,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-87,-88,-89,-90,-91,-91,-92,-93,-93,-94,
		-95,-95,-96,-96,-97,-97,-97,-98,-98,-98,-99,-99,-99,-99,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-99,-99,-99,-99,-98,-98,-98,
		-97,-97,-97,-96,-96,-95,-95,-94,-93,-93,-92,-91,-91,-90,-89,-88,-87,-87,-86,-85,-84,-83,-82,-81,-80,-79,-78,-77,-75,-74,-73,-72,-71,-69,-68,
		-67,-66,-64,-63,-62,-60,-59,-57,-56,-54,-53,-52,-50,-48,-47,-45,-44,-42,-41,-39,-37,-36,-34,-33,-31,-29,-28,-26,-24,-22,-21,-19,-17,-16,-14,
		-12,-10,-9,-7,-5,-3,-2};
int32_t sqrt6 = 2449;
int32_t sqrt2 = 1414;
int32_t sqrt2_3 = 816;
int32_t sqrt1_6 = 408;
int32_t sqrt1_2 = 707;
/*
 * Function:	rawdata_to_angle
 * ----------------------
 * computers the power invariant DQZ transform using a LUT for sin and cos and fixed point numbers
 * rawdata: 12 bit value from encoder
 *
 * returns:angle in milli mechanical degrees
 */
void rawdata_to_angle(int32_t rawdata,int32_t &mech_angle,int32_t &electrical_angle,int32_t debug_offset,int32_t pole_pairs)
{
	int32_t mech_angle_temp = (879 * rawdata)/1000;
	electrical_angle = ((((mech_angle_temp)*pole_pairs+debug_offset*10)%3600)%3600)/10;
	mech_angle = mech_angle_temp/10;
}
//https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation basically ripped the math out of the implementation example
//angle should be in degrees
/*
 * Function:	dqz
 * ----------------------
 * computers the power invariant DQZ transform using a LUT for sin and cos and fixed point numbers
 * Ia: current of phase A in mA
 * Ib: current of phase B in mA
 * Ic: current of phase C in mA
 * theta: angle of motor in electrical degrees, must be between 0 and 359 inclusive
 * Iq: pointer to where the result of Iq in mA should be stored
 * Id: pointer to where the result of Id in mA should be stored
 *
 * returns:nothing
 */
void dqz(int32_t Ia,int32_t Ib,int32_t Ic,int32_t theta,int32_t *Iq,int32_t *Id )
{
	int32_t alpha = (2*Ia-Ib-Ic)*sqrt1_6;
	int32_t beta = (Ib - Ic) * sqrt1_2;
	int32_t si = sin_lut_table[theta%360];
	int32_t co = sin_lut_table[(theta+90)%360];
	*Id = (co*alpha+si*beta)/100000;
	*Iq = (co*beta-si*alpha)/100000;
}
/*
 * Function:	inv_dqz
 * ----------------------
 * computers the invariant DQZ transform using a LUT for sin and cos and fixed point numbers
 * Va: pointer to where the result of phase A in mV should be stored
 * Vb: pointer to where the result of phase B in mV should be stored
 * Vc: pointer to where the result of phase C in mV should be stored
 * theta: angle of motor in electrical degrees, must be between 0 and 359 inclusive
 * Vq: voltage of Vq in mV
 * Vd: voltage of Vd in mV
 *
 * returns:nothing
 */
void inv_dqz(int32_t *Va,int32_t *Vb,int32_t *Vc,int32_t theta,int32_t Vq,int32_t Vd )
{
	int32_t si=sin_lut_table[theta];
	int32_t co=sin_lut_table[(theta+90)%360];
	int32_t alpha = co*Vd- si *Vq;
	int32_t beta = si*Vd+co*Vq;
	int32_t Vb_temp = -sqrt1_6*alpha;
	*Va = (sqrt2_3*alpha)/100000;
	*Vc = (Vb_temp-sqrt1_2*beta)/100000;
	*Vb = (Vb_temp+sqrt1_2*beta)/100000;
}
