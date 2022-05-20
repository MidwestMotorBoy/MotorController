/*
 * FOC.c
 *
 *  Created on: Oct 1, 2019
 *      Author: LoganRosenmayer
 */
#include "FOC.h"



//int32_t sin_lut_table[360] = {0,2,3,5,7,9,10,12,14,16,17,19,21,22,24,26,28,29,31,33,34,36,37,39,41,42,44,45,47,48,50,52,53,54,56,57,59,60,62,63,64,
//		66,67,68,69,71,72,73,74,75,77,78,79,80,81,82,83,84,85,86,87,87,88,89,90,91,91,92,93,93,94,95,95,96,96,97,97,97,98,98,98,99,99,99,99,100,100,
//		100,100,100,100,100,100,100,100,100,99,99,99,99,98,98,98,97,97,97,96,96,95,95,94,93,93,92,91,91,90,89,88,87,87,86,85,84,83,82,81,80,79,78,77,
//		75,74,73,72,71,69,68,67,66,64,63,62,60,59,57,56,54,53,52,50,48,47,45,44,42,41,39,37,36,34,33,31,29,28,26,24,22,21,19,17,16,14,12,10,9,7,5,3,
//		2,0,-2,-3,-5,-7,-9,-10,-12,-14,-16,-17,-19,-21,-22,-24,-26,-28,-29,-31,-33,-34,-36,-37,-39,-41,-42,-44,-45,-47,-48,-50,-52,-53,-54,-56,-57,
//		-59,-60,-62,-63,-64,-66,-67,-68,-69,-71,-72,-73,-74,-75,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-87,-88,-89,-90,-91,-91,-92,-93,-93,-94,
//		-95,-95,-96,-96,-97,-97,-97,-98,-98,-98,-99,-99,-99,-99,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-99,-99,-99,-99,-98,-98,-98,
//		-97,-97,-97,-96,-96,-95,-95,-94,-93,-93,-92,-91,-91,-90,-89,-88,-87,-87,-86,-85,-84,-83,-82,-81,-80,-79,-78,-77,-75,-74,-73,-72,-71,-69,-68,
//		-67,-66,-64,-63,-62,-60,-59,-57,-56,-54,-53,-52,-50,-48,-47,-45,-44,-42,-41,-39,-37,-36,-34,-33,-31,-29,-28,-26,-24,-22,-21,-19,-17,-16,-14,
//		-12,-10,-9,-7,-5,-3,-2};
float sin_lut_table[360] = {0.000000,0.017452,0.034899,0.052336,0.069756,0.087156,0.104528,0.121869,0.139173,0.156434,0.173648,0.190809,0.207912,
		0.224951,0.241922,0.258819,0.275637,0.292372,0.309017,0.325568,0.342020,0.358368,0.374607,0.390731,0.406737,0.422618,0.438371,0.453990,
		0.469472,0.484810,0.500000,0.515038,0.529919,0.544639,0.559193,0.573576,0.587785,0.601815,0.615661,0.629320,0.642788,0.656059,0.669131,
		0.681998,0.694658,0.707107,0.719340,0.731354,0.743145,0.754710,0.766044,0.777146,0.788011,0.798636,0.809017,0.819152,0.829038,0.838671,
		0.848048,0.857167,0.866025,0.874620,0.882948,0.891007,0.898794,0.906308,0.913545,0.920505,0.927184,0.933580,0.939693,0.945519,0.951057,
		0.956305,0.961262,0.965926,0.970296,0.974370,0.978148,0.981627,0.984808,0.987688,0.990268,0.992546,0.994522,0.996195,0.997564,0.998630,
		0.999391,0.999848,1.000000,0.999848,0.999391,0.998630,0.997564,0.996195,0.994522,0.992546,0.990268,0.987688,0.984808,0.981627,0.978148,
		0.974370,0.970296,0.965926,0.961262,0.956305,0.951057,0.945519,0.939693,0.933580,0.927184,0.920505,0.913545,0.906308,0.898794,0.891007,
		0.882948,0.874620,0.866025,0.857167,0.848048,0.838671,0.829038,0.819152,0.809017,0.798636,0.788011,0.777146,0.766044,0.754710,0.743145,
		0.731354,0.719340,0.707107,0.694658,0.681998,0.669131,0.656059,0.642788,0.629320,0.615661,0.601815,0.587785,0.573576,0.559193,0.544639,
		0.529919,0.515038,0.500000,0.484810,0.469472,0.453990,0.438371,0.422618,0.406737,0.390731,0.374607,0.358368,0.342020,0.325568,0.309017,
		0.292372,0.275637,0.258819,0.241922,0.224951,0.207912,0.190809,0.173648,0.156434,0.139173,0.121869,0.104528,0.087156,0.069756,0.052336,
		0.034899,0.017452,0.000000,-0.017452,-0.034899,-0.052336,-0.069756,-0.087156,-0.104528,-0.121869,-0.139173,-0.156434,-0.173648,-0.190809,
		-0.207912,-0.224951,-0.241922,-0.258819,-0.275637,-0.292372,-0.309017,-0.325568,-0.342020,-0.358368,-0.374607,-0.390731,-0.406737,-0.422618,
		-0.438371,-0.453990,-0.469472,-0.484810,-0.500000,-0.515038,-0.529919,-0.544639,-0.559193,-0.573576,-0.587785,-0.601815,-0.615661,-0.629320,
		-0.642788,-0.656059,-0.669131,-0.681998,-0.694658,-0.707107,-0.719340,-0.731354,-0.743145,-0.754710,-0.766044,-0.777146,-0.788011,-0.798636,
		-0.809017,-0.819152,-0.829038,-0.838671,-0.848048,-0.857167,-0.866025,-0.874620,-0.882948,-0.891007,-0.898794,-0.906308,-0.913545,-0.920505,
		-0.927184,-0.933580,-0.939693,-0.945519,-0.951057,-0.956305,-0.961262,-0.965926,-0.970296,-0.974370,-0.978148,-0.981627,-0.984808,-0.987688,
		-0.990268,-0.992546,-0.994522,-0.996195,-0.997564,-0.998630,-0.999391,-0.999848,-1.000000,-0.999848,-0.999391,-0.998630,-0.997564,-0.996195,
		-0.994522,-0.992546,-0.990268,-0.987688,-0.984808,-0.981627,-0.978148,-0.974370,-0.970296,-0.965926,-0.961262,-0.956305,-0.951057,-0.945519,
		-0.939693,-0.933580,-0.927184,-0.920505,-0.913545,-0.906308,-0.898794,-0.891007,-0.882948,-0.874620,-0.866025,-0.857167,-0.848048,-0.838671,
		-0.829038,-0.819152,-0.809017,-0.798636,-0.788011,-0.777146,-0.766044,-0.754710,-0.743145,-0.731354,-0.719340,-0.707107,-0.694658,-0.681998,
		-0.669131,-0.656059,-0.642788,-0.629320,-0.615661,-0.601815,-0.587785,-0.573576,-0.559193,-0.544639,-0.529919,-0.515038,-0.500000,-0.484810,
		-0.469472,-0.453990,-0.438371,-0.422618,-0.406737,-0.390731,-0.374607,-0.358368,-0.342020,-0.325568,-0.309017,-0.292372,-0.275637,-0.258819,
		-0.241922,-0.224951,-0.207912,-0.190809,-0.173648,-0.156434,-0.139173,-0.121869,-0.104528,-0.087156,-0.069756,-0.052336,-0.034899,-0.017452
};
float sqrt6 = 2.4495f;
float sqrt2 = 1.414f;
float sqrt2_3 = 0.816f;
float sqrt1_6 = 0.408f;
float sqrt1_2 = 0.707f;
/*
 * Function:	rawdata_to_angle
 * ----------------------
 * computers the power invariant DQZ transform using a LUT for sin and cos and fixed point numbers
 * rawdata: 12 bit value from encoder
 *
 * returns:angle in milli mechanical degrees
 */
void rawdata_to_angle(int32_t rawdata,float &mech_angle,int32_t &electrical_angle,int32_t debug_offset,int32_t pole_pairs)
{
	mech_angle = 360-rawdata*0.087890625000000f;
	electrical_angle = ((int32_t)(mech_angle*pole_pairs+debug_offset))%360;
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
void dqz(float Ia,float Ib,float Ic,int32_t theta,float &Iq,float &Id )
{
	float alpha = (2*Ia-Ib-Ic)*sqrt1_6;
	float beta = (Ib - Ic) * sqrt1_2;
	float si = sin_lut_table[theta%360];
	float co = sin_lut_table[(theta+90)%360];
	Id = (co*alpha+si*beta);
	Iq = (co*beta-si*alpha);
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
void inv_dqz(float &Va,float &Vb,float &Vc,int32_t theta,float Vq,float Vd )
{
	float si=sin_lut_table[theta];
	float co=sin_lut_table[(theta+90)%360];
	float alpha = co*Vd- si *Vq;
	float beta = si*Vd+co*Vq;
	float Vb_temp = -sqrt1_6*alpha;
	Va = (sqrt2_3*alpha);
	Vc = (Vb_temp-sqrt1_2*beta);
	Vb = (Vb_temp+sqrt1_2*beta);
}
