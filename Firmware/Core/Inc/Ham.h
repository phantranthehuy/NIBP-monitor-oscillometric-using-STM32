/*
 * Ham.h
 *
 *  Created on: Dec 30, 2024
 *      Author: Admin
 */

#ifndef INC_HAM_H_
#define INC_HAM_H_

#include "main.h"
#include "Lcd.h"
void setup(float *maxpressure, uint8_t *meas_state, float *former, float *TH_sys, float *TH_rate,float *TH_dias, float *time_pulse, float *systolic, float *diastolic, float *total_pulse_period,
		float *pulse_per_min, uint8_t *sys_count, uint8_t *count_average, uint8_t *countpulse, float *Vref, float *DC_gain, float *accum_data, float *press_data, uint8_t *count);
void start_state(uint8_t *sys_count, float *pressure, float *accum_data, float *press_data, uint8_t *count, uint8_t *stop_count, uint8_t *meas_state,
		float *former, uint8_t *timerun_dias, float *time_pulse, uint8_t *timerate, uint8_t *timing, float *total_pulse_period, float *systolic,
		float *diastolic, float *pulse_per_min, uint8_t *count_average, uint8_t *countpulse, float TH_sys, uint8_t *currentState, uint8_t *timecount);
void read_adc(int Channel);
void inflate1_state(uint8_t *currentState);
void inflate2_state(float *pressure, float maxpressure, uint8_t *stop_count, uint8_t *currentState, uint8_t *timedeflate, float Vref);
void deflatestate(uint8_t *currentState);
void display_state(void);
void reset_state(uint8_t *currentState);
void pressuremeasure(uint8_t *meas_state, uint8_t *timing);
void sysmeasure(float Vref, float *pressure, uint8_t timing, uint8_t *meas_state, uint8_t *timecount, uint8_t *sys_count);
void syscal(void);
void diasmeasure(void);
void diascal(void);

#endif /* INC_HAM_H_ */
