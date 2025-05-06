/*
 * Ham.c
 *
 *  Created on: Dec 22, 2024
 *      Author: Admin
 */
#include "Ham.h"
#include "Lcd.h"
#include "stm32f1xx_hal.h"
void setup(float* maxpressure, uint8_t *meas_state, float *former, float *TH_sys, float *TH_rate,float *TH_dias, float *time_pulse, float *systolic, float *diastolic, float *total_pulse_period,
		float *pulse_per_min, uint8_t *sys_count, uint8_t *count_average, uint8_t *countpulse, float *Vref, float *DC_gain, float *accum_data, float *press_data, uint8_t *count)
{
	  *maxpressure = 200; // max huyet ap
	  *meas_state = Sys_Measure;
	  *former=TH_sys-0.01;

	    *TH_sys=4.9; //nguong dao dong huyet ap tam thu
	    *TH_rate = 2.3; //nguong dao dong
	    *TH_dias = 4.6; // huyet ap tam truong
	    *timerun_dias=0;
	    *time_pulse=0;
	    *timerate=0;

	    *timing=40;
	    *timedisplay=0;

	    *total_pulse_period=0;
	    *systolic=0;
	    *diastolic=0;
	    *pulse_per_min=0;


	    *sys_count=0;
	    *count_average=0;
	    *countpulse=0;
	    *Vref = 3.3;
	    *DC_gain=66;

	    *accum_data=0;
	    *press_data=0;
	    *count=0;
}
void start_state(uint8_t *sys_count, float *pressure, float *accum_data, float *press_data, uint8_t *count, uint8_t *stop_count, uint8_t *meas_state,
		float *former, uint8_t *timerun_dias, float *time_pulse, uint8_t *timerate, uint8_t *timing, float *total_pulse_period, float *systolic,
		float *diastolic, float *pulse_per_min, uint8_t *count_average, uint8_t *countpulse, float TH_sys, uint8_t *currentState, uint8_t *timecount)
{
	*sys_count=0;
    *pressure = 0;
    *accum_data=0;           //dữ liệu tích luỹ
    *press_data=0;
    *count=0;
    *stop_count=0;

    *meas_state = Sys_Measure;
    *former=TH_sys-0.01;

    *timerun_dias=0;
    *time_pulse=0;
    *timerate=0;

    *timing=40;          //thời gian lấy mẫu 40ms

    *total_pulse_period=0;
    *systolic=0;
    *diastolic=0;
    *pulse_per_min=0;    //nhịp tim trong 1 phút

    *sys_count=0;
    *count_average=0;
    *countpulse=0;
    if(x1 == 1)				//nút start1 nhấn
    {
    	*maxpressure = 220;
    	*currentState = inflate1State;
    	*timecount = 0;
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);				// bật motor, valve
    }
    if(x2 == 1)				//nút start2 nhấn
	{
		*maxpressure = 200;
		*currentState = inflate1State;
		*timecount = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);				// bật motor, valve
	}
}

void inflate1_state(uint8_t *currentState)
{
	if(x2 == 1)   //nút stop nhấn
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	//tắt motor, valve
		*currentState = resetState;
	}
	else
	{
		*currentState = inflate2State;
	}
}

void inflate2_state(float *pressure, float maxpressure, uint8_t *stop_count, uint8_t *currentState, uint8_t *timedeflate, float Vref)
{
	int data0;
	float adc_data0;
	data0 = HAL_ADC_GetValue(&hadc1);			//gọi lần đầu để lấy từ ADC kênh 0
	adc_data0 = (float)(((float)data0)/1023*Vref);
	*pressure = (adc_data0/DC_gain)*6000;
	if(*pressure >= maxpressure) *stop_count++;
	else *stop_count = 0;
	if(stop_count>=5){              //nếu đã vượt qua mức max và ổn định thì xả hơi
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);     //tắt motor
	    *currentState = deflateState;          //chuyển qua quá trình xả hơi
	    *timedeflate = 0;
	  }
	  else
	  {
	    *currentState = inflate1State;       //nếu chưa vượt quá mức max thì quay lại bơm hơi
	  }
}

void deflatestate(uint8_t *currentState)
{
	if(x2 == 1)   //nút stop nhấn
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	//tắt motor, valve
			*currentState = resetState;
		}
	if(currentState==deflateState) pressuremeasure();
}


void display_state()
{

}


void reset_state(uint8_t *currentState)
{
	HAL_Delay(5000);       //chờ 5s để xả hết hơi
	*currentState = start_state;
}



void pressuremeasure(uint8_t *meas_state, uint8_t *timing)
{
	switch(*meas_state)
	    {
	        case Sys_Measure:
	            if(*timing == 0) {
	                sysmeasure();  // sampling signal at 40msec
	                *timing = 1;  // Đặt lại timing để tránh việc gọi lại ngay lập tức
	            }
	            break;

	        case Sys_Cal:
	            if(*timing == 0) {
	                syscal();
	                *timing = 1;  // Đặt lại timing
	            }
	            break;

	        case dias_Measure:
	            diasmeasure();
	            break;

	        case dias_Cal:
	            diascal();
	            break;

	        default:
	            // Xử lý các trường hợp mặc định
	            break;
	    }
}


void sysmeasure(float Vref, float *pressure, uint8_t timing, uint8_t *meas_state, uint8_t *timecount, uint8_t *sys_count)
{
	int data0;
	float adc_data0;
	data0 = HAL_ADC_GetValue(&hadc1);
	HAL_DELAY(10);
	adc_data0 = (float)(((float)data0)/1023*Vref);        //chuyển đổi về điện áp
	pressure = (adc_data0/DC_gain)*6000;
	if(timing == 0)
	{
	  read_adc(1);
	}
	if(*sys_count >= 2)
	{
	  meas_state = Sys_Cal;
	  timecount = 0;
	}

}

void syscal(void)
{
	read_adc(0);
}

void diasmeasure(float Vref, float *pressure)
{
	int data0;
	float adc_data0;
	read_adc(1);
	data0 = HAL_ADC_GetValue(&hadc1);
	adc_data0 = (float)(((float)data0)/1023*Vref);
	pressure = (adc_data0/DC_gain)*6000;  //9375 he so chuyen doi can tinh  toan lai
}


void diascal(void)
{
	read_adc(0);
}



void read_adc(int Channel, float Vref, uint8_t *meas_state, float *former, float TH_sys, uint8_t *sys_count, uint8_t *count, float *accum_data, float *press_data, float *systolic)
{
  int data;
  switch (Channel)
  {
  case 0:
    data = HAL_ADC_GetValue(&hadc1);
    data = HAL_ADC_GetValue(&hadc1);			// đọc từ kênh 1 phải gọi 2 lần hàm này
    //Serial.println(data);
    break;
  case 1:
    data = HAL_ADC_GetValue(&hadc1);			//đọc từ kênh 0
    break;
  }
 //then calculate adc_data into float;
    adc_data = (float)(((float)data)/1023*Vref);

 //if signal is above threshold, go to calculate systolic pressure
 if(meas_state == Sys_Measure)
   {

     if(former <= TH_sys && adc_data > TH_sys){
      sys_count++;
      //Serial.println("detect");
     }
     former = adc_data;

    }
 //-----------------------------------------------------------
 else if(meas_state == Sys_Cal)
  {

    if(count < 4)
    {
     accum_data = accum_data + adc_data;        // ĐÁNG LẼ PHẢI LÀ ADC_DATA TỪ KÊNH 0 CHỨ
     count++;
    }
    if(count==4)
    {
    press_data = accum_data / 4;
    systolic = (press_data / DC_gain)*6000;//calculate from adc_data      //ĐÁNG LẼ PHẢI TỪ KÊNH 0
    meas_state = Rate_Measure;
    countpulse=0;
    former = 1.9; //set the initial point for rate measuring
    count_average=0;
    }
  }
 //----------------------------------------------------------
 else if(meas_state==Rate_Measure)
 {
  if(count_average<5)
  {

   if(former<TH_rate && adc_data>TH_rate && countpulse==0)
    {
      timerate=0;     // THỜI GIAN 1 NHỊP TIM
      countpulse=1;
      former=adc_data;
      }

   if(former<TH_rate && adc_data>TH_rate && countpulse==1)
      {
      total_pulse_period=total_pulse_period+timerate;
      timerate=0;
      count_average++; //finish reading one period
      }

   }//count_average

   former=adc_data;

}// else if(meas_state=Rate_Measure)
//-------------------------------------------------------------
else if(meas_state==dias_Measure)
{    /*Serial.println(adc_data);
     Serial.print("Timerun_dias = ");
     Serial.println(timerun_dias);*/
    if(timerun_dias < 2000)
      {
      if(adc_data > TH_dias)
      {
    	  timerun_dias = 0;
      }

      }


    if(timerun_dias >= 2000)
      {
      meas_state = dias_Cal;//if done go back to Sys_Measure to be ready for next opt
      }

}
//-------------------------------------------------------------
else if(meas_state==dias_Cal)
{
    diastolic = (adc_data/DC_gain)*6000;//calculate from adc_data
    meas_state = Sys_Measure;
    currentState = displayState;
    //open valve
    digitalWrite(valve,off);

}

 timing = 40;//set time for another conversion

}// end of read ADC
