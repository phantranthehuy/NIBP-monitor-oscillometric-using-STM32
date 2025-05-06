#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#define on HIGH
#define off LOW
//define states for motor control
#define startState 0
#define inflate1State 1
#define inflate2State 2
#define deflateState 3
#define displayState 4
#define resetState 5
//define states for Measure control
#define Sys_Measure 6
#define Sys_Cal 7
#define Rate_Measure 8
#define dias_Measure 9 
#define dias_Cal 10

#define valve A4 // khai bao chan valve khi
#define motor A3 // k hai bao chan motor
#define bt_start A5 // nut nhan start
#define bt_stop A6
#define bt_resume A7
#define ADC0 A0
#define ADC1 A1
//define states for Measure control
//declare variable for motor controls
unsigned char currentState;
unsigned int timepress0, timepress1, timepress2, timelcd;      
//declare variable for measuring and calculating value
float DC_gain;
unsigned char meas_state;
unsigned int timing, timerate, timerun_dias, timecount, timedeflate, timedisplay; 
float  maxpressure, pressure,accum_data, press_data; 
unsigned char count, stop_count;

//ADC data variabls
float Vref;
int data;
float adc_data, former; 

//define counter
unsigned char sys_count,count_average, countpulse;

//declare rate measure variable
float time_pulse,pulse_period, total_pulse_period, pulse_per_min;

//declare systolic and diastolic variable
float systolic, diastolic;

//declare all the threshold values
float TH_sys, TH_rate, TH_dias;
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);
void start_state(void);
void read_adc(int Channel);
void inflate1_state(void);
void inflate2_state(void);
void deflatestate(void);
void display_state(void);
void reset_state(void);
void pressuremeasure(void);
void setup() {
  //khai bao serial
  Serial.begin(9600);
  //khai bao nut nhan
  pinMode(A5,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);
  //khai bao motor valve
  pinMode(motor,OUTPUT);
  pinMode(valve,OUTPUT);
  //Cai dat LCD

  lcd.begin(16,2);
  Serial.print("READY");
  timecount=0;         
  lcd.setCursor(0,0);
  lcd.print("White: Start");
  lcd.setCursor(0,1);
  lcd.print("Grey: Stop");
  
  //set up timer0
    cli();              // tắt ngắt toàn cục
    /* Reset Timer/Counter1 */
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    
    /* Setup Timer/Counter1 */
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // prescale = 64 and CTC mode 4
    OCR1A = 249;              // initialize OCR1A
    TIMSK1 = (1 << OCIE1A);     // Output Compare Interrupt Enable Timer/Counter1 channel A
    sei();                      // cho phép ngắt toàn cục   
    //ket thuc khai bao ngat          
                                 
    maxpressure = 200; // max huyet ap
    meas_state = Sys_Measure; 
    former=TH_sys-0.01;


  TH_sys=4.9; //nguong dao dong huyet ap tam thu
  TH_rate = 2.3; //nguong dao dong  
  TH_dias = 4.6; // huyet ap tam truong
  timerun_dias=0; 
  time_pulse=0;
  timerate=0;

  timing=40;
  timedisplay=0;

  total_pulse_period=0;
  systolic=0;
  diastolic=0;
  pulse_per_min=0;        
  

  sys_count=0;
  count_average=0;
  countpulse=0;
  Vref = 5.0;                  
  DC_gain=105;
  
  accum_data=0; 
  press_data=0; 
  count=0;
}

void loop() {
  switch(currentState)
    { 
      case startState:
         start_state();
         break;
      case inflate1State:
         inflate1_state();
         break;
      case inflate2State:
         inflate2_state();
         break;
      case deflateState:
         deflatestate();
         break;
      case displayState:
         display_state();
         break;
      case resetState:
         reset_state();
         break;

   }
}
//chuong trinh phuc vu ngat 
ISR (TIMER1_COMPA_vect) 
{ //Serial.println("Timer1 COMPA");
   //TCNT1 = 250;
   if(digitalRead(bt_start)) timepress0++;
   if((analogRead(bt_stop) >= 900)) timepress1++;  
   if(analogRead(bt_resume)>= 900) timepress2++;  
   timecount++;                      
   timedeflate++;
    //Decrement each time tast if they are not already zero
    
    //timing for sampling data at every 40 msec
   if(timing>0) --timing; 
  //-----------------------------------------------------
   //run time for different tasks

  //run timerate for measuring heart rate
   if(timerate<6000) ++timerate;
 
   //run timerun_dias
   if(timerun_dias<2000) ++timerun_dias;
 
   //if(countlcd) timelcd++; 
   
    //run time for the display
    if(timedisplay<2000) ++timedisplay;   
}
void start_state(void)
{   //Serial.println("start_state");
    sys_count=0;              
    pressure = 0;
    accum_data=0; 
    press_data=0; 
    count=0;
    stop_count=0; 
     
    maxpressure = 220; 
    meas_state = Sys_Measure; 
    former=TH_sys-0.01;

    timerun_dias=0;
    time_pulse=0;
    timerate=0;

    timing=40;

    total_pulse_period=0;
    systolic=0;
    diastolic=0;
    pulse_per_min=0;

    sys_count=0;
    count_average=0;
    countpulse=0;

  if((digitalRead(bt_start)) && (timepress0 > 30)){
//      countlcd = 1;
      timelcd = 0;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Inflating");
      currentState = inflate1State;
      timepress0 = 0;  
      timecount=0;
      //turn on motor and close the valve
      digitalWrite(valve,on);
      digitalWrite(motor,on);
  } 
}

void inflate1_state(void)
{  //Serial.println("inflate_state");       
  if(timecount>=200)
  {
    lcd.setCursor(0,1);
    lcd.print("presure = ");
    lcd.print(pressure);  
    timecount=0;
  }
  
  if((analogRead(bt_stop)>=900) && (timepress1 > 30))
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Emergency Stop");             
    //turn off motor and open the valve
    digitalWrite(motor, off);
    digitalWrite(valve, off);
    currentState = resetState;
    timepress1 = 0;
    //countlcd = 0;
    //Serial.println("tat van tat motor");
  }
  else
  {
    currentState = inflate2State;
  } 
    
}

void inflate2_state(void){   
    int data1;
    float adc_data1;
    data1 = analogRead(ADC1);Serial.println(data1);
    adc_data1 = (float)(((float)data1)/1023*Vref);
    pressure = (adc_data1/DC_gain)*6000;  //9375 he so chuyen doi can tinh  toan lai
    
    
    if(pressure>=maxpressure) stop_count++;   
    else stop_count = 0;
    
  if(stop_count>=5){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Deflating ");        
    lcd.print(pressure);
    //turn off motor but keep the valve
    digitalWrite(motor, off);
    delay(1000);
    currentState = deflateState;   
    timedeflate = 0;    
    lcd.setCursor(0,1);
    lcd.print(pressure);
  }
  else
  {
    currentState = inflate1State;   
  } 
  
}

void deflatestate(void)
{                                         
  if((analogRead(bt_stop)>=900) && (timepress1 > 30))
  { 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Emergency Stop");  
    lcd.setCursor(0,1);                
    lcd.print(pressure);  
    //turn off motor and open the valve
    digitalWrite(motor,off);
    digitalWrite(valve,off);
    currentState = resetState;
    timepress1 = 0;                            
    }
  //if(done) --> Display state
  if(currentState==deflateState) pressuremeasure(); //if still deflating, measure everything
}
void display_state(void)
{   
     
if(timedisplay<=1000)
{
    if(timecount>=200)
  {
  lcd.clear();
  timecount=0;
  lcd.setCursor(0,0);
  lcd.print("Sys"); 
  lcd.setCursor(7,0);
  lcd.print("Dias");
  lcd.setCursor(14,0);
  lcd.print("HR"); 
  lcd.setCursor(0,1);                    
  lcd.print(systolic);
  lcd.setCursor(7,1);
  lcd.print(diastolic); 
  lcd.setCursor(14,1);
  lcd.print(pulse_per_min); 
  } 
}
else if (timedisplay>1000&&timedisplay<2000)
{ 
  if(timecount>=200)
  { lcd.clear();              
    lcd.setCursor(0,0);
    lcd.print("Red: Resume");  
    timecount=0;
  }  
}

else
{
    timedisplay=0;
} 
  if((analogRead(bt_resume) >= 900)&&(timepress2 >30))
  {
    lcd.clear();       
    lcd.setCursor(0,0);
    lcd.print("White: Start");
    lcd.setCursor(0,1);
    lcd.print("Grey: Stop");
    currentState = startState;
    timepress2 = 0;         
    systolic=0;
    diastolic=0;
    pulse_per_min=0;
  
  } 
     
}
void reset_state(void)
{   
if(timedisplay<=1000)
{
    if(timecount>=200)
    {   
      timecount=0;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Emergency Stop"); 
  } 
}
else if (timedisplay>1000&&timedisplay<2000)
{ 
  if(timecount>=200)
  { lcd.clear();
    timecount=0;      
    lcd.setCursor(0,0);
    lcd.print("Red: resume"); 
  
  }  
}

else
{
    timedisplay=0;
}           
  if((analogRead(bt_resume) >= 900) && (timepress2 > 30))
  {
    lcd.clear();       
    lcd.setCursor(0,0);
    lcd.print("White: Start");
    lcd.setCursor(0,1);
    lcd.print("Grey: Stop");
    currentState = startState;
    timepress2 = 0;
  }
  
}
void pressuremeasure(void)
{
 switch (meas_state)
   {   
      case Sys_Measure:
         if(timing==0) sysmeasure(); //sampling signal at 40msec
         break;
         
      case Sys_Cal:
           if(timing==0) syscal();
           break;
           
      case Rate_Measure:
         if(timing==0) ratemeasure();
         break;
      
      case dias_Measure:
         diasmeasure();
         break;
         
      case dias_Cal:
         diascal();
         break;
        
    } //switch
    
}//pressuremeasure
void sysmeasure(void)
{
    int data1;
    float adc_data1;
    data1 = analogRead(ADC1);
   delay(10);
    adc_data1 = (float)(((float)data1)/1023*Vref);
    pressure = (adc_data1/DC_gain)*6000;  //9375 he so chuyen doi can tinh  toan lai
    Serial.println(pressure);
  if(timing==0)
    {
      read_adc(0);    
        } 
   if(sys_count>=2)
     { 
      meas_state = Sys_Cal;
      timecount=0;
      }
        
        if(timecount>=200)
        {  
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Measuring");
        lcd.setCursor(0,1);
        lcd.print("P = ");
        lcd.print(pressure);
        lcd.print("mmHg");
        timecount=0;
        } 
}

//***********************************************************
//this function is to calculate systolic pressure
void syscal(void)
{
   read_adc(1);
        
   if(timecount>=200){  
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Sys Cal");
      timecount=0;
   } 
        
        
}//syscal

//************************************************************
void ratemeasure(void)
{
  
    read_adc(0);
        //calculate the mean of pulse rate
      if(count_average==5)
      {
       pulse_period = total_pulse_period/5000;
       pulse_per_min= 60/pulse_period;  
       
      lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Pulse Rate");    
    lcd.setCursor(0,1);
    lcd.print(pulse_per_min);
    
       meas_state = dias_Measure;
       //then set timerun_dias=0
       //also reset count_average for the next operation
       count_average=0;
       timerun_dias=0;
      }  
}
//************************************************************
void diasmeasure(void)
{   int data1;
    float adc_data1;
    read_adc(0); 
    data1 = analogRead(ADC1);
    adc_data1 = (float)(((float)data1)/1023*Vref);
    pressure = (adc_data1/DC_gain)*6000;  //9375 he so chuyen doi can tinh  toan lai
    if(timecount>=200)
        {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Dia measuring");
          //lcd.setCursor(0,1);
          //lcd.print("P = ");
          //lcd.print(pressure);
          //lcd.print(" mmHg");
          timecount = 0;  
        }
    
         
}//dias measure
//*************************************************************

void diascal(void)
{
    //choose ADC1 for reading DC 
    read_adc(1);        
        if(timecount>=200)
        {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Dias_Cal");    
        timecount=0;
        } 

}
void read_adc(int Channel)  
{
  switch (Channel){
  case 0:
    data = analogRead(ADC0);
    //Serial.println(data);
    break;
  case 1:
    data = analogRead(ADC1);
    break;
  }
 //then calculate adc_data into float;
    adc_data = (float)(((float)data)/1023*Vref);
    Serial.print(adc_data);
    Serial.print(" ");Serial.println(3.9);
 
 //if signal is above threshold, go to calculate systolic pressure
 if(meas_state ==Sys_Measure)
   {   
     
     if(former<=TH_sys && adc_data>TH_sys){
      sys_count++;           
      //Serial.println("detect");   
     }
     former = adc_data;            
      
    }
 //-----------------------------------------------------------
 else if(meas_state==Sys_Cal)
  { 
  
    if(count<4)
    {
     accum_data=accum_data+adc_data;
     count++;
    }
    if(count==4)
    {
    press_data=accum_data/4;
    systolic = (press_data/DC_gain)*6000;//calculate from adc_data
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
      timerate=0;
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
    if(timerun_dias<2000)
      {
      if(adc_data>TH_dias)
      { timerun_dias=0; //reset time if the signal 
      //is still greater than threshold (so it will never reach 1999)
      //if it doesn't reset,the time will stuck at 1999
        
        //lcd.clear();
        //lcd.setCursor(0,0);
        //lcd.print("Dias measure");   
      }
      }
      
    if(timerun_dias>=2000)
      {  //Serial.println("timerun_dias > 2000"); 
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
