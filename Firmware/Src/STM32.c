/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2; // Sử dụng ADC2 nếu cần
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2; // Sử dụng TIM2 thay cho Timer1
UART_HandleTypeDef huart1;

/* Definitions */
#define on GPIO_PIN_SET
#define off GPIO_PIN_RESET

// Define states for motor control
#define startState 0
#define inflate1State 1
#define inflate2State 2
#define deflateState 3
#define displayState 4
#define resetState 5
// Define states for Measure control
#define Sys_Measure 6
#define Sys_Cal 7
#define Rate_Measure 8
#define dias_Measure 9
#define dias_Cal 10

/* Pin Definitions */
#define valve_Pin GPIO_PIN_3 // PA3
#define valve_GPIO_Port GPIOA
#define motor_Pin GPIO_PIN_2 // PA2
#define motor_GPIO_Port GPIOA
#define bt_start_Pin GPIO_PIN_5  // PA5
#define bt_stop_Pin GPIO_PIN_6   // PA6
#define bt_resume_Pin GPIO_PIN_7 // PA7

#define ADC0_Channel ADC_CHANNEL_0 // PA0
#define ADC1_Channel ADC_CHANNEL_1 // PA1

// Declare variables for motor controls
unsigned char currentState;
unsigned int timepress0, timepress1, timepress2, timelcd;
// Declare variables for measuring and calculating value
float DC_gain;
unsigned char meas_state;
unsigned int timing, timerate, timerun_dias, timecount, timedeflate, timedisplay;
float maxpressure, pressure, accum_data, press_data;
unsigned char count, stop_count;

// ADC data variables
float Vref;
uint32_t data;
float adc_data, former;

// Define counters
unsigned char sys_count, count_average, countpulse;

// Declare rate measure variables
float time_pulse, pulse_period, total_pulse_period, pulse_per_min;

// Declare systolic and diastolic variables
float systolic, diastolic;

// Declare all the threshold values
float TH_sys, TH_rate, TH_dias;

// Buffer for UART communication
char uart_buf[50];
int uart_buf_len;

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

void start_state(void);
void read_adc(int Channel);
void inflate1_state(void);
void inflate2_state(void);
void deflate_state(void);
void display_state(void);
void reset_state(void);
void pressure_measure(void);

/* Main Function */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init(); // Sử dụng I2C cho LCD
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* Initialize LCD */
  lcd_init();

  /* Start Timer Interrupt */
  HAL_TIM_Base_Start_IT(&htim2);

  /* Initialize variables */
  timecount = 0;

  /* Set up các biến */
  maxpressure = 200; // Max huyết áp
  meas_state = Sys_Measure;
  TH_sys = 4.9;  // Ngưỡng dao động huyết áp tâm thu
  TH_rate = 2.3; // Ngưỡng dao động
  TH_dias = 4.6; // Huyết áp tâm trương
  timerun_dias = 0;
  time_pulse = 0;
  timerate = 0;

  timing = 40;
  timedisplay = 0;

  total_pulse_period = 0;
  systolic = 0;
  diastolic = 0;
  pulse_per_min = 0;

  sys_count = 0;
  count_average = 0;
  countpulse = 0;
  Vref = 3.3; // Vref của STM32 là 3.3V
  DC_gain = 105;

  accum_data = 0;
  press_data = 0;
  count = 0;

  /* Hiển thị thông điệp READY trên LCD */
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("READY");

  /* Send READY message over UART */
  uart_buf_len = sprintf(uart_buf, "READY\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);

  /* Infinite loop */
  while (1)
  {
    switch (currentState)
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
      deflate_state();
      break;
    case displayState:
      display_state();
      break;
    case resetState:
      reset_state();
      break;
    }
  }
}

/* Timer Interrupt Callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // Hàm này được gọi mỗi khi Timer2 tràn (ví dụ mỗi 1ms)
    // Triển khai mã ISR ở đây

    if (HAL_GPIO_ReadPin(GPIOA, bt_start_Pin) == GPIO_PIN_RESET)
      timepress0++;
    if (HAL_GPIO_ReadPin(GPIOA, bt_stop_Pin) == GPIO_PIN_RESET)
      timepress1++;
    if (HAL_GPIO_ReadPin(GPIOA, bt_resume_Pin) == GPIO_PIN_RESET)
      timepress2++;

    timecount++;
    timedeflate++;

    // Decrement each time task if they are not already zero
    if (timing > 0)
      timing--;

    // Run timerate for measuring heart rate
    if (timerate < 6000)
      timerate++;

    // Run timerun_dias
    if (timerun_dias < 2000)
      timerun_dias++;

    // Run time for the display
    if (timedisplay < 2000)
      timedisplay++;
  }
}

/* Hardware Initialization Functions */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Khởi tạo bộ dao động */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // Sử dụng HSE
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                   // Bật HSE
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;    // Không chia trước
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Bật PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // Nguồn PLL từ HSE
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;               // Nhân x9 (8MHz x9 = 72MHz)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Khởi tạo xung nhịp hệ thống */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Nguồn xung hệ thống từ PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // Không chia
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         // Chia 2 cho APB1 (36MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // Không chia cho APB2 (72MHz)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Kích hoạt xung cho GPIOA và GPIOB */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Cấu hình GPIO Output cho motor (PA2) và valve (PA3) */
  GPIO_InitStruct.Pin = motor_Pin | valve_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Output Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // Không kéo lên/kéo xuống
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Tốc độ thấp
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Cấu hình GPIO Input cho các nút nhấn (PA5, PA6, PA7) */
  GPIO_InitStruct.Pin = bt_start_Pin | bt_stop_Pin | bt_resume_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Chế độ Input
  GPIO_InitStruct.Pull = GPIO_PULLUP;     // Kéo lên nội
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Cấu hình chung cho ADC */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; // Chuyển đổi đơn kênh
  hadc1.Init.ContinuousConvMode = DISABLE;    // Chuyển đổi không liên tục
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Kích hoạt bằng phần mềm
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // Căn phải
  hadc1.Init.NbrOfConversion = 1;                   // Số kênh chuyển đổi
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* Cấu hình chung cho TIM2 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199; // Prescaler để giảm tần số
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9; // Tạo ngắt mỗi 1ms
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Cấu hình Clock Source */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Bật ngắt cho TIM2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_USART1_UART_Init(void)
{
  /* USART1 Initialization Function */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;                 // Tốc độ baud
  huart1.Init.WordLength = UART_WORDLENGTH_8B; // 8 bit dữ liệu
  huart1.Init.StopBits = UART_STOPBITS_1;      // 1 bit stop
  huart1.Init.Parity = UART_PARITY_NONE;       // Không parity
  huart1.Init.Mode = UART_MODE_TX_RX;          // Chế độ truyền và nhận
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  /* I2C1 Initialization Function */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000; // Tốc độ I2C 100kHz
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Error Handler */
void Error_Handler(void)
{
  while (1)
  {
    // Giữ vi điều khiển ở đây nếu có lỗi
  }
}

/* Retarget printf to USART1 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}