/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max30102.h"
#include "algorithm.h"
#include <stdio.h>
#include "Adafruit_GPS.h"
	/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BRIGHTNESS 255

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate,lati,logi,alti;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t temporary_xbee[26];
char hex_str[((45-1)*2)];
uint32_t un_min, un_max;
uint32_t un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int i;
int32_t n_brightness;
float f_temp;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
uint8_t temporary;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	int i = 0;
	if (huart->Instance == USART2)  {
		
		HAL_UART_Receive_IT(&huart2, &temporary, 1);
		read(temporary);
		if ( newNMEAreceived() ) {
					 if ( !parse(lastNMEA()) ) {       
							return;   
					 }  
				//printf("%s\n", lastNMEA());
					 if (true) {
           
           printf("Time: %d:%d:%d.%u\n", hour, minute, seconds, milliseconds);
           printf("Date: %d/%d/20%d\n", day, month, year);
           printf("Fix: %d\n", (int) fix);
           printf("Quality: %d\n", (int) fixquality);
           if (fix) {
               printf("Location: %5.2f%c, %5.2f%c\n", latitude, lat, longitude, lon);
               printf("Speed: %5.2f knots\n", speed);
               printf("Angle: %5.2f\n", angle);
               printf("Altitude: %5.2f\n", altitude);
               printf("Satellites: %d\n", satellites);
						}
					} 
		}
	}else {
		HAL_UART_Receive_IT(&huart1, (uint8_t *) &temporary_xbee, 26);
		printf("%s\n", temporary_xbee);

	}
	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
}


void calibrate()
{
	int fail_number = 0;
  un_min=0x3FFFF;
  un_max=0;
	for(int i=0;i<n_ir_buffer_length;i++)
	{
			HAL_Delay(20);
			
			bool ok = maxim_max30102_read_fifo(hi2c1, (aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
			if(!ok)
				fail_number += 1;
			
			if(fail_number > 5){
				maxim_max30102_reset(hi2c1);
				fail_number = 0;
			}
				
			if(un_min>aun_red_buffer[i])
					un_min=aun_red_buffer[i];    //update signal min
			if(un_max<aun_red_buffer[i])
					un_max=aun_red_buffer[i];    //update signal max
			
			if(aun_ir_buffer[i] < 50000 && ok ){
				/*LCD1602_clear();
				LCD1602_print("Put your finger!");*/
				i = 0;
			} else {
				/*LCD1602_clear();
				LCD1602_print("Calibr. started");*/
			}
	}
	un_prev_data=aun_red_buffer[i];

}
void string2hexString(char* input, char* output)
{
    int loop;
    int i; 
    
    i=0;
    loop=0;
    
    while(input[loop] != '\0')
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	__USART2_CLK_ENABLE();
	__USART1_CLK_ENABLE();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	/////////////////////////GPS////////////////////////////////////////
	
	
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
	 
	HAL_Delay(10);
	//printf("\nthe size is: %d\n",sizeof(trans));
	HAL_UART_Receive_IT(&huart2, &temporary, 1);
	HAL_Delay(10);
  HAL_UART_Receive_IT(&huart1, (uint8_t *) &temporary_xbee, 26);
	

	//HAL_UART_Transmit_IT(&huart3, (uint8_t *) &temporary_xbee, 250);
	//HAL_Delay(10);

	Adafruit_GPS(huart2);
	

	HAL_Delay(10);
	
	sendCommand(huart2, PMTK_SET_BAUD_9600);
	HAL_Delay(10);
	sendCommand(huart2, PMTK_SET_NMEA_OUTPUT_RMCGGA);
	HAL_Delay(10);
	//sendCommand(huart2, PMTK_SET_NMEA_UPDATE_10HZ);
	sendCommand(huart2, PMTK_API_SET_FIX_CTL_5HZ);

	HAL_Delay(10);
	sendCommand(huart2, PGCMD_ANTENNA);
	HAL_Delay(10);

	printf(PMTK_Q_RELEASE);
	printf("Connection established at 9600 baud...\n");
	HAL_Delay(1);
	
	//////////////////////////////////////////////////////////////////
	
	for(uint8_t i=0;i<255;i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,10)==HAL_OK)
		{
			//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
			break;
		}			
		
	}
	
	maxim_max30102_restart(hi2c1);
	maxim_max30102_init(hi2c1);

	n_brightness=0;

	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
	
	calibrate();
	uint8_t trans[45]="H=10, S=20, 00.000000N, 00.000000E, 0000.00M";
	//trans[19]=(char)n_heart_rate;
	
	lati=(uint32_t)(latitude*10000);
	logi=(uint32_t)(longitude*10000);
	alti=(uint32_t)(altitude*100);
	lati=38742461;
	logi=35478911;
	alti=105412;
	char c = n_heart_rate%10+'0';
	trans[3]=c;
	c = ((n_heart_rate-(n_heart_rate%10))/10)%10+'0';
	trans[2]=c;
	c = n_sp02%10+'0';
	trans[9]=c;
	c = ((n_sp02-(n_sp02%10))/10)%10+'0';
	trans[8]=c;
	
	c=lati%10+'0';
	trans[20]=c;
	c = ((lati-(lati%10))/10)%10+'0';
	trans[19]=c;
	c = ((lati-(lati%100))/100)%10+'0';
	trans[18]=c;
	c = ((lati-(lati%1000))/1000)%10+'0';
	trans[17]=c;
	c = ((lati-(lati%10000))/10000)%10+'0';
	trans[16]=c;
	c = ((lati-(lati%100000))/100000)%10+'0';
	trans[15]=c;
	c = ((lati-(lati%1000000))/1000000)%10+'0';
	trans[13]=c;
	c = ((lati-(lati%10000000))/10000000)%10+'0';
	trans[12]=c;
	
	c=logi%10+'0';
	trans[32]=c;
	c = ((logi-(logi%10))/10)%10+'0';
	trans[31]=c;
	c = ((logi-(logi%100))/100)%10+'0';
	trans[30]=c;
	c = ((logi-(logi%1000))/1000)%10+'0';
	trans[29]=c;
	c = ((logi-(logi%10000))/10000)%10+'0';
	trans[28]=c;
	c = ((logi-(logi%100000))/1000000)%10+'0';
	trans[27]=c;
	c = ((logi-(logi%1000000))/1000000)%10+'0';
	trans[25]=c;
	c = ((logi-(logi%10000000))/10000000)%10+'0';
	trans[24]=c;
	
	c=alti%10+'0';
	trans[42]=c;
	c = ((alti-(alti%10))/10)%10+'0';
	trans[41]=c;
	c = ((alti-(alti%100))/100)%10+'0';
	trans[39]=c;
	c = ((alti-(alti%1000))/1000)%10+'0';
	trans[38]=c;
	c = ((alti-(alti%10000))/10000)%10+'0';
	trans[37]=c;
	c = ((alti-(alti%100000))/100000)%10+'0';
	trans[36]=c;
	//string2hexString((char*)&trans,hex_str);
	//uint8_t sendd[118]="7E003700010013A200419A292A00483D30302C20533D30302C2030302E3030303030304E2C2030302E303030303030452C20303030302E30304D";
	//sendd[116]='8';
	//sendd[117]='C';
	/*for(int j=0;j<87;j++){
	sendd[j+28]=hex_str[j];
	}*/
	
	HAL_UART_Transmit_IT(&huart1,trans,sizeof(trans));
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
	
	
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	i =  0;
  int last_i = 0;
	int fail_number = 0;
	bool need_calibration = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		// THIS IS AN INTERRUPT. WE USE INTERRUPTS EVERYWHERE :) 	
		
    /* USER CODE BEGIN 3 */
		HAL_Delay(10);
		
		if(last_i > 400){
			last_i = 0;
		}
		//take 100 sets of samples before calculating the heart rate.
		for(int i=last_i;i<last_i+100;i++)
		{
			  if(i == 0)
					un_prev_data=aun_red_buffer[499];
				else
					un_prev_data=aun_red_buffer[i-1];

				
				HAL_Delay(20);
				bool ok = maxim_max30102_read_fifo(hi2c1,(aun_red_buffer+i), (aun_ir_buffer+i));
				if(!ok)
					fail_number += 1;
				
				if(fail_number > 5){
					maxim_max30102_reset(hi2c1);
					fail_number = 0;
				}
				if(aun_red_buffer[i]>un_prev_data)
				{
						f_temp=aun_red_buffer[i]-un_prev_data;
						f_temp/=(un_max-un_min);
						f_temp*=MAX_BRIGHTNESS;
						n_brightness-=(int)f_temp;
						if(n_brightness<0)
								n_brightness=0;
				}
				else
				{
						f_temp=un_prev_data-aun_red_buffer[i];
						f_temp/=(un_max-un_min);
						f_temp*=MAX_BRIGHTNESS;
						n_brightness+=(int)f_temp;
						if(n_brightness>MAX_BRIGHTNESS)
								n_brightness=MAX_BRIGHTNESS;
				}
				
				if(i %10 == 0){
					
					if(aun_ir_buffer[i] < 50000 && ok){
						need_calibration = true;
						//LCD1602_print("Put your finger!");
						i = last_i;
					} else {
						if(need_calibration){
							calibrate();
							need_calibration = false;
						}
						/*LCD1602_print("   HR: ");
						LCD1602_PrintInt(n_heart_rate);
						LCD1602_2ndLine();
						LCD1602_print("   Spo2: ");
						LCD1602_PrintInt(n_sp02);*/
						lati=38742461;
						logi=35478911;
						alti=105412;
						char c = n_heart_rate%10+'0';
						trans[3]=c;
						c = ((n_heart_rate-(n_heart_rate%10))/10)%10+'0';
						trans[2]=c;
						c = n_sp02%10+'0';
						trans[9]=c;
						c = ((n_sp02-(n_sp02%10))/10)%10+'0';
						trans[8]=c;
						
						c=lati%10+'0';
						trans[20]=c;
						c = ((lati-(lati%10))/10)%10+'0';
						trans[19]=c;
						c = ((lati-(lati%100))/100)%10+'0';
						trans[18]=c;
						c = ((lati-(lati%1000))/1000)%10+'0';
						trans[17]=c;
						c = ((lati-(lati%10000))/10000)%10+'0';
						trans[16]=c;
						c = ((lati-(lati%100000))/100000)%10+'0';
						trans[15]=c;
						c = ((lati-(lati%1000000))/1000000)%10+'0';
						trans[13]=c;
						c = ((lati-(lati%10000000))/10000000)%10+'0';
						trans[12]=c;
						
						c=logi%10+'0';
						trans[32]=c;
						c = ((logi-(logi%10))/10)%10+'0';
						trans[31]=c;
						c = ((logi-(logi%100))/100)%10+'0';
						trans[30]=c;
						c = ((logi-(logi%1000))/1000)%10+'0';
						trans[29]=c;
						c = ((logi-(logi%10000))/10000)%10+'0';
						trans[28]=c;
						c = ((logi-(logi%100000))/1000000)%10+'0';
						trans[27]=c;
						c = ((logi-(logi%1000000))/1000000)%10+'0';
						trans[25]=c;
						c = ((logi-(logi%10000000))/10000000)%10+'0';
						trans[24]=c;
						
						c=alti%10+'0';
						trans[42]=c;
						c = ((alti-(alti%10))/10)%10+'0';
						trans[41]=c;
						c = ((alti-(alti%100))/100)%10+'0';
						trans[39]=c;
						c = ((alti-(alti%1000))/1000)%10+'0';
						trans[38]=c;
						c = ((alti-(alti%10000))/10000)%10+'0';
						trans[37]=c;
						c = ((alti-(alti%100000))/100000)%10+'0';
						trans[36]=c;
						//string2hexString((char*)&trans,hex_str);
						//uint8_t sendd[118]="7E003700010013A200419A292A00483D30302C20533D30302C2030302E3030303030304E2C2030302E303030303030452C20303030302E30304D";
						//sendd[116]='8';
						//sendd[117]='C';
						/*for(int j=0;j<87;j++){
						sendd[j+28]=hex_str[j];
						}*/
						HAL_UART_Transmit_IT(&huart1,trans,sizeof(trans));
					}
				}
				
		}
				
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 

		//send samples and calculation result to terminal program through UART

    last_i += 100;
		
	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
