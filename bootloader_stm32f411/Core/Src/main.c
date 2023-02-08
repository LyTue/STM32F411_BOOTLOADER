/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

// enable this line to get debug message over debug uart
#define BL_DEBUG_MSG_EN



/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

uint8_t supported_commands[] = {BL_GET_VER,
                                BL_GET_HELP, 
                                BL_GET_CID, 
                                BL_GET_RDP_STATUS,
                                BL_GO_TO_ADDR,
                                BL_FLASH_ERASE,
                                BL_MEM_WRITE,
                                BL_READ_SECTOR_STATUS};

#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];

#define D_UART &huart1
#define C_UART &huart2

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

static void printmsg(char *format, ...);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char somedata[] = "Hello from Bootloader\r\n";

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET){
    printmsg("BL_DEBUG_MSG:Button is pressed .... going to BL mode\n\r");
    
    bootloader_uart_read_data();
  }
  else {
    printmsg("BL_DEBUG_MSG:Button is not pressed .. excecuting user \n\r");
    bootloader_jump_to_user_app();
  }
}


void bootloader_uart_read_data(){
  uint8_t rcv_len = 0;
  
  while(1) {
    
    memset(bl_rx_buffer, 0, 200);
    //here we will read and decode the commands coming from host
    // first read only one byte from the host, which is the "lenght" field of the command
    HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
    rcv_len = bl_rx_buffer[0];
    HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
    switch(bl_rx_buffer[1]){
      
      case BL_GET_VER:
        bootloader_handle_getver_cmd(bl_rx_buffer);
        break;
      case BL_GET_HELP:
        bootloader_handle_gethelp_cmd(bl_rx_buffer);
        break;
      case BL_GET_CID:
        bootloader_handle_getcid_cmd(bl_rx_buffer);
        break;  
      case BL_GET_RDP_STATUS:
        bootloader_handle_getrdp_cmd(bl_rx_buffer);
        break;
      case BL_GO_TO_ADDR:
        bootloader_handle_go_cmd(bl_rx_buffer);
        break;
      case BL_FLASH_ERASE:
        bootloader_handle_flash_erase_cmd(bl_rx_buffer);
        break;
      case BL_MEM_WRITE:
        bootloader_handle_mem_write_cmd(bl_rx_buffer);
        break;
      case BL_EN_R_W_PROTECT:
        bootloader_handle_endis_rw_protect(bl_rx_buffer);
        break;
      case BL_MEM_READ:
        bootloader_handle_mem_read(bl_rx_buffer);
        break;
      case BL_READ_SECTOR_STATUS:
        bootloader_handle_read_sector_status(bl_rx_buffer);
        break;
      case BL_OTP_READ:
        bootloader_handle_getver_cmd(bl_rx_buffer);
        break;
      default:
        printmsg("BL_DEBUG_MSG:Invalid command code received from host\n\r");
        break;
    }
    
  }
}

void bootloader_jump_to_user_app(){
  
  void (*app_reset_handler)(void);
  
  printmsg("BL_DEBUG_MSG: bootloader_jumb_to_user_app\n\r");
  
  //1. configure the MSP by reading the value from base address of the sector 2
  uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR_2; // sector of user application
  
  //this function comes from CMSIS
  __set_MSP(msp_value);
  
  //SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS
  
  /*2. Now fetch the reset handler address of the user application
   *from the location FLASH SECTOR 2 BASE ADDRESS + 4
   */
  
  uint32_t resethandler_address = *(uint32_t *)(0X00000000 + 4);
  app_reset_handler = (void *)resethandler_address;
  
  printmsg("BL_DEBUG_MSG: app_reset_handler addr : %#x\n\r", app_reset_handler);
  
  //3. jump to reset handler of the user application
  app_reset_handler();
  
}

 
/*
* print message 
*/
void printmsg(char * format, ...){

#ifdef BL_DEBUG_MSG_EN
  
  char str[80];
  
  /*extract the agrument list using VA apis */
  va_list args;
  va_start(args, format);
  vsprintf(str, format, args);
  HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  va_end(args);
  
#endif
  
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC1 PC2 PC4
                           PC5 PC6 PC8 PC9
                           PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB13
                           PB14 PB15 PB4 PB5
                           PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD0 PD1 PD2 PD3
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(Audio_SDA_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



/********************bootloader implement****************/
void bootloader_handle_getver_cmd(uint8_t * bl_rx_buffer){
  uint8_t bl_version;
  
  // 1) verify the checksum
  printmsg("BL_DEBUG_MSG: bootloader_handle_getver cmd\n");
  if( ! bootloader_verify_crc(&bl_rx_buffer[0], bl_rx_buffer[0] + 1, 0)){
    printmsg("BL_DEBUG_MSG:checksum success !!\n");
    //check sum is correct..
    bootloader_send_ack(bl_rx_buffer[0], 1);
    bl_version = get_bootloader_version();
    printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n", bl_version, bl_version);
    bootloader_uart_write_data(&bl_version, 1);
  }
  else {
    printmsg("BL_DEBUG_MSG:checksum fail !! \n");
    bootloader_send_nack();
  }
}

void bootloader_handle_gethelp_cmd(uint8_t * pBuffer){
  
  //total length of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  // extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer+command_packet_len - 4));
                         
  if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
  {
    bootloader_send_ack(pBuffer[0], sizeof(supported_commands));
    bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
  }
  else {
    bootloader_send_nack();
  }
}

void bootloader_handle_getcid_cmd(uint8_t * pBuffer){
  
  uint16_t bl_cid_num = 0;
  
  //total lenght of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
  
  if(! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
  {
  
    bootloader_send_ack(pBuffer[0], 2);
    bl_cid_num = get_mcu_chip_id();
    bootloader_uart_write_data((uint8_t *)&bl_cid_num, 2);
  }
  else {
    bootloader_send_nack();
  }
}

void bootloader_handle_getrdp_cmd(uint8_t * pBuffer){
  uint8_t rdp_level = 0x00;
  
  //total lenght of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
  
  if( ! bootloader_verify_crc(&bl_rx_buffer[0], bl_rx_buffer[0] + 1, 0)){
    printmsg("BL_DEBUG_MSG:checksum success !!\n");
    //check sum is correct..
    bootloader_send_ack(bl_rx_buffer[0], 1);
    rdp_level = get_flash_rdp_level();
    printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n", rdp_level, rdp_level);
    bootloader_uart_write_data(&rdp_level, 1);
  }
  else {
    printmsg("BL_DEBUG_MSG:checksum fail !! \n");
    bootloader_send_nack();
  }  
}

void bootloader_handle_go_cmd(uint8_t * pBuffer){
  uint32_t go_address = 0;
  uint8_t addr_valid = ADDR_VALID;
  uint8_t addr_invalid = ADDR_INVALID;
  
  //total lenght of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
  
  if( ! bootloader_verify_crc(&bl_rx_buffer[0], bl_rx_buffer[0] + 1, 0)){
    printmsg("BL_DEBUG_MSG:checksum success !!\n");
    //check sum is correct..
    bootloader_send_ack(pBuffer[0], 1);
    go_address = *((uint32_t*)&pBuffer[2]);
    printmsg("BL_DEBUG_MSG:GO addr : %#x\n", go_address);
    
    if(verify_address(go_address) == ADDR_VALID){
      bootloader_uart_write_data(&addr_valid, 1);
      go_address+=1;
      void (*lets_jump)(void) = (void*)go_address;
      printmsg("BL_DEBUG_MSG: jumping to go address! \n");
      lets_jump();
    }
    else {
      bootloader_uart_write_data(&addr_invalid, 1);
    }
    
  }
  else {
    printmsg("BL_DEBUG_MSG:checksum fail !! \n");
    bootloader_send_nack();
  }  
  
}

void bootloader_handle_flash_erase_cmd(uint8_t * pBuffer){
  uint8_t erase_status = 0x00;
  
  //total lenght of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
  
  if( ! bootloader_verify_crc(&bl_rx_buffer[0], bl_rx_buffer[0] + 1, 0)){
    printmsg("BL_DEBUG_MSG:checksum success !!\n");
    //check sum is correct..
    bootloader_send_ack(pBuffer[0], 1);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
    erase_status = execute_flash_erase(pBuffer[2], pBuffer[3]);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
    bootloader_uart_write_data(&erase_status, 1);
  }
  else {
    printmsg("BL_DEBUG_MSG:checksum fail !! \n");
    bootloader_send_nack();
  }  
  
}

void bootloader_handle_mem_write_cmd(uint8_t * pBuffer){

  uint8_t addr_valid = ADDR_VALID;
  uint8_t write_status = 0X00;
  uint8_t chksum = 0, len = 0;
  len = pBuffer[0];
  uint8_t payload_len = pBuffer[6];
  uint32_t mem_address = *((uint32_t*)(&pBuffer[2]));
  
  chksum = pBuffer[len];
  
  printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");
  
  //total lenght of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
  
  if( ! bootloader_verify_crc(&bl_rx_buffer[0], bl_rx_buffer[0] + 1, 0)){
    
    bootloader_send_ack(pBuffer[0], 1);
    if(verify_address(mem_address) == ADDR_VALID){
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      write_status = execute_mem_write(&pBuffer[7], mem_address, payload_len);
      
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      bootloader_uart_write_data(&write_status, 1);
    }
    else {
      write_status = ADDR_INVALID;
      bootloader_uart_write_data(&write_status, 1);
      
    }
  }
  else {
    printmsg("BL_DEBUG_MSG:checksum fail !! \n");
    bootloader_send_nack();
  }  
  
}

void bootloader_handle_endis_rw_protect(uint8_t * pBuffer){
  
  uint8_t status = 0X00;
  
  //total lenght of the command packet
  uint32_t command_packet_len = bl_rx_buffer[0] + 1;
  
  //extract the CRC32 sent by the Host
  uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
  
  if( ! bootloader_verify_crc(&bl_rx_buffer[0], bl_rx_buffer[0] + 1, 0)){
    printmsg("BL_DEBUG_MSG:checksum success !!\n");
    //check sum is correct..
    bootloader_send_ack(pBuffer[0], 1);
    status = configure_flash_sector_rw_protection(pBuffer[2], pBuffer[3], 0);
    bootloader_uart_write_data(&status, 1);
  }
  else {
    printmsg("BL_DEBUG_MSG:checksum fail !! \n");
    bootloader_send_nack();
  }
  
}
void bootloader_handle_mem_read(uint8_t * bl_rx_buffer){}
void bootloader_handle_read_sector_status(uint8_t * bl_rx_buffer){}



/**support cmd*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len){
  uint8_t ack_buf[2];
  ack_buf[0] = BL_ACK;
  ack_buf[1] = follow_len;
  HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void){
  uint8_t nack = BL_NACK;
  HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host){
  uint32_t uwCRCValue = 0XFF;
  for (uint32_t i = 0; i < len; i++){
    uint32_t i_data = pData[i];
    uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
  }
  
  if(uwCRCValue == crc_host){
    return VERIFY_CRC_SUCCESS;
  }
  
  return VERIFY_CRC_FAIL;
}

void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len){
  HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}

uint8_t get_bootloader_version(void){
  return (uint8_t)BL_VERSION;
}

uint16_t get_mcu_chip_id(void){
  uint16_t cid;
  cid = (uint16_t)(DBGMCU->IDCODE) & 0X0FFF;
  return cid;
}

uint8_t get_flash_rdp_level(void){
  uint8_t rdp_status = 0;
#if 0
  FLASH_OBProgramInitTypeDef ob_handle;
  HAL_FLASHEx_OBGetConfig(&ob_handle);
  rdp_status = (uint8_t)ob_handle.RDPLevel;
#else 
  volatile uint32_t *pOB_addr = (uint32_t*)0x1FFFC000;
  rdp_status = (uint8_t)(*pOB_addr >> 8);
#endif
  
  return rdp_status;
}

uint8_t verify_address(uint32_t go_address){
  if(go_address >= SRAM1_BASE && go_address <= SRAM1_BB_BASE){
    return ADDR_VALID;
  }
  else if (go_address >= FLASH_BASE && go_address <= FLASH_END){
    return ADDR_VALID;
  }
  else if (go_address <= BKPSRAM_BB_BASE){
    return ADDR_VALID;
  }
  else 
    return ADDR_INVALID;
  
  
}

uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector){
  FLASH_EraseInitTypeDef flashErase_handle;
  uint32_t sectorError;
  HAL_StatusTypeDef status;
  
  if(number_of_sector > 8) return INVALID_SECTOR;
  if((sector_number == 0xFF) || (sector_number <= 7)){
    if(sector_number == (uint8_t)0xff){
      flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
    }
    
    else {
      uint8_t remanining_sector = 8 - sector_number;
      if(number_of_sector > remanining_sector){
        number_of_sector = remanining_sector;
      }
      flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
      flashErase_handle.Sector = sector_number;
      flashErase_handle.NbSectors = number_of_sector;
    }
    flashErase_handle.Banks = FLASH_BANK_1;
    
    HAL_FLASH_Unlock();
    flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    status = (uint8_t)HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
    HAL_FLASH_Lock();
    
    return status;
  }
  
  return INVALID_SECTOR;
}

uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len){
  uint8_t status = HAL_OK;
  
  HAL_FLASH_Unlock();
  
  for(uint32_t i = 0; i < len; i++){
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_address + i, pBuffer[i]);
  }
  
  HAL_FLASH_Lock();
  
  return status;
}

uint32_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable){
  volatile uint32_t *pOPTCR = (uint32_t*)0X40023C14;
  
  if(disable){
    HAL_FLASH_OB_Unlock();
    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
    
    *pOPTCR &= ~(1 << 31);
    
    *pOPTCR |= (0XFF << 16);
    
    *pOPTCR |= (1 << 1);
    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
    
    HAL_FLASH_OB_Lock();
    
    return 0;
  }
  
  if(protection_mode == (uint8_t)1) {
    HAL_FLASH_OB_Unlock();
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
    
    *pOPTCR &= ~(1 << 31);
    
    *pOPTCR &= ~(sector_details << 16);
    
    *pOPTCR |= (1 << 1);
    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
    
    HAL_FLASH_OB_Lock();   
  }
  
  else if (protection_mode == (uint8_t)2){
    
    HAL_FLASH_OB_Unlock();
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
    
    *pOPTCR &= ~(1 << 31);
    
    *pOPTCR &= ~(0xff << 16);
    *pOPTCR |= (sector_details << 16);
    
    *pOPTCR |= (1 << 1);
    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
    
    HAL_FLASH_OB_Lock();   
    
  }
  
  
  return *pOPTCR;
  
}




