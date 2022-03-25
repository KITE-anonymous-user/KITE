/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "stdarg.h"
#include "string.h"
#include "math.h"
#include "usbd_cdc_if.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define j2h(x) (3.1415926*(x)/180.0)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint32_t clk_cnt0,clk_cnt1;
extern uint8_t data_cnt0,data_cnt1;
extern uint16_t data_poll0[256],data_poll1[256],data_poll00[256],data_poll11[256];//1路0、1双波形
extern uint32_t freq0,freq1;

extern uint8_t select0,select1;//1路波形选择，0-data_poll0[256]，1-data_poll00[256]

extern void Freq0_Init(uint32_t freq);
extern void Freq1_Init(uint32_t freq);

uint16_t buffer_SIZE;

//BMI160
uint32_t BMI_lasttime;
extern __IO  uint32_t uwTick;

#define  id_addr 0x00
#define  accxl_addr 0x12
#define  accxh_addr 0x13
#define  accyl_addr 0x14
#define  accyh_addr 0x15
#define  acczl_addr 0x16
#define  acczh_addr 0x17
#define  gryxl_addr 0x0c

uint8_t addr,status=0;

uint8_t BMI_buff_0[12],BMI_buff_1[12];

uint8_t accxl,accxh,accyl,accyh,acczl,acczh;


extern uint32_t freqT;
extern void BMI160_Read_ID_0(void);
extern void BMI160_Read_ID_1(void);
extern void BMI160_Config_0(void);
extern void BMI160_Config_1(void);
extern void BMI160_Getdata_0(void);
extern void BMI160_Getdata_1(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6,htim7;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t rx_len=0;
volatile uint8_t recv_end_flag=0;
uint8_t rx_buffer[1024];
uint8_t tx_buffer[1024];

void USART1_IRQHandler(void)
{
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
		temp = huart1.Instance->SR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
		temp = huart1.Instance->DR; //读取数据寄存器中的数据
		HAL_UART_DMAStop(&huart1); //
		temp  = hdma_usart1_rx.Instance->NDTR;// 获取DMA中未传输的数据个数，NDTR寄存器分析见下面
		rx_len =  1024 - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		recv_end_flag = 1;	// 接受完成标志位置1	
	 }
  HAL_UART_IRQHandler(&huart1);

}

void info_printf(char* fmt,...)  
{  	
	__va_list ap;
	
	va_start(ap,fmt);
	vsprintf((char*)tx_buffer,fmt,ap);
	va_end(ap);
	
	if(strlen((char*)tx_buffer)!=0)
	{
    HAL_UART_Transmit(&huart1,tx_buffer,strlen((char*)tx_buffer),200);
	}
}

void CMD_Process(void)
{
	if(recv_end_flag ==1)
	{
		rx_buffer[rx_len]=0;
		info_printf("rev:%d:%s\r\n",rx_len,rx_buffer);
		recv_end_flag=0;
		HAL_UART_Receive_DMA(&huart1,rx_buffer,1024);

		switch(rx_buffer[2])
		{
			case '?' :
			{
				info_printf("----------------------\r\n");
				info_printf("?--help\r\n");
				info_printf("q|Q--reset\r\n");
				info_printf("----------------------\r\n");
				info_printf("a--gpiod 0xff\r\n");
				info_printf("b--gpiod 0x00\r\n");
				info_printf("c--gpioe 0xff\r\n");
				info_printf("d--gpioe 0x00\r\n");
				info_printf("e--BMI ID\r\n");
				info_printf("f--BMI config\r\n");
				info_printf("e--BMI getdata\r\n");
				info_printf("----------------------\r\n");
				info_printf("A--ch0 sin\r\n");
				info_printf("B--ch0 square\r\n");
				info_printf("C--ch1 sin\r\n");
				info_printf("D--ch1 square\r\n");
				info_printf("E--enable ch0\r\n");
				info_printf("F--enable ch1\r\n");
				info_printf("H--disable waves\r\n");
				info_printf("S--*.S.ch1_freq.ch2_freq.bufferSize\r\n");
				info_printf("T--*.T.freqT(MAX200hz)\r\n");
				info_printf("U--*.U.select0\r\n");
				info_printf("V--*.V.select1\r\n");
				info_printf("X--*.X.select0/select00.data(raw)\r\n");
				info_printf("Y--*.Y.select1/select11.data(raw)\r\n");
				info_printf("----------------------\r\n");
				break;
			}
			/*******************************************************************/
			case 'a' :
			{
				GPIOD->ODR=0x00000fff;
				HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_SET);
				break;
			}
			case 'b' :
			{
				GPIOD->ODR=0x00000000;
				HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_SET);
				break;
			}
			case 'c' :
			{
				GPIOE->ODR=0x00000fff;
				HAL_GPIO_WritePin(GPIOA,CLK1_Pin, GPIO_PIN_SET);
				break;
			}
			case 'd' :
			{
				GPIOE->ODR=0x00000000;
				HAL_GPIO_WritePin(GPIOA,CLK1_Pin, GPIO_PIN_SET);
				break;
			}
			case 'e' :
			{
				BMI160_Read_ID_0();
				BMI160_Read_ID_1();
				break;
			}
			case 'f' :
			{
				BMI160_Config_0();
				BMI160_Config_1();
				break;
			}
			case 'g' :
			{
				BMI160_Getdata_0();
				info_printf("BMI_buff_0=%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n",
				BMI_buff_0[0],BMI_buff_0[1],BMI_buff_0[2],BMI_buff_0[3],BMI_buff_0[4],BMI_buff_0[5],
				BMI_buff_0[6],BMI_buff_0[7],BMI_buff_0[8],BMI_buff_0[9],BMI_buff_0[10],BMI_buff_0[11]
				);
				BMI160_Getdata_1();
				info_printf("BMI_buff_1=%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n",
				BMI_buff_1[0],BMI_buff_1[1],BMI_buff_1[2],BMI_buff_1[3],BMI_buff_1[4],BMI_buff_1[5],
				BMI_buff_1[6],BMI_buff_1[7],BMI_buff_1[8],BMI_buff_1[9],BMI_buff_1[10],BMI_buff_1[11]
				);
				break;
			}
					
			/*******************************************************************/
			case 'H' ://停止输出波形
			{
				HAL_TIM_Base_Stop_IT(&htim6);
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}
			case 'A' ://填充正弦波到0
			{
				uint16_t i;
//				for(i=0;i<256;i++)
//				{
//					data_poll0[i]=sin(j2h(360.0/256.0*i))*0x800+0x800;
//				}
				for(i=0;i<buffer_SIZE;i++)
				{
					data_poll0[i]=sin(j2h(360.0/buffer_SIZE*i))*0x800+0x800;
				}
				Freq0_Init(freq0);
				HAL_TIM_Base_Start_IT(&htim6);
				
				
				break;
			}
			case 'B' ://填充方波到0
			{
				uint16_t i;
//				for(i=0;i<128;i++)
//				{
//					data_poll0[i]=0xfff;
//				}
//				for(i=128;i<256;i++)
//				{
//					data_poll0[i]=0x000;
//				}
				for(i=0;i<buffer_SIZE/2;i++)
				{
					data_poll0[i]=0xfff;
				}
				for(i=buffer_SIZE/2;i<buffer_SIZE;i++)
				{
					data_poll0[i]=0x000;
				}
				Freq0_Init(freq0);
				HAL_TIM_Base_Start_IT(&htim6);
				break;
			}
			case 'C' ://填充正弦波到1
			{
				uint16_t i;
//				for(i=0;i<256;i++)
//				{
//					data_poll1[i]=sin(j2h(360.0/256.0*i))*0x800+0x800;
//				}
				for(i=0;i<buffer_SIZE;i++)
				{
					data_poll1[i]=sin(j2h(360.0/buffer_SIZE*i))*0x800+0x800;
				}
				Freq1_Init(freq1);
				HAL_TIM_Base_Start_IT(&htim7);
				break;
			}
			case 'D' ://填充方波到1
			{
				uint16_t i;
//				for(i=0;i<128;i++)
//				{
//					data_poll1[i]=0xfff;
//				}
//				for(i=128;i<256;i++)
//				{
//					data_poll1[i]=0x000;
//				}
				for(i=0;i<buffer_SIZE/2;i++)
				{
					data_poll1[i]=0xfff;
				}
				for(i=buffer_SIZE/2;i<buffer_SIZE;i++)
				{
					data_poll1[i]=0x000;
				}
				Freq1_Init(freq1);
				HAL_TIM_Base_Start_IT(&htim7);
				break;
			}
			case 'E' ://使能ch0
			{
				Freq0_Init(freq0);
				HAL_TIM_Base_Start_IT(&htim6);
				break;
			}
			case 'F' ://使能ch1
			{
				Freq1_Init(freq1);
				HAL_TIM_Base_Start_IT(&htim7);
				break;
			}
			case 'S' ://设置波形频率
			{
				sscanf((char*)rx_buffer,"*.S.%d.%d.%d",&freq0,&freq1,&buffer_SIZE);
				if(((buffer_SIZE*freq0)>=896000)||((buffer_SIZE*freq0)>=896000))
				{
					freq0=1000;freq1=1000;buffer_SIZE=256;
				}
				info_printf("\r\nfreq0=%d,freq1=%d,buffer_SIZE=%d\r\n",freq0,freq1,buffer_SIZE);
				break;
			}
			case 'T' ://设置采样ITU频率
			{
				sscanf((char*)rx_buffer,"*.T.%d",&freqT);
				info_printf("\r\nfreqT=%d\r\n",freqT);
				break;
			}
			case 'U' ://设置0路输出波形
			{
				sscanf((char*)rx_buffer,"*.U.%d",&select0);
				info_printf("\r\nselect0=%d\r\n",select0);
				break;
			}
			case 'V' ://设置1路输出波形
			{
				sscanf((char*)rx_buffer,"*.U.%d",&select1);
				info_printf("\r\nselect1=%d\r\n",select1);
				break;
			}
			case 'X' ://设置0路波形数据
			{
				//"X--*.X.select0/select00.data(raw)\r\n"
				uint8_t i;
				
				if(rx_buffer[4]==0)
				{
					for(i=0;i<buffer_SIZE;i++)
					{
						data_poll0[i]=rx_buffer[6+i*2]+rx_buffer[6+i*2+1]*256;
					}
					info_printf("data_poll0=");
					for(i=0;i<buffer_SIZE;i++)
					{
						info_printf("[%d]=%d\r\n",i,data_poll0[i]);
					}
				}
				else if(rx_buffer[4]==1)
				{
					for(i=0;i<buffer_SIZE;i++)
					{
						data_poll00[i]=rx_buffer[6+i*2]+rx_buffer[6+i*2+1]*256;
					}
					info_printf("data_poll00=");
					for(i=0;i<buffer_SIZE;i++)
					{
						info_printf("[%d]=%d\r\n",i,data_poll00[i]);
					}
				}
				break;
			}
			case 'Y' ://设置1路波形数据
			{
				//"Y--*.Y.select1/select11.data(raw)\r\n"
				uint8_t i;
				
				if(rx_buffer[4]==0)
				{
					for(i=0;i<buffer_SIZE;i++)
					{
						data_poll1[i]=rx_buffer[6+i*2]+rx_buffer[6+i*2+1]*256;
					}
					info_printf("data_poll1=");
					for(i=0;i<buffer_SIZE;i++)
					{
						info_printf("[%d]=%d\r\n",i,data_poll1[i]);
					}
				}
				else if(rx_buffer[4]==1)
				{
					for(i=0;i<buffer_SIZE;i++)
					{
						data_poll11[i]=rx_buffer[6+i*2]+rx_buffer[6+i*2+1]*256;
					}
					info_printf("data_poll11=");
					for(i=0;i<buffer_SIZE;i++)
					{
						info_printf("[%d]=%d\r\n",i,data_poll11[i]);
					}
				}
				break;
			}
			/*******************************************************************/
			case 'q' :
			{
				if((rx_buffer[2]=='Q')&&(rx_buffer[3]=='q'))
					//Board_Reset();
				break;
			}
			default  :
			{
				break;
      }
		}
	}
}


/*****************************************************
BMI160
*****************************************************/

uint32_t freqT=10;

void BMI160_Read_ID_0(void)
{
	addr=id_addr|0x80;//读地址
	
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Receive(&hspi2, &status, 1, 100);//读取
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	info_printf("id0=%02x\r\n",status);
}
void BMI160_Read_ID_1(void)
{
	addr=id_addr|0x80;//读地址
	
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS
	HAL_Delay(20);	
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Receive(&hspi2, &status, 1, 100);//读取
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	info_printf("id1=%02x\r\n",status);
}

void BMI160_Config_0(void)//配置 ODR resolution BW
{
	uint8_t temp;
	
	temp=0x26;
	//SPI_master_write_register(0x40, 1, &temp);		//ACC	ODR:25Hz		acc_bwp=3db(defult:acc_us=0b0)	
	addr=0x40|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS
	HAL_Delay(20);	
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x0C;
	//SPI_master_write_register(0x41, 1, &temp);		//Acc_range:16g
	addr=0x41|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x26;
	//SPI_master_write_register(0x42, 1, &temp);		//Gro	ODR:25Hz		gro_bwp=3db	
	addr=0x42|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x03;
	//SPI_master_write_register(0x43, 1, &temp);		//Gro_range:250dps
	addr=0x43|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS
	HAL_Delay(20);	
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	//FIFO  Config
	temp=0xfe;
	//SPI_master_write_register(0x47, 1, &temp);		//enable
	addr=0x47|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	//Set PMU mode	Register(0x7E) CMD		attention the command
	temp=0x11;
	//SPI_master_write_register(0x7E, 1, &temp);		//Acc normal mode
	addr=0x7E|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x15;
	//SPI_master_write_register(0x7E, 1, &temp);		//Gro normal mode
	addr=0x7E|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	//check the PMU_status	Register(0x03) 
	//SPI_master_read_register(0x03,1,&ch);
	addr=0x03|0x80;//读地址
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Receive(&hspi2, &status, 1, 100);//读取
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	if (status == 0x14)
	{
		info_printf("sensor0 is Normal \r\n");
	}
}

void BMI160_Config_1(void)//配置 ODR resolution BW
{
	uint8_t temp;
	
	temp=0x26;
	//SPI_master_write_register(0x40, 1, &temp);		//ACC	ODR:25Hz		acc_bwp=3db(defult:acc_us=0b0)	
	addr=0x40|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x0C;
	//SPI_master_write_register(0x41, 1, &temp);		//Acc_range:16g
	addr=0x41|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x26;
	//SPI_master_write_register(0x42, 1, &temp);		//Gro	ODR:25Hz		gro_bwp=3db	
	addr=0x42|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x03;
	//SPI_master_write_register(0x43, 1, &temp);		//Gro_range:250dps
	addr=0x43|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	//FIFO  Config
	temp=0xfe;
	//SPI_master_write_register(0x47, 1, &temp);		//enable
	addr=0x47|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS
	HAL_Delay(20);	
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	//Set PMU mode	Register(0x7E) CMD		attention the command
	temp=0x11;
	//SPI_master_write_register(0x7E, 1, &temp);		//Acc normal mode
	addr=0x7E|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	temp=0x15;
	//SPI_master_write_register(0x7E, 1, &temp);		//Gro normal mode
	addr=0x7E|0x00;//写地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);//发送
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	//check the PMU_status	Register(0x03) 
	//SPI_master_read_register(0x03,1,&ch);
	addr=0x03|0x80;//读地址
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Receive(&hspi2, &status, 1, 100);//读取
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	HAL_Delay(20);
	
	if (status == 0x14)
	{
		info_printf("sensor1 is Normal \r\n");
	}
}

void BMI160_Getdata_0(void)
{
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(1);
	addr=accxl_addr|0x80;
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Receive(&hspi2, BMI_buff_0, 12, 100);//读取
	HAL_GPIO_WritePin(GPIOB, CS0_Pin, GPIO_PIN_SET);//失能CS
	//HAL_Delay(1);
}
void BMI160_Getdata_1(void)
{
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_RESET);//使能CS	
	HAL_Delay(1);
	addr=accxl_addr|0x80;
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);//发送
	HAL_SPI_Receive(&hspi2, BMI_buff_1, 12, 100);//读取
	HAL_GPIO_WritePin(GPIOB, CS1_Pin, GPIO_PIN_SET);//失能CS
	//HAL_Delay(1);
}

uint32_t BMI_count;

void BMI_Process(void)
{
	if((uwTick-BMI_lasttime)>=(1000/freqT))
	{
		uint8_t buff[32];
		BMI_lasttime=uwTick;
		BMI160_Getdata_0();
		BMI160_Getdata_1();
		memcpy(buff,BMI_buff_0,12);
		memcpy(buff+12,BMI_buff_1,12);
		
		CDC_Transmit_FS(buff,24);
		
		//4test
		//BMI_count++;
		//if((BMI_count%100)==0) info_printf("BMI_100 loop uwTick=%d",uwTick);
	}
}


/*******************************************/

/*******************************************/


uint32_t clk_cnt0,clk_cnt1;

uint8_t data_cnt0,data_cnt1;
uint16_t data_poll0[256],data_poll1[256],data_poll00[256],data_poll11[256];//1路0、1双波形

uint32_t freq0=1000,freq1=1000;

uint8_t select0,select1;//1路波形选择，0-data_poll0[256]，1-data_poll00[256]

void Freq0_Init(uint32_t freq)
{
	
	//分频计算
	uint32_t cnt,prescaler,period;
	
	//cnt=84000000.0/3.0/(freq*256.0);
	cnt=84000000.0/1.0/(freq*buffer_SIZE);
	if(1)//(cnt>=0x10000)
	{
		period=prescaler=sqrt(cnt);
	}
	
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = prescaler-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = period-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}

void Freq1_Init(uint32_t freq)
{
	
	//分频计算
	uint32_t cnt,prescaler,period;
	
	//cnt=84000000.0/3.0/(freq*256.0);
	cnt=84000000.0/1.0/(freq*buffer_SIZE);
	if(1)//(cnt>=0x10000)
	{
		period=prescaler=sqrt(cnt);
	}
	
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = prescaler-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = period-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
	  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
//		clk_cnt0++;
//		if((clk_cnt0%3)==0)
//		{
//			data_cnt0++;
//			GPIOD->ODR=data_poll0[data_cnt0];
//		}
//		else if((clk_cnt0%3)==1)
//		{
//			HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_SET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_RESET);
//		}
		HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_RESET);
		if(select0==0) GPIOD->ODR=data_poll0[data_cnt0];
		else if(select0==1) GPIOD->ODR=data_poll00[data_cnt0];
		HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_SET);
		data_cnt0++;
		if(data_cnt0>=buffer_SIZE) 
		{
			data_cnt0=0;
		}
		
	}
	if(htim == &htim7)
	{
	  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
//		clk_cnt1++;
//		if((clk_cnt1%3)==0)
//		{
//			data_cnt1++;
//			GPIOE->ODR=data_poll1[data_cnt1];
//		}
//		else if((clk_cnt1%3)==1)
//		{
//			HAL_GPIO_WritePin(GPIOA,CLK1_Pin, GPIO_PIN_SET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOA,CLK1_Pin, GPIO_PIN_RESET);
//		}
		HAL_GPIO_WritePin(GPIOA,CLK1_Pin, GPIO_PIN_RESET);
		if(select1==1) GPIOE->ODR=data_poll1[data_cnt1];
		else if(select1==1) GPIOE->ODR=data_poll11[data_cnt1];
		HAL_GPIO_WritePin(GPIOA,CLK1_Pin, GPIO_PIN_SET);
		data_cnt1++;
		if(data_cnt1>=buffer_SIZE) 
		{
			data_cnt1=1;
		}
	}
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	uint64_t gl_cnt=0;
	uint32_t gl_lasttime=0;
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
	MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	info_printf("ZD DAC Demo\r\n");
	
	//BMI160
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS0_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	BMI160_Read_ID_0();
	BMI160_Read_ID_1();
	HAL_Delay(200);
	BMI160_Config_0();
	BMI160_Config_1();
	BMI160_Config_0();
	BMI160_Config_1();
	
	//TIM
	//HAL_TIM_Base_Start_IT(&htim6);//HAL_TIM_Base_Stop_IT(&htim6);
	//HAL_TIM_Base_Start_IT(&htim7);//HAL_TIM_Base_Stop_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	gl_lasttime=uwTick;
  while (1)
  {
    /* USER CODE END WHILE */
		CMD_Process();
		BMI_Process();
		
//		{
//			HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_RESET);
//			data_cnt0++;
//			GPIOD->ODR=data_poll0[data_cnt0];
//			HAL_GPIO_WritePin(GPIOA,CLK2_Pin, GPIO_PIN_SET);
//			gl_cnt++;
//			if((uwTick-gl_lasttime)>=5000)
//			{
//				gl_lasttime=uwTick;
//				info_printf("5s gl_cnt=%d\r\n",gl_cnt);
//				gl_cnt=0;
//			}
//		}
    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	/* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}
/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 6;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能idle中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,256);
  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PD1_Pin|CLK1_Pin|PD2_Pin|CLK2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE8 PE9 
                           PE10 PE11 PE12 PE13 
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1_Pin PD2_Pin */
  GPIO_InitStruct.Pin = PD1_Pin|PD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pins : PA1*/
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK1_Pin CLK2_Pin */
  GPIO_InitStruct.Pin = CLK1_Pin|CLK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_Pin */
  GPIO_InitStruct.Pin = INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS0_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
