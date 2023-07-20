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
#include "adc.h"
#include "crc.h"
#include "dcmi.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include <math.h>
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_camera.h"

#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include "network_1.h"
#include "network_1_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static void AI_Init(void);
static void AI_Run(int *pIn, int *pOut);
static void AI_Init_1(void);
static void AI_Run_1(float *pIn, float *pOut);

static void prepare_yolo_data(uint16_t* image_data);
static void post_process_my(void);

static uint32_t argmax(const float *values, uint32_t len);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define FRAME_BUFFER  0xC0000000
#define SCALE_IMAGE  0xC0200000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t Im_size = 0;
uint32_t Im_size2 = 0;



ai_handle network;
ai_handle network_1;

int in_data[AI_NETWORK_IN_1_SIZE];
int out_data[AI_NETWORK_OUT_1_SIZE];

float in_data_1[AI_NETWORK_1_IN_1_SIZE];
float out_data_1[AI_NETWORK_1_OUT_1_SIZE];

ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
ai_u8 activations_1[AI_NETWORK_1_DATA_ACTIVATIONS_SIZE];

ai_buffer * ai_input;
ai_buffer * ai_output;

ai_buffer * ai_input_1;
ai_buffer * ai_output_1;

//uint16_t temp_data[3136] ;
int x1, y1, x2, y2;                                                  // 人脸方框的左上右下像素坐标
uint8_t anchors[3][2] = {{9, 14}, {12, 17}, {22, 21}};               // yoloface的anchor尺寸
static uint16_t*image_data = (uint16_t*)0xC0200000; 

const char *activities[AI_NETWORK_1_OUT_1_SIZE] =
{
    "anger", "disgust", "fear", "happy", "sad", "surprised", "normal"
};
// '0'anger'#生气 , '1':'disgust'#厌恶 , '2':'fear'#恐惧 , '3':'happy'#开心 , '4':'sad'#伤心 , '5':'surprised'#惊讶 , '6':'normal#正常

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DCMI_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_DMA2D_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */
	BSP_LCD_Init();
//	BSP_LCD_SelectLayer(0);
//  BSP_LCD_DisplayStringAt(50, 0, (uint8_t *)"raw picture", CENTER_MODE);
//	HAL_Delay(1000);
//	HAL_Delay(1000);
//  BSP_LCD_Clear(LCD_COLOR_WHITE);

	BSP_LCD_LayerRgb565Init(0, FRAME_BUFFER);                                                            //layer of camera        layer0
	BSP_LCD_LayerRgb565Init(1, FRAME_BUFFER + (BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));                //layer of other info
	BSP_CAMERA_Init(CAMERA_R320x240);
	HAL_Delay(100);
  Im_size = 0x9600; //size=320x240*2/4	
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS , (uint32_t)FRAME_BUFFER, Im_size);
	
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);                                                              //Initialization Interface
	BSP_LCD_SetTransparency(1, 255);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED); 
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)"TinyML", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, 120, (uint8_t *)"214 Dont't Emo", CENTER_MODE);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	BSP_LCD_SetTransparency(1, 100);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	
	BSP_LCD_SelectLayer(0);                                                                              //Camera Capture Video
	//BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
	BSP_LCD_SetTextColor(LCD_COLOR_RED); 
	BSP_LCD_DisplayStringAt(5, 245, (uint8_t *)"raw video", CENTER_MODE);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	
//  BSP_CAMERA_Init(CAMERA_R480x272);
//	HAL_Delay(100);
//  Im_size = 0x25800; //size=480*240*2/4	
//	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS , (uint32_t)LCD_FRAME_BUFFER, Im_size);

//  HAL_Delay(1000);
//	BSP_LCD_Init();
//	TEST_LCD_LayerRgb565Init(LTDC_ACTIVE_LAYER, SCALE_IMAGE); 
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		prepare_yolo_data(image_data);		
		AI_Init();
    //printf("AI_Run begin :%d\r\n",TIM2->CNT);		
		AI_Run(in_data, out_data);
		//printf("AI_Run end :%d\r\n",TIM2->CNT);		
		post_process_my();
		//printf("post process end :%d\r\n",TIM2->CNT);	
		AI_Init_1();		
		AI_Run_1(in_data_1,out_data_1);		
		printf("AI_Run end :%d\r\n",TIM2->CNT);


		printf("AI_Run_1 out_data_1 : %f, %f, %f, %f, %f, %f, %f\r\n", out_data_1[0], out_data_1[1], out_data_1[2], out_data_1[3], out_data_1[4], out_data_1[5], out_data_1[6] );

		uint32_t class = argmax(out_data_1, AI_NETWORK_1_OUT_1_SIZE);
		printf(": %d - %s\r\n", (int) class, activities[class]);
		printf("run over\r\n");

		BSP_LCD_DisplayStringAt(5, 245, (uint8_t *)"emotion:", LEFT_MODE);
		BSP_LCD_DisplayStringAt(5, 245, (uint8_t *)(activities[class]), RIGHT_MODE);
		HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//========================================================================================WSJ
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

static void AI_Init(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
}

static void AI_Run(int *pIn, int *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}

static void AI_Init_1(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations_1 };
  /* Create an instance of the model */
  err = ai_network_1_create_and_init(&network_1, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_1_create error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
  ai_input_1 = ai_network_1_inputs_get(network_1, NULL);
  ai_output_1 = ai_network_1_outputs_get(network_1, NULL);
}

static void AI_Run_1(float *pIn, float *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* Update IO handlers with the data payload */
  ai_input_1[0].data = AI_HANDLE_PTR(pIn);
  ai_output_1[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_1_run(network_1, ai_input_1, ai_output_1);
  if (batch != 1) {
    err = ai_network_1_get_error(network_1);
    printf("AI ai_network_1_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}






void prepare_yolo_data(uint16_t* image_data)
{
	for(int i = 0; i < 3136; i++)
	{
		uint16_t color = image_data[i];
		// 这里要注意，网络的输入张量维度是BHWC，对应1*56*56*3，通道顺序是RGB
					// 所以输入数组的存储顺序应该是先行后列，颜色是R G B
		in_data[i*3] = (uint8_t)((color&0xF800)>>8) - 128;
		in_data[i*3+1] = (uint8_t)((color&0x07E0)>>3) - 128;
		in_data[i*3+2] = (uint8_t)((color&0x001F)<<3) - 128;
	}
}
	

// 定义sigmoid函数
float sigmoid(float x)
{
	float y = 1/(1+expf(-x));
	return y;
}


#define DEST_WIDTH 64
#define DEST_HEIGHT 64
void resizeFace(int8_t* src, float* dst, int SRC_WIDTH, int SRC_HEIGHT)  //后面的两个参数是上一步裁剪后不确定的长宽，这里是自适应的插值变为64 x 64
	{
    // 计算行和列的缩放比例
    float rowScale = (float)SRC_HEIGHT / DEST_HEIGHT;
    float colScale = (float)SRC_WIDTH / DEST_WIDTH;

    // 遍历目标图像的每一行和每一列
    for (int i = 0; i < DEST_HEIGHT; i++) {
        for (int j = 0; j < DEST_WIDTH; j++) {
            // 计算在原图像中的行和列位置
            float srcRow = i * rowScale;
            float srcCol = j * colScale;

            // 计算左上角像素位置
            int srcRowFloor = (int)floor(srcRow);
            int srcColFloor = (int)floor(srcCol);

            // 计算行和列的偏移量
            float dy = srcRow - srcRowFloor;
            float dx = srcCol - srcColFloor;

            // 计算四个相邻像素在原图像中的线性索引
            int srcIdxTL = srcRowFloor * SRC_WIDTH + srcColFloor;
            int srcIdxTR = srcRowFloor * SRC_WIDTH + (srcColFloor + 1);
            int srcIdxBL = (srcRowFloor + 1) * SRC_WIDTH + srcColFloor;
            int srcIdxBR = (srcRowFloor + 1) * SRC_WIDTH + (srcColFloor + 1);

            // 根据四个相邻像素的插值进行插值计算
            float pixelTL = src[srcIdxTL];
            float pixelTR = src[srcIdxTR];
            float pixelBL = src[srcIdxBL];
            float pixelBR = src[srcIdxBR];

            float interpolatedPixel = (1 - dx) * (1 - dy) * pixelTL +
                                      dx * (1 - dy) * pixelTR +
                                      (1 - dx) * dy * pixelBL +
                                      dx * dy * pixelBR;

            // 在目标图像中的线性索引
            int dstIdx = i * DEST_WIDTH + j;

            // 复制插值后的像素值到目标图像
            dst[dstIdx] = interpolatedPixel;
						dst[dstIdx] = dst[dstIdx] +128;
						
        }
    }
		

}



// 网络后处理函数
// 注意，正常的YOLO后处理都应该包含非极大值抑制NMS操作，但因为我比较懒所以没有加，这里只是根据置信度做了初步提取
void post_process_my()
{
	int grid_x, grid_y;
	float x=0, y=0, w=0 ,h=0;
	bool found = false;  
	for(int i = 0; i < 49; i++)
	{
		for(int j = 0; j < 3; j++)
		{
            // 网络输出维度是1*7*7*18
            // 其中18维度包含了每个像素预测的三个锚框，每个锚框对应6个维度，依次为x y w h conf class
            // 当然因为这个网络是单类检测，所以class这一维度没有用
			int8_t conf = out_data[i*18+j*6+4];
			printf("conf is %d\r\n", conf);
            // 这里的-9是根据网络量化的缩放偏移量计算的，对应的是70%的置信度
            // sigmoid((conf+15)*0.14218327403068542) < 0.7 ==> conf > -9
			if(conf > -9)
			{
				grid_x = i % 7;
				grid_y = (i - grid_x)/7;
				// 这里的15和0.14218327403068542就是网络量化后给出的缩放偏移量
				x = ((float)out_data[i*18+j*6]+15)*0.14218327403068542f;
				y = ((float)out_data[i*18+j*6+1]+15)*0.14218327403068542f;
				w = ((float)out_data[i*18+j*6+2]+15)*0.14218327403068542f;
				h = ((float)out_data[i*18+j*6+3]+15)*0.14218327403068542f;
                // 网络下采样三次，缩小了8倍，这里给还原回56*56的尺度
			
				x = (sigmoid(x)+grid_x) * 8;
				y = (sigmoid(y)+grid_y) * 8;
				w = expf(w) * anchors[j][0];
				h = expf(h) * anchors[j][1];		
				w=w/3; h=h/3;
				y2 = (x - w/2);
				y2 = 56-y2;
				y1 = (x + w/2);
				y1 = 56-y1;
				x1 = (y - h/2);
				x2 = (y + h/2);                                              
				if(x1 < 0) x1 = 0;
				if(y1 < 0) y1 = 0;
				if(x2 > 55) x2 = 55;
				if(y2 > 55) y2 = 55;



				printf("x1:%d, x2:%d, y1:%d ,y2:%d ,w:%f ,h:%f ,x:%f ,y:%f \r\n", x1, x2, y1, y2, w, h, x, y);
         // 绘制方框，左上角坐标为(x1, y1)，右下角坐标为(x2, y2)
         // 注意，如果输入图像是缩放到56*56再输入网络的话，这里的坐标还要乘以图像的缩放系数

				// 创建一个新的数组用于存储裁剪后的像素
				int8_t cropped_data[(y2-y1+1) * (x2-x1+1)];

				for(int i = y1; i <= y2; i++)
				{
						for(int j = x1; j <= x2; j++)
						{
								// 将RGB三通道转换为单通道
								cropped_data[(i-y1)*(x2-x1+1) + (j-x1)] = in_data[((i-1)*56+(j-1))*3] * 0.299 + in_data[((i-1)*56+(j-1))*3+1] * 0.587 + in_data[((i-1)*56+(j-1))*3+2] * 0.144;
						}
				}
				
//				  for(int i=0; i < ((y2-y1+1) * (x2-x1+1)); i++)
//					{
//						printf(" %d,", cropped_data[i]+128);
//					}


				// 使用插值算法进行缩放
				printf("cropped img size is %d  X %d\r\n",y2-y1+1,x2-x1+1);
				resizeFace(cropped_data, in_data_1, x2-x1+1, y2-y1+1);		

				BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);                                                              //Show inference results
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_DrawRect(x1*5.7143, y1*4.286, (x2-x1)*5.7143, (y2-y1)*4.286);                                            //画框，此处需要乘56*56到320*240的偏移量

				
        found = true;	
				break;				
			}
			
		}
				if(found)  // 如果找到符合条件的锚框，则跳出外层循环
				{
				break;
				}
	}
}



static uint32_t argmax(const float *values, uint32_t len)
{
    float max_value = values[0];
    uint32_t max_index = 0;
    for (uint32_t i = 1; i < len; i++)
    {
        if (values[i] > max_value)
        {
            max_value = values[i];
            max_index = i;
        }
    }
    return max_index;
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
