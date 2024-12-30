/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
//#include "ads1292r.h"
#include "lcd.h"
//#include "ADS1292Rii.h"
//#include "filter.h"
#include "arm_math.h"
#include "ADS1292.h"
#include "IIRFilter.h"
#include "FIRFilter.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// �����ĵ��źŷ�Χ
#define ECG_Y_MAX 25000  // �����źŷ�ΧΪ ��1000
#define FFT_X_START 10
#define FFT_Y_START 310
#define FFT_WIDTH 300
#define FFT_HEIGHT 120
#define FFT_LENGTH 1024
#define LCD_WIDTH  320
#define LCD_HEIGHT 240

#define KEY0 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)//��ȡPE4��ƽ
#define KEY1 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)//��ȡPE3��ƽ
#define KEY2 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)//��ȡPE2��ƽ
#define WK_UP HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)//��ȡPA0��ƽ
#define KEY0_Press 1//KEY0�����±�־λ
#define KEY1_Press 2//KEY1�����±�־λ
#define KEY2_Press 3//KEY2�����±�־λ
#define WK_UP_Press 4//WK_UP�����±�־λ
uint16_t signal_type_flag = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern uint8_t ADS1292R_data_buf[9]; //ADS1292 data buffer
//extern uint8_t ADS1292R_receive_flag;
//
//uint32_t channel[2] = {0}; //ADS1292R dual channel data
//
//extern uint8_t testID;

/************************************************************************************************/
//
//float freq;
//int x, y;
//float ft = 1300000;
//float32_t data_cache[36], output[36], mid_filt_cache[midfilt_num], mid_filt_cache1[midfilt_num], data_cache1[36];
//int mid_filt_count;
//int j, z;
//float mid_val;
//int mid_filt_start_flag = 0;
//volatile uint8_t ads1292_recive_flag=1;
//volatile uint8_t ads1292_Cache[9];
//int HR = 0;
//float HB_count = 0;
//float time1 = 0;
//float time2 = 0;
//float delta_t = 0;
//float derv = 0;
//int near_flag = 0;
//float last_ft = 0;
//
//
//#define FFT_SIZE 512  // ���� FFT �Ĵ�С�������� 2 ����
//float32_t fft_input[FFT_SIZE];      // FFT ���뻺����
//float32_t fft_output[FFT_SIZE];     // FFT ����������������ԳƲ��֣�
//float32_t fft_magnitude[FFT_SIZE / 2];  // ����Ƶ�׷�ֵ
//arm_rfft_fast_instance_f32 fft_instance;  // FFT ʵ��
/************************************************************************************************/


uint8_t ads1292_recive_flag;

extern int16_t IIR_Result;
extern int16_t FIRResult;
int heart_beat = 0;
double heart_rate;

#define ECG_COUNT 1024 // 345
int16_t ECG_Signal[ECG_COUNT];
uint16_t ecg_num = 0;
float frequency = 0;

#define FFT_LENGTH 1024
float FFT_InputBufmy[FFT_LENGTH * 2];
float FFT_OutputBufmy[FFT_LENGTH];
arm_cfft_radix4_instance_f32 scfft;
float32_t max_value = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int32_t get_volt(uint32_t num);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Draw_ECG_Waveform(float signal_value);
int32_t convert_to_signed(uint32_t raw_data);
//float convert_to_voltage(int32_t data, float vref, int gain);
float convert_to_voltage(float data, float vref, int gain, int resolution);

int test_i = 0;
uint8_t key = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
void axis_drawing(){
    LCD_ShowString(90,10,300,16,16,"QEA_WORK");
    LCD_DrawLine(0,160,320,160);
//    LCD_ShowNum(20,200,0,4,12);
//    LCD_ShowNum(64,200,20,4,12);
//    LCD_ShowNum(108,200,30,4,12);
//    LCD_ShowNum(152,200,500,4,12);
//    LCD_ShowNum(196,200,50,4,12);
//    LCD_ShowNum(240,200,60,4,12);
//    LCD_ShowNum(280,200,1000,4,12);
    LCD_DrawLine(120,195,120,40);
//    LCD_DrawLine(160,40,155,50);
//    LCD_DrawLine(100,410,165,50);
//    LCD_ShowString(45,240,300,16,16,"-------  Spectrogram  -------");
//    LCD_ShowNum(170,00,90,2,12);
//    LCD_ShowNum(170,20,80,2,12);
//    LCD_ShowNum(170,40,70,2,12);
//    LCD_ShowNum(170,60,60,2,12);
//    LCD_ShowFloat(170,60,12,3.31,1,2);
//    LCD_ShowNum(170,80,50,2,12);
//    LCD_ShowNum(170,100,40,2,12);
//    LCD_ShowFloat(170,110,12,0,1,2);
//    LCD_ShowNum(170,120,30,2,12);
//    LCD_ShowNum(170,140,20,2,12);
//    LCD_ShowNum(170,140,0,2,12);
//    LCD_ShowNum(170,180,0,2,12);
//    LCD_ShowString(20,450,300,16,16,"************  QEA LYT  ************");

}

//void calculate_fft_and_heart_rate(float32_t *input, uint32_t size, float32_t sampling_rate) {
//    // ִ�� FFT
//    arm_rfft_fast_f32(&fft_instance, input, fft_output, 0);
//
//    // �����ֵ��ƽ�������֣�
//    for (uint32_t i = 0; i < size / 2; i++) {
//        fft_magnitude[i] = sqrtf(fft_output[2 * i] * fft_output[2 * i] +
//                                 fft_output[2 * i + 1] * fft_output[2 * i + 1]);
//    }
//
//    // �ҵ���Ƶ�ʶ�Ӧ������
//    float32_t max_value;
//    uint32_t max_index;
//    arm_max_f32(fft_magnitude, size / 2, &max_value, &max_index);
//
//    // ������Ƶ��
//    float32_t main_frequency = (float32_t)max_index * sampling_rate / (float32_t)size;
//
//    // �������� (BPM)
//    float32_t heart_rate = main_frequency * 60.0f;
////    printf("{HRR}");
////    printf("%.2f\n", heart_rate);
//}
void DrawFFT(void) {
    float x, y;
    float x_scale = (float) FFT_WIDTH / (FFT_LENGTH / 2); // X�����ű���
    float y_scale;


    // Ѱ�ҷ�ֵ���ֵ������Y������
    for (int i = 0; i < FFT_LENGTH / 2; i++) {
        if (FFT_OutputBufmy[i] > max_value) {
            max_value = FFT_OutputBufmy[i];
        }
    }

    // ��ֹ������
    if (max_value == 0) {
        max_value = 1.0f;
    }

    y_scale = (float) FFT_HEIGHT / max_value;

    // �����ͼ��
    // LCD_ClearArea(FFT_X_START, FFT_Y_START, FFT_WIDTH, FFT_HEIGHT);

    // ����Ƶ��
    for (int i = 0; i < FFT_LENGTH / 2; i++) {
        x = FFT_X_START + i * x_scale;
        y = FFT_Y_START - FFT_OutputBufmy[i] * y_scale;

        // ����������ת��Ϊ����������л���
        LCD_DrawLine((uint16_t) x, FFT_Y_START, (uint16_t) x, (uint16_t) y);
    }
//    LCD_ShowString(10,380,20,10,16,"���ź�");

    LCD_ShowString(15,200,300,16,16,"Heartbeat rate is");

    LCD_ShowNum(135,200,heart_rate,4,16);

}
void test_mode();
void normal_mode();
uint8_t KEY_Scan(uint8_t mode)
{
    static uint8_t key_up=1;//�����ɿ���־
    if(mode)key_up=1; //֧������
    if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
    {
        HAL_Delay(10);
        key_up=0;
        if(KEY0==0)return KEY0_Press;
        else if(KEY1==0)return KEY1_Press;
        else if(KEY2==0)return KEY2_Press;
        else if(WK_UP==1)return WK_UP_Press;
    }else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1;
    return 0;//�ް�����???
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
//  ADS1292R_Init();
//  printf("Init OK!!!!!! \n");
////  ADS1292R_ADCStartNormal();
//  ADS1292R_ADCStartTest();
//  printf("ADS start successfully!!!!!! \n");


  HAL_Delay(50);
  LCD_Init();
  LCD_Clear(WHITE);
  POINT_COLOR=RED;
  LCD_Clear(RED);
    HAL_Delay(2000);
    LCD_Clear(WHITE);

    key = KEY_Scan(0);
/************************************************************************/
//  ADS1292_Init();
//  while(Set_ADS1292_Collect(0))
//  {
//    printf("reg set error\r\n");
//    HAL_Delay(1);
//  }
//  printf("reg set success\r\n");
//  ads1292_recive_flag=0;
/************************************************************************/
    ADS1292R_Init();
    if(key == KEY0_Press){
        PowerOnTest();
        test_i = 1;
    }else if(key == KEY1_Press){
        PowerOnNormal();

    }else {
        PowerOnNormal();

    }




    ADS1292R_Work();
    arm_cfft_radix4_init_f32(&scfft, FFT_LENGTH, 0, 1);
    FIRInit();
/************************************************************************/

//  arm_fir48_init();
//  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
  POINT_COLOR=GREEN;
  axis_drawing();
  POINT_COLOR=RED;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /*************************************************************************************/
//      if(ADS1292R_receive_flag == 1){
//          ADS1292R_receive_flag = 0;
//          int32_t tmp;
////          channel[0] = get_volt(ADS1292R_data_buf[3] << 16 | ADS1292R_data_buf[4] << 8 | ADS1292R_data_buf[5]); //get ECG data
//          tmp = get_volt(ADS1292R_data_buf[6] << 16 | ADS1292R_data_buf[7] << 8 | ADS1292R_data_buf[8]);
////          float tmp = convert_to_voltage((int32_t)channel[1], 2.4, 12);
//          float ecg_data = convert_to_voltage(tmp, 2.42f, 12);
////          printf("Channel 0: %lu, Channel 1: %lu\r\n", channel[0], channel[1]);
////          printf("{channel0}");
////          printf("%lu\n",channel[0]);
//          printf("{channel1}");
//          printf("%lu\n",tmp);
//          printf("{ecg}");
//          printf("%f\n", ecg_data);
//          // ��ǰX����
////          static uint16_t x = 0;
////
////          // �����ĵ粨��
////          Draw_ECG_Waveform(x, channel[1]);
////
////          // ����X���꣬ѭ����ͷ��ʼ
////          x = (x + 10) % LCD_WIDTH;
//
//          HAL_Delay(10);  // ���Ƹ���Ƶ��
          /*************************************************************************************/
//      uint32_t channel[2];
//      int32_t p_Temp[2];

//      if(ads1292_recive_flag == 1) {
//          channel[0] = ads1292_Cache[3] << 16 | ads1292_Cache[4] << 8 | ads1292_Cache[5];//��ȡԭʼ����
//          channel[1] = ads1292_Cache[6] << 16 | ads1292_Cache[7] << 8 | ads1292_Cache[8];
//
//          p_Temp[0] = get_volt(channel[0]);    //�Ѳɵ���3���ֽ�ת���з���32λ��
//          p_Temp[1] = get_volt(channel[1]);    //�Ѳɵ���3���ֽ�ת���з���32λ��
//          printf("{channel1}");
//          printf("%lu\n",p_Temp[1]);
//          data_cache[j] = p_Temp[1];                                //??????????????
//
//          j++;
//          z++;
//
//          if(j == 19)
//          {
////
//              j=18;
////
//              arm_fir_f32_lp_48(data_cache, output);              //?????FIR 48Hz????
////
//              if(mid_filt_start_flag == 0)
//              {
//                  mid_filt_cache[mid_filt_count] = output[0];
//                  mid_filt_count++;
//                  if(mid_filt_count == midfilt_num)
//                  {
//                      mid_filt_start_flag = 1;
//                  }
//              }
//              else if(mid_filt_start_flag == 1)
//              {
//                  arm_copy_f32(mid_filt_cache+1,mid_filt_cache1,midfilt_num-1);
//                  mid_filt_cache1[midfilt_num-1] = output[0];
//                  mid_val = midfilt1(mid_filt_cache1, midfilt_num, midfilt_num);
//
//                  arm_copy_f32(mid_filt_cache1, mid_filt_cache, midfilt_num);
//              }
////
//              arm_copy_f32(data_cache+1, data_cache1, 18);
//              arm_copy_f32(data_cache1, data_cache, 18);
////
//          }
////
//          ft = output[0] - mid_val;
//          printf("{ft}");
//          printf("%f\n",ft);
//          float voltage = convert_to_voltage(ft, 2.42f, 6, 24);
//          printf("{v}");
//          printf("%f\n",voltage);
////				ft = output[0];
////				printf("f = %f\r\n", ft);
//
////          lcd_display(ft);
//          Draw_ECG_Waveform(voltage);
////          //heart rate
//          derv = ft - last_ft;
//          if(near_flag == 0)
//          {
//              if(derv < - 1000)
//              {
//                  time2 = HAL_GetTick();
//                  delta_t = 0.001 * (time2 - time1);
//                  time1 = time2;
//                  near_flag = 1;
//              }
//          }
//          else if(near_flag < 20)
//          {
//              near_flag += 1;
//          }
//          else if(near_flag == 20)
//          {
//              near_flag = 0;
//          }
//          last_ft = ft;
//
//          HR = (int)(60.0 / delta_t);
//          printf("{HR}");
//          printf("%d\n", HR);
//
////          LCD_ShowxNum(50, 440, HR, 3, 24, 0);
////          LCD_ShowString(10, 440, 200, 24, 24, "HR: ");
//          ads1292_recive_flag = 0;
//          static uint32_t fft_index = 0;
//          if (fft_index < FFT_SIZE) {
//              fft_input[fft_index++] = ft;
//          } else {
//              fft_index = 0;
//              calculate_fft_and_heart_rate(fft_input, FFT_SIZE, 500.0f);  // ���������Ϊ 250 Hz
//          }
//
////          HAL_Delay(1000);

//      }
      /*************************************************************************************/
      if (ads1292_recive_flag == 1) {
          if(test_i){
              test_mode();
          }else{
              normal_mode();
          }




      }


    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void test_mode(void){
    ads1292_recive_flag = 0; // �����־λ
    data_trans();             // ��������
//          printf("{ecg_raw_data}");
//          printf("%x\n",ECGRawData[0]);
    Draw_ECG_Waveform(ECGRawData[0]);
    LCD_ShowFloat(170,180,12,0.1,1,2);
}
void normal_mode(void){
    ads1292_recive_flag = 0; // �����־λ
    data_trans();             // ��������
//          printf("{ecg_raw_data}");
//          printf("%x\n",ECGRawData[0]);
//    Draw_ECG_Waveform(ECGRawData[0]);
    IIRFilter(ECGRawData[0]); // IIR�˲�
    FIRFilter(IIR_Result);    // FIR�˲�
//          FIRFilter(ECGRawData[0]);
    printf("{filterdata}");
    printf("%d\n",-FIRResult);

    heart_beat = heartbeat_check(FIRResult);      // �������
    heart_rate = calc_heartbeat_rate(heart_beat); // ���ʼ���
    Draw_ECG_Waveform(-FIRResult);
    printf("{hrr}");
    printf("%f\n",heart_rate);
    int atmp = 0;
    if (ecg_num <= ECG_COUNT)
    {
        ECG_Signal[ecg_num] = FIRResult; // ��¼����
//              printf("{FIRRes}");
//              printf("%hd\n", FIRResult);
        FFT_InputBufmy[2 * ecg_num] = ECG_Signal[ecg_num];
        FFT_InputBufmy[2 * ecg_num + 1] = 0;
        ecg_num++;
    }
    else
    {
        arm_cfft_radix4_f32(&scfft, FFT_InputBufmy);                    //
        arm_cmplx_mag_f32(FFT_InputBufmy, FFT_OutputBufmy, FFT_LENGTH); //
        ecg_num = 0;
//              drawCurve(ECG_Signal, ECG_COUNT); // ������

        DrawFFT();
        atmp = 1;
    }
}


/**
  * @name   get_volt(uint32_t num)
  *
  * @brief  Convert the 3 bytes complement to a signed 32-bit number.
  *
  * @note
  *
  * @param  num: the 3 bytes data you have received.
  *
  * @retval A signed 32-bit number
  */
int32_t get_volt(uint32_t num)
{
    uint32_t temp;
    temp = num;
    temp <<= 8;
    temp >>= 8;
    return temp;
}

/**
  * @name   HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  *
  * @brief  External interrupt callback function.
  *         When a falling edge on DRDY_Pin, the ADS1292R_receive_flag will
  *         set to 1 , then SPI will be ready to receive data.
  *
  * @note
  *
  * @retval None
  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == ADS1292R_DRDY_Pin)
//    {
//        ADS1292R_GetValue();
//        ADS1292R_receive_flag = 1;
//    }
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    if (GPIO_Pin == key_up_Pin)
//    {
//        key_flag = 1;
//    }
//    if (GPIO_Pin == key_down_Pin)
//    {
//        key_flag = 2;
//    }
    if (GPIO_Pin == ADS1292R_DRDY_Pin)
    {
      if(ads1292_recive_flag == 0)
      {
          ADS1292R_ReadData();
          ads1292_recive_flag = 1;
      }
    }
}
void Draw_ECG_Waveform( float signal_value) {
    static uint16_t global_x = 0;
    static uint16_t prev_x = 0;
    static uint16_t prev_y = 150;

    // ���ź�ֵӳ�䵽��ĻY����
    uint16_t y = 150 - (signal_value * 150 / ECG_Y_MAX);

    // �����ǰ�еľ�����
//    for (int i = 0; i < 240; i++) {
//        LCD_DrawPoint(global_x, i);
//    }
    if (prev_x < 319){
        // �����²��ε�
        LCD_DrawLine(prev_x, prev_y, global_x, y);

    }


    // ����ǰһ����
    prev_x = global_x;
    prev_y = y;
    global_x += 1;
    if(global_x == 320){
        LCD_Clear(WHITE);
        global_x = 0;
        POINT_COLOR=GREEN;
        axis_drawing();
        POINT_COLOR=RED;
    }

}

int32_t convert_to_signed(uint32_t raw_data) {
    if (raw_data & 0x800000) {  // ������λ���� 24 λ��
        raw_data |= 0xFF000000; // ����Ǹ��������� 8 λ���Ϊ 1
    }
    return raw_data;  // ת��Ϊ�з��� 32 λ����
}

//float convert_to_voltage(int32_t data, float vref, int gain)
//{
//    return (data * vref) / (gain * 8388608.0); // 8388608 = 2^23
//}
float convert_to_voltage(float data, float vref, int gain, int resolution) {
    return (data / (float)((1 << resolution) - 1)) * vref / gain;
}

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
