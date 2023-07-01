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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct car_state
{
  float v_left;//�����ٶ�
  float v_right;//�����ٶ�
  float x_left;//����λ��
  float x_right;//����λ��
  float x_all;//��λ��
  float v_target;//ֱ���ٶ�Ŀ��ֵ v_target=(v_left+v_right)/2
  float diff_v;//���ֺ����ֵĲ��ٶ�
  uint8_t dis_line;//С���Ҳ������ߵľ���
  uint8_t dis_target;
  uint8_t state;
  uint8_t task;
  float * x_left_targetArray;
};

struct pidstruct
{
  float kp,ki,kd;//p i d�ֱ��ϵ����Ҳ��������Ҫ���ڵ�ֵ
  float p_value,i_value,d_value;//P�� I�� D����������ֵ
  float i_integral;// ���Ļ���ֵ
  float thiserr,lasterr;//��ǰʱ�̵����ֵ ��ȥʱ�̵����ֵ ���ڼ��� i d
  unsigned char i_refresh_flag;
  unsigned char i_seprate_flag;
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define threshold 25 //��������ֵ��������Ϊ�޳���С����Ϊ�г�
#define wheel_len 20.42 //�����ܳ� ��λ��cm
#define Speed_ratio 2564.1//����������=13��ÿתһȦ�������������������� ���ٱ�1��30  ת��=1/(13*num*10^-6 *30) =2564.1/num (numΪ���α���������֮���õ�ʱ�Ӽ�������)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_PinState judge1,judge2;
float catch1[3],catch2[3];//catch[0] �����ϴ������ؼ�������ֵ catch[1] ������������ؼ�������ֵ catch[2]�������μ�����ֵ�Ĳ�
float mean_V1[10],mean_V2[10];
int index_V1,index_V2;
struct car_state now_car_state;
struct pidstruct Velocity_pidStruct_left;
struct pidstruct Velocity_pidStruct_right;
struct pidstruct Velocity_pidStruct_left_back;
struct pidstruct Velocity_pidStruct_right_back;
struct pidstruct Velocity_pidStruct_left_back_highspeed;
struct pidstruct Velocity_pidStruct_right_back_highspeed;
struct pidstruct* Velocity_pidStruct_left_points[3] = {&Velocity_pidStruct_left, &Velocity_pidStruct_left_back , &Velocity_pidStruct_left_back_highspeed};
struct pidstruct* Velocity_pidStruct_right_points[3] = {&Velocity_pidStruct_right, &Velocity_pidStruct_right_back , &Velocity_pidStruct_right_back_highspeed};
struct pidstruct Angle_pidStruct;
uint8_t Control_Flag = 1; //ң��ģʽ�ı�־
uint8_t rx_buf1[2]={0,0};//uart1���յ����ַ����ݼĴ�����
uint8_t rx_buf3[1]={0};//uart3���յ����ַ����ݼĴ�����
//float x_left_targetArray1[6] = {0 , 0 , 32 , -20,15,-37};
float x_left_targetArray1[6] = {0 , 0 , 42 , -35,15,-26};
float x_left_targetArray2[6] = {0 , 25 , -38 , -17,9,0};
float distance;
int judge;
int judge_open_flag = 0; //�����������ı�־ ������һ��T����֮����Ϊ1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Wheel_Drive(int num,int pwm);
int fputc(int ch, FILE *f);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);//���벶���ж�
// С��״̬�ṹ���ʼ������ ����С��״̬�ṹ�塢�趨��ʼ��Ŀ�깫���ٶȡ���ʼ�Ĳ��ٶ�
void init_carStruct(struct car_state * now_car_state, float v_common, float v_diff);
//pid�ṹ���ʼ���������ֱ���kp ki kdֵ���Լ�Ҫ��ʼ����pid�ṹ���ָ��
void init_pidStruct( float kp, float ki, float kd, unsigned char i_refresh_flag, unsigned char i_seprate_flag, struct pidstruct * pidStruct_to_init);
//��ʱ��2�жϻص����� ����PID����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//pid���ֵ���㺯�� �����Ѿ���ʼ���õ�pid�ṹ���ָ�� ��ʱ�����ֵ ���ֵ����ѹ�������ֵ ���ֵ����Сֵ ���PID����������ֵ
float PID(struct pidstruct * pidStruct_inited,float err,float outhigh,float outlow,unsigned char refresh_i_flag, unsigned char i_seprate_flag);
//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART1_DataGet(uint8_t * databuf);
uint8_t infrared_value(int mode);/*mode1����ͷ��·��mode2����β��·��mode3������·*/
void WaveStart(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void UART3_DataGet(uint8_t * databuf);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,rx_buf1,2);//�ȴ�uart1����һ���ֽ����ݣ�������rxbuf1��
  HAL_UART_Receive_IT(&huart3,rx_buf3,2);//�ȴ�uart1����һ���ֽ����ݣ�������rxbuf1��
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//ʹ��TIM2��PWM Channel1 ��� w1_pwm
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//ʹ��TIM2��PWM Channel2 ��� w2_pwm
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);//ʹ��TIM1 CH3 CH4�����벶���ж�
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
  __HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE ); //���IT��־λ
	HAL_TIM_Base_Start_IT(&htim3);             //������ʱ��3 ��ʱ���ж�


  init_carStruct(&now_car_state, 0, 0.0);

  /********************v-target = 20 *****diff-v = 0 ***************************/
  // init_pidStruct(25, 0.8, 0.0,1, 0x0,  &Velocity_pidStruct_left);
  // init_pidStruct(25, 0.7, 0.0, 1, 0x0,&Velocity_pidStruct_right);
  /******************************************************************************/
  /********************v-target = -20 diff-v = 0 *******************************/
  init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
  init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
  /******************************************************************************/
  /********************v-target = 20 diff-v = 2 *********************************/
  // init_pidStruct(20, 0.5, 0.0,1, 0x0,  &Velocity_pidStruct_left);
  // init_pidStruct(40, 1.0, 0.0, 1, 0x0,&Velocity_pidStruct_right);
  /*******************************************************************************/
  /*******************v-target = -20 diff-v = 2 *********************************/
  // init_pidStruct(20, 0.8, 0.0,1, 0x0,  &Velocity_pidStruct_left);
  // init_pidStruct(40, 0.8, 0.0, 0, 0x0,&Velocity_pidStruct_right);
  /******************************************************************************/
  init_pidStruct(35, 0.15, 0.0,1, 0x80+10,  &Velocity_pidStruct_left_back);
  init_pidStruct(40, 0.15, 0.0, 1, 0x80+10,&Velocity_pidStruct_right_back);
  init_pidStruct(55, 0.15, 0.0,1, 0x80+15,  &Velocity_pidStruct_left_back_highspeed);
  init_pidStruct(52, 0.15, 0.0, 1, 0x80+15,&Velocity_pidStruct_right_back_highspeed);
  init_pidStruct(6, 0, 0, 0, 0x0, &Angle_pidStruct);//10

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		WaveStart();
    HAL_Delay(200);

    //printf("judge = %d\tdistance=%.2f\n", judge, distance);
		
    //printf("state = %d, vt = %.2f\n", now_car_state.state, now_car_state.v_target);
    //printf("%d\n", infrared_value(3));
    //printf("vl=%.2f,vr=%.2f,vt=%.2f\n",now_car_state.v_left,now_car_state.v_right,now_car_state.v_target);
    //printf("%.2f,%.2f\n",now_car_state.v_left,now_car_state.v_right);
    // printf("dis_line = %.2f,diff_v = %.2f\n", now_car_state.dis_line, now_car_state.diff_v);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
void Wheel_Drive(int num,int pwm)
{
  if((num==1)||(num==2))
  {
    // ��С�޷� -1000<=pwm<=1000
    if(pwm>1000)
    {
      pwm=1000;
    }
    else if(pwm<-1000)
    {
      pwm=-1000;
    }
    switch(num)
    {
      case 1:
      {
        if(pwm>=0)
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000-pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);//w1_gpio_out ���Ʒ�����ת
        }
        else
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);//w1_gpio_out ���Ʒ���ת
        }
      }
      break;
      case 2:
      {
        if(pwm>=0)
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);//w2_gpio_out ���Ʒ�����ת
        }
        else
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000+pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);//w2_gpio_out ���Ʒ���ת
        }
      }
      break;
      default:
      break;
    }
  }
}

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);  
	return ch;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//���벶���ж�
{

  if(htim==&htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)// w1
  {
      judge1=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);//����w1B��������ʱ�ж�A���ƽ �ߵ�ƽΪ��ת���͵�ƽΪ��ת
      catch1[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);//��ȡ������ʱ��������ֵ
      catch1[2] = catch1[1]- catch1[0];
      if(catch1[2]<=0)
      {
        catch1[2]+=0xffff;//���������س��������ڵ�����ʱ������
      }
      catch1[0] = catch1[1];//����
      if(judge1 == GPIO_PIN_RESET)//��ת
      {
          //now_car_state.x_left+=wheel_len/13/30;
          *(mean_V1+index_V1) = wheel_len/catch1[2]*Speed_ratio;
          index_V1++;
      }
      else//��ת
      {
          //now_car_state.x_left-=wheel_len/13;
          *(mean_V1+index_V1) = -wheel_len/catch1[2]*Speed_ratio;
          index_V1++;
      }
      // *(mean_V1+index_V1) = wheel_len/catch1[2]*Speed_ratio;
      // index_V1++;
      if(index_V1==10)
      {
        now_car_state.v_left=(mean_V1[0]+mean_V1[1]+mean_V1[2]+mean_V1[3]+mean_V1[4]+mean_V1[5]+mean_V1[6]+mean_V1[7]+mean_V1[8]+mean_V1[9])/10;
        index_V1=0;
        //printf("%3.2lf\t",now_car_state.v_right);
      }
  }
    if(htim==&htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)// w2
  {
      judge2=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);//����w2B��������ʱ�ж�A���ƽ �ߵ�ƽΪ��ת���͵�ƽΪ��ת
      catch2[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);//��ȡ������ʱ��������ֵ
      catch2[2] = catch2[1]- catch2[0];
      if(catch2[2]<=0)
      {
        catch2[2]+=0xffff;//���������س��������ڵ�����ʱ������
      }
      catch2[0] = catch2[1];//����
      if(judge2 == GPIO_PIN_SET)//��ת
      {
          now_car_state.x_right+=wheel_len/13/30;
          *(mean_V2+index_V2) = wheel_len/catch2[2]*Speed_ratio;
          index_V2++;
      }
      else//��ת
      {
          now_car_state.x_right-=wheel_len;
          *(mean_V2+index_V2) = -wheel_len/catch2[2]*Speed_ratio;
          index_V2++;
      }
      // *(mean_V2+index_V2) = wheel_len/catch2[2]*Speed_ratio;
      // index_V2++;
      if(index_V2==10)
      {
        now_car_state.v_right=(mean_V2[0]+mean_V2[1]+mean_V2[2]+mean_V2[3]+mean_V2[4]+mean_V2[5]+mean_V2[6]+mean_V2[7]+mean_V2[8]+mean_V2[9])/10;
        //printf("%3.2lf\t",now_car_state.v_left);
        index_V2=0;
      }
  }

  //printf("%d\t",judge2);
}

// С��״̬�ṹ���ʼ������ ����С��״̬�ṹ�塢�趨��ʼ��Ŀ�깫���ٶȡ���ʼ�Ĳ��ٶ�
void init_carStruct(struct car_state * now_car_state, float v_common, float v_diff)
{
  now_car_state->v_target = v_common;
  now_car_state->diff_v = v_diff;
  now_car_state->v_left = 0.0;
  now_car_state->v_right = 0.0;
  now_car_state->x_left = 0.0;
  now_car_state->x_right= 0.0;
  now_car_state->x_all = 0.0;
  now_car_state->dis_line = 0;
  now_car_state->state=0;//��ʼ��Ϊѭ��ģʽ
  now_car_state->task=0;//��ʼ��Ϊ�����������
  now_car_state->x_left_targetArray = x_left_targetArray1;
}

//pid�ṹ���ʼ���������ֱ���kp ki kdֵ���Լ�Ҫ��ʼ����pid�ṹ���ָ��
void init_pidStruct( float kp, float ki, float kd, unsigned char i_refresh_flag, unsigned char i_seprate_flag, struct pidstruct * pidStruct_to_init)
{
  pidStruct_to_init->kp=kp;
  pidStruct_to_init->ki=ki;
  pidStruct_to_init->kd=kd;
  pidStruct_to_init->p_value=0;
  pidStruct_to_init->i_value=0;
  pidStruct_to_init->d_value=0;
  pidStruct_to_init->lasterr=0;
  pidStruct_to_init->thiserr=0;
  pidStruct_to_init->i_integral=0;
  pidStruct_to_init->i_refresh_flag = i_refresh_flag;
  pidStruct_to_init->i_seprate_flag = i_seprate_flag;
}

//pid���ֵ���㺯�� �����Ѿ���ʼ���õ�pid�ṹ���ָ�� ��ʱ�����ֵ ���ֵ����ѹ�������ֵ ���ֵ����Сֵ ���PID����������ֵ
float PID(struct pidstruct * pidStruct_inited,float err,float outhigh,float outlow,unsigned char refresh_i_flag, unsigned char i_seprate_flag)
{
  float out_value;
  pidStruct_inited->thiserr = err;
  pidStruct_inited->p_value = pidStruct_inited->kp * pidStruct_inited->thiserr;//�Ե�ǰ���ֵ�������ԷŴ�
  //�������΢���Ӧ�û�Ҫ���ԣ����ԣ���������T ������һ����õ��Ƕ�ʱ���жϽ������� ����TΪ���� ���Թ�ΪKi Kd��ֵ����
  if((i_seprate_flag & 0x10 == 0x80) && (abs(err) > i_seprate_flag & 0x7f)) // �����û��ַ���ģʽ�������ֵ̫��ʱ��������Ϊ0
  {
    pidStruct_inited->i_value = 0.0;
  }
  else // �������û��ַ���ģʽ���߲��û��ַ���ģʽ�������ֵ��Сʱ ���л�����ļ���
  {
    pidStruct_inited->i_integral += err;
    pidStruct_inited->i_value = pidStruct_inited->ki * (pidStruct_inited->i_integral);//�����ֵ�����ۼӺ�
    if(refresh_i_flag == 1)
    {
      if(pidStruct_inited->i_value > 0.5*outhigh || pidStruct_inited->i_value < 0.5*outlow)
      {
          init_pidStruct(pidStruct_inited->kp,pidStruct_inited->ki,pidStruct_inited->kd,pidStruct_inited->i_refresh_flag, pidStruct_inited->i_seprate_flag, pidStruct_inited);
      }
    } 
  }
  pidStruct_inited->d_value = pidStruct_inited->kd * (pidStruct_inited->thiserr - pidStruct_inited->lasterr);//�����ֵ���в��
  pidStruct_inited->lasterr = pidStruct_inited->thiserr;//�������
  out_value = pidStruct_inited->p_value + pidStruct_inited->i_value + pidStruct_inited->d_value;
  //�����ֵ�����޷�
  
  if(out_value > outhigh)
  {
    out_value = outhigh;
  }
  else if(out_value < outlow)
  {
    out_value = outlow;
  }
  return out_value;
}

//��ʱ��2�жϻص����� ����PID����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int pidOUTpwm1_v,pidOUTpwm2_v;
	if(htim == &htim3)//��ʱ���ж� ����pid
	{
    /*
    if(control_flag == 0)
    {
      Diff_Velocity=(int)PID( &Angle_pidStruct , now_car_state.angle_z_target-now_car_state.angle_z,now_car_state.v_target,-now_car_state.v_target,0);//��ֵ���Ϊ600
    }
    if(Diff_Velocity<2.5 && Diff_Velocity>-2.5)
    {
      Diff_Velocity=0;
    }
    pidOUTpwm3_v=(int)PID(  &Velocity_pidStruct_right , now_car_state.v_target+Diff_Velocity-now_car_state.v_right ,v_pid_outhigh,v_pid_outlow,0);
    pidOUTpwm4_v=(int)PID(  &Velocity_pidStruct_left , now_car_state.v_target-Diff_Velocity-now_car_state.v_left ,v_pid_outhigh,0.0,0);
    Wheel_Drive(3, pidOUTpwm3_v);
    Wheel_Drive(4, pidOUTpwm4_v);
    //ÿ6s��ʼ��һ�νǶ�pid
    if(pid_fresh_num == 300)
    {
      pid_fresh_num=0;
      init_pidStruct(Angle_pidStruct.kp,Angle_pidStruct.ki,Angle_pidStruct.kd,&Angle_pidStruct);
    }
    */
    if(Control_Flag == 0)
    {
      if(now_car_state.state == 5)//��������Һ���ʻ ���������
      {
        if(now_car_state.task == 1)
        {
          now_car_state.v_target = now_car_state.v_target > -20 ? now_car_state.v_target -2 : -20;
          now_car_state.diff_v = 0;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[5]) < 1) //������Һ���ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 6;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
            judge_open_flag = 0;
          }
        }
        
      }
      else if(now_car_state.state == 4)
      {
        if(now_car_state.task == 1)// ������� �Һ󷽵������ �ٴ�����ǰ����ʻ 
        {
          now_car_state.v_target = now_car_state.v_target < 20 ? now_car_state.v_target + 2 : 20;
          now_car_state.diff_v = now_car_state.diff_v < 5 ? now_car_state.diff_v + 0.5 : 5;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[4]) < 1) //�������ǰ����ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 5;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
          }
        }
        else if(now_car_state.task == 2)//�෽ͣ�� ����ǰ����ʻһС�ξ���
        {
          now_car_state.v_target = now_car_state.v_target < 20 ? now_car_state.v_target + 2 : 20;
          now_car_state.diff_v = now_car_state.diff_v > -5 ? now_car_state.diff_v - 0.5 : -5;
          
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[4]) < 1) //�������ǰ����ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 6;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
            judge_open_flag = 0;
          }
        }
      }
      else if(now_car_state.state == 3)
      {
        if(now_car_state.task == 1)// ������� ��ǰ����ʻ��Ŀ��� ��ʼ���Һ󷽵���
        {
          now_car_state.v_target = now_car_state.v_target > -20 ? now_car_state.v_target - 2 : -20;
          now_car_state.diff_v = now_car_state.diff_v < 4 ? now_car_state.diff_v + 0.4 : 4; 
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[3]) < 1) //������Һ���ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 5;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
          }
        }
        else if(now_car_state.task == 2) //�෽ͣ�� ����󲿷ֽ��복�� ���Һ󷽵��� ��������
        {
          // now_car_state.v_target = now_car_state.v_target > -25 ? now_car_state.v_target - 2.5 :-25;
          // now_car_state.diff_v = now_car_state.diff_v > -8 ? now_car_state.diff_v -0.8 : -8;
          now_car_state.v_target = -25;
          now_car_state.diff_v = -8;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[3]) < 1) //������Һ���ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 4;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
            init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
            init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
          }
        }
        
      }
      else if(now_car_state.state == 2)
      {
        if(now_car_state.task == 1)//������� С���������⿪ʼ�� ��ʼ����ǰ����ʻ
        {
          now_car_state.v_target = now_car_state.v_target < 20 ? now_car_state.v_target + 2 : 20;
          now_car_state.diff_v = now_car_state.diff_v < 4 ? now_car_state.diff_v + 0.8 : 4;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[2]) < 1) //�������ǰ����ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 3;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
          }
        }
        else if(now_car_state.task == 2)//�෽ͣ�� С���ڳ���ǰ��һ�ξ��봦��ʼ���Һ󷽵���
        {
          now_car_state.v_target = now_car_state.v_target > -25 ? now_car_state.v_target - 2.5 : -25;
          now_car_state.diff_v = now_car_state.diff_v < 8 ? now_car_state.diff_v + 0.8 : 8;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[2]) < 1) //������Һ󷽷���ʻ���趨�ľ���
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 3;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
            init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
            init_pidStruct(35, 1.2, 0.0, 1, 0x0,&Velocity_pidStruct_right);
          }
        }
      }
      else if(now_car_state.state == 1)//ѭ�����
      {
        if(now_car_state.task == 1)//������� ѭ����� ��ʼ���������⿪ʼ��
        {
          now_car_state.v_target =  -20;//-15;
          now_car_state.diff_v = 0;
          if(((infrared_value(3) & 0x80) == 0x80) && (now_car_state.x_left<-25)) //�ٴμ�⵽T���� ��ı�״̬
          {
            now_car_state.v_target =0;
            now_car_state.diff_v = 0 ;
            now_car_state.state = 2;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
          }          
        }
        else if(now_car_state.task == 2)//�෽ͣ�� ѭ����� ����ǰ��һ������
        {
          now_car_state.v_target = 20;
          //now_car_state.diff_v = 0;
          uint8_t dis_temp = infrared_value(3);
          if(((dis_temp & 0x80) == 0x00) )//��������ľ���ߵľ��벻Ϊ0������о���ĸ��£����򲻽��о�����£���ֹ���ֺ����������������м�����
          {
            if(dis_temp != 0x00 )
            {
              now_car_state.dis_line =  dis_temp; 
            }
            now_car_state.diff_v = PID( &Angle_pidStruct , 3 - now_car_state.dis_line, 20, -20, Angle_pidStruct.i_refresh_flag, Angle_pidStruct.i_seprate_flag);    
          }
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[1])<1)
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 2;
            now_car_state.x_left = 0; //�ڿ�ʼ��ת֮ǰ��λ��ֵ����
          }
        }
      }
      else if( now_car_state.state == 0) // ѭ��ģʽ
      {
        now_car_state.v_target = 15;
        uint8_t dis_temp = infrared_value(3);

        if(((dis_temp & 0x80) == 0x00) )//��������ľ���ߵľ��벻Ϊ0������о���ĸ��£����򲻽��о�����£���ֹ���ֺ����������������м�����
        {
          if(dis_temp != 0x00 )
          {
            now_car_state.dis_line =  dis_temp; 
          }
          if(now_car_state.task == 1)
          {
            if(judge_open_flag == 1 && now_car_state.x_left > 12 && now_car_state.x_left < 15 && judge == 1)
            {
              judge_open_flag = 3;
              now_car_state.dis_target = 2;
            }
            now_car_state.diff_v = PID( &Angle_pidStruct , now_car_state.dis_target - now_car_state.dis_line, 20, -20, Angle_pidStruct.i_refresh_flag, Angle_pidStruct.i_seprate_flag);    
          }
          else if(now_car_state.task == 2)
          {
            if(judge_open_flag == 1 && now_car_state.x_left > 15 && now_car_state.x_left < 18 && judge == 1)
            {
              judge_open_flag = 3;
            }
            now_car_state.diff_v = PID( &Angle_pidStruct , 3 - now_car_state.dis_line, 20, -20, Angle_pidStruct.i_refresh_flag, Angle_pidStruct.i_seprate_flag);    
          }
          //now_car_state.diff_v = PID( &Angle_pidStruct , 3 - now_car_state.dis_line, 20, -20, Angle_pidStruct.i_refresh_flag, Angle_pidStruct.i_seprate_flag);    
        }
        else if((dis_temp & 0x80) == 0x80 ) //��⵽T���� �ı�״̬ 
        {
          if(judge_open_flag == 3 ) // �Ѿ��������˳���������ʱ������T���β��ǵ�һ��,Ҳ���Ǿ�����һ������ judge == 1 ��ʾ��λ���� ���Խ���ͣ��
          {
            now_car_state.state = 1;
            now_car_state.v_target = 0;
            now_car_state.diff_v = 0;
          }
          else if(judge_open_flag == 0)
          {
            judge_open_flag = 1;
          }
          now_car_state.x_left = 0;
        }
      }

    }
    if(abs(now_car_state.v_left)>5)//5
    {
      now_car_state.x_left += now_car_state.v_left * 0.02 ;
    }
    //pidOUTpwm1_v=(int)PID(  Velocity_pidStruct_left_points[((now_car_state.v_target - now_car_state.diff_v)<=0)+(now_car_state.v_target - now_car_state.diff_v)<-30] , now_car_state.v_target - now_car_state.diff_v - now_car_state.v_left ,1000,-1000,Velocity_pidStruct_left.i_refresh_flag, Velocity_pidStruct_left.i_seprate_flag);
    //pidOUTpwm2_v=(int)PID(  Velocity_pidStruct_right_points[((now_car_state.v_target + now_car_state.diff_v)<=0)+(now_car_state.v_target + now_car_state.diff_v)<-30] , now_car_state.v_target + now_car_state.diff_v - now_car_state.v_right ,1000,-1000,Velocity_pidStruct_right.i_refresh_flag, Velocity_pidStruct_right.i_seprate_flag);
    pidOUTpwm1_v=(int)PID(  &Velocity_pidStruct_left, now_car_state.v_target - now_car_state.diff_v - now_car_state.v_left ,1000,-1000,Velocity_pidStruct_left.i_refresh_flag, Velocity_pidStruct_left.i_seprate_flag);
    pidOUTpwm2_v=(int)PID(  &Velocity_pidStruct_right , now_car_state.v_target + now_car_state.diff_v - now_car_state.v_right ,1000,-1000,Velocity_pidStruct_right.i_refresh_flag, Velocity_pidStruct_right.i_seprate_flag);
    Wheel_Drive(1, pidOUTpwm1_v);
    Wheel_Drive(2, pidOUTpwm2_v);    
	}
}


//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    UART1_DataGet(rx_buf1);
    HAL_UART_AbortReceive_IT(&huart1);//��ֹ���ڽ��е�Rx����
    HAL_UART_Receive_IT(&huart1,rx_buf1,2);//������һ�ִ��� �ȴ�2���ֽ����� ��д��
  }

  if(huart == &huart3)
  {
    UART1_DataGet(rx_buf3);
    HAL_UART_AbortReceive_IT(&huart3);//��ֹ���ڽ��е�Rx����
    HAL_UART_Receive_IT(&huart3,rx_buf3,2);//������һ�ִ��� �ȴ�2���ֽ����� ��д��
  }
}

void UART1_DataGet(uint8_t * databuf)
{
  if(*databuf == 'c')
  {
    Control_Flag = 1;
    if(*(databuf+1) == 'l' ) // ��ת
    {
      // Wheel_Drive(1,200);
      // Wheel_Drive(2,600);
      now_car_state.v_target = 20;
      now_car_state.diff_v = 5;
      // init_pidStruct(40, 0.8, 0.0,0, 0x0,  &Velocity_pidStruct_left);
      // init_pidStruct(20, 0.8, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'r' ) // ��ת
    {
      // Wheel_Drive(1,600);
      // Wheel_Drive(2,200);
      now_car_state.v_target = 20;
      now_car_state.diff_v = -5;
      //now_car_state.diff_v += 5;
    }
    else if(*(databuf+1) == 'f' ) // ǰ��
    {
      // Wheel_Drive(1,400);
      // Wheel_Drive(2,400);
      now_car_state.v_target = 20;
      now_car_state.diff_v = 0;
      init_pidStruct(25, 0.8, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.7, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'b' ) // ����
    {
      // Wheel_Drive(1,-400);
      // Wheel_Drive(2,-400);
      now_car_state.v_target = -20;
      now_car_state.diff_v = 0;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'R')
    {
      // now_car_state.v_target = -20;
      // now_car_state.diff_v = 2;
      now_car_state.v_target = -25;
      now_car_state.diff_v = -8;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(35, 1.2, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 's' ) // ֹͣ
    {
      // Wheel_Drive(1,0);
      // Wheel_Drive(2,0);
      now_car_state.v_target = 0;
      now_car_state.diff_v = 0;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'c' ) // תȦ
    {
      // Wheel_Drive(1,-300);
      // Wheel_Drive(2,300);
      now_car_state.v_target = 0;
      now_car_state.diff_v = 30;
    }

  }
  else if(*databuf == 't')//�������ģʽ
  {
    Control_Flag = 0; 
    now_car_state.v_target = 20;
    now_car_state.diff_v =0 ;
    now_car_state.state = 0;
    now_car_state.x_left = 0;
    if(*(databuf+1)== '1' || *(databuf+1)== 1)
    {
      now_car_state.task = 1;
      now_car_state.x_left_targetArray = x_left_targetArray1;
      now_car_state.dis_target = 3;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1)== '2' || *(databuf+1)== 2)
    {
      now_car_state.task = 2;
      now_car_state.x_left_targetArray = x_left_targetArray2;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
  }
}
void UART3_DataGet(uint8_t * databuf)
{
    //printf("%d\n",*(databuf));
    Control_Flag = 0; 
    now_car_state.v_target = 20;
    now_car_state.diff_v =0 ;
    now_car_state.state = 0;
    now_car_state.x_left = 0;
    if(*(databuf)== 1)
    {
      now_car_state.task = 1;
      now_car_state.x_left_targetArray = x_left_targetArray1;
      now_car_state.dis_target = 3;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf)== 2)
    {
      now_car_state.task = 2;
      now_car_state.x_left_targetArray = x_left_targetArray2;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
}
uint8_t infrared_value(int mode)
{/*mode1����ͷ��·��mode2����β��·��mode3������·*/
  uint8_t value =0x00;//����ֵ 
  uint8_t num = 0;//5·��⵽���ߵĴ���������
  switch (mode){
      case 1:
      {
          if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_SET)
          {
            value = 0x01;
          }
          else
          {
            value= 0x00;
          }
          break;   
      }
      case 2:
      {
          if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_SET)
          {
            value= 0x01;
          }
          else
          {
            value= 0x00;
          }
          break;
      }
      case 3:
      {
          if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_RESET)
          {
              value = 0x05;
              num++;
          }
          if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_RESET)
          {
              value = 0x04;
              num++;
          }
          if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_RESET)
          {
              value = 0x03;
              num++;
          }
          if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_RESET)
          {
              value = 0x02;
              num++;
          }
          if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==GPIO_PIN_RESET)
          {
              value = 0x01;
              num++;
          }
          if(num>=2)
            value=value|0x80;
          break;                
      }
      default:
      break;
  }
  return value;
}

void WaveStart(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    for(int i=0;i<720;i++);
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4); 
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)!=GPIO_PIN_RESET)
	{
		HAL_TIM_Base_Start(&htim4);
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_SET);
		HAL_TIM_Base_Stop(&htim4);
		distance=(__HAL_TIM_GetCounter(&htim4)*340)/2000.0;//��Ϊ������ÿ��һ��10us��ת��ΪcmΪ��λ*����340*100/100000/2
		__HAL_TIM_SET_COUNTER (&htim4, 0);
		if(distance>threshold)
		{
			judge=1;
		}
		else
		{
			judge=0;
		}
	}
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
