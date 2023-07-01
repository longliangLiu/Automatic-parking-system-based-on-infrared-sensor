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
  float v_left;//左轮速度
  float v_right;//右轮速度
  float x_left;//左轮位移
  float x_right;//右轮位移
  float x_all;//总位移
  float v_target;//直线速度目标值 v_target=(v_left+v_right)/2
  float diff_v;//左轮和右轮的差速度
  uint8_t dis_line;//小车右侧距离黑线的距离
  uint8_t dis_target;
  uint8_t state;
  uint8_t task;
  float * x_left_targetArray;
};

struct pidstruct
{
  float kp,ki,kd;//p i d分别的系数，也是我们需要调节的值
  float p_value,i_value,d_value;//P项 I项 D项计算出来的值
  float i_integral;// 误差的积分值
  float thiserr,lasterr;//当前时刻的误差值 过去时刻的误差值 用于计算 i d
  unsigned char i_refresh_flag;
  unsigned char i_seprate_flag;
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define threshold 25 //超声波阈值，大于判为无车，小于判为有车
#define wheel_len 20.42 //轮子周长 单位：cm
#define Speed_ratio 2564.1//编码器线数=13（每转一圈编码器产生的脉冲数） 减速比1：30  转速=1/(13*num*10^-6 *30) =2564.1/num (num为两次编码器脉冲之间测得的时钟计数个数)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_PinState judge1,judge2;
float catch1[3],catch2[3];//catch[0] 保存上次上升沿计数器的值 catch[1] 保存这次上升沿计数器的值 catch[2]保存两次计数器值的差
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
uint8_t Control_Flag = 1; //遥控模式的标志
uint8_t rx_buf1[2]={0,0};//uart1接收到的字符数据寄存数组
uint8_t rx_buf3[1]={0};//uart3接收到的字符数据寄存数组
//float x_left_targetArray1[6] = {0 , 0 , 32 , -20,15,-37};
float x_left_targetArray1[6] = {0 , 0 , 42 , -35,15,-26};
float x_left_targetArray2[6] = {0 , 25 , -38 , -17,9,0};
float distance;
int judge;
int judge_open_flag = 0; //开启超声波的标志 碰到第一个T字形之后置为1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Wheel_Drive(int num,int pwm);
int fputc(int ch, FILE *f);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);//输入捕获中断
// 小车状态结构体初始化函数 传入小车状态结构体、设定初始的目标公共速度、初始的差速度
void init_carStruct(struct car_state * now_car_state, float v_common, float v_diff);
//pid结构体初始化函数，分别传入kp ki kd值，以及要初始化的pid结构体的指针
void init_pidStruct( float kp, float ki, float kd, unsigned char i_refresh_flag, unsigned char i_seprate_flag, struct pidstruct * pidStruct_to_init);
//定时器2中断回调函数 进行PID更新
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//pid输出值计算函数 传入已经初始化好的pid结构体的指针 此时的误差值 输出值（电压）的最大值 输出值的最小值 输出PID计算出的输出值
float PID(struct pidstruct * pidStruct_inited,float err,float outhigh,float outlow,unsigned char refresh_i_flag, unsigned char i_seprate_flag);
//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART1_DataGet(uint8_t * databuf);
uint8_t infrared_value(int mode);/*mode1代表车头单路，mode2代表车尾单路，mode3代表五路*/
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
  HAL_UART_Receive_IT(&huart1,rx_buf1,2);//等待uart1接收一个字节数据，并存入rxbuf1中
  HAL_UART_Receive_IT(&huart3,rx_buf3,2);//等待uart1接收一个字节数据，并存入rxbuf1中
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//使能TIM2的PWM Channel1 输出 w1_pwm
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//使能TIM2的PWM Channel2 输出 w2_pwm
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);//使能TIM1 CH3 CH4的输入捕获中断
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
  __HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE ); //清除IT标志位
	HAL_TIM_Base_Start_IT(&htim3);             //启动定时器3 定时器中断


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
    // 大小限幅 -1000<=pwm<=1000
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
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);//w1_gpio_out 控制方向正转
        }
        else
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);//w1_gpio_out 控制方向反转
        }
      }
      break;
      case 2:
      {
        if(pwm>=0)
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);//w2_gpio_out 控制方向正转
        }
        else
        {
          __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000+pwm);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);//w2_gpio_out 控制方向反转
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


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//输入捕获中断
{

  if(htim==&htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)// w1
  {
      judge1=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);//捕获到w1B相上升沿时判断A相电平 高电平为正转，低电平为反转
      catch1[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);//读取上升沿时计数器的值
      catch1[2] = catch1[1]- catch1[0];
      if(catch1[2]<=0)
      {
        catch1[2]+=0xffff;//两次上升沿出现在相邻的两个时钟周期
      }
      catch1[0] = catch1[1];//更新
      if(judge1 == GPIO_PIN_RESET)//正转
      {
          //now_car_state.x_left+=wheel_len/13/30;
          *(mean_V1+index_V1) = wheel_len/catch1[2]*Speed_ratio;
          index_V1++;
      }
      else//反转
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
      judge2=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);//捕获到w2B相上升沿时判断A相电平 高电平为正转，低电平为反转
      catch2[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);//读取上升沿时计数器的值
      catch2[2] = catch2[1]- catch2[0];
      if(catch2[2]<=0)
      {
        catch2[2]+=0xffff;//两次上升沿出现在相邻的两个时钟周期
      }
      catch2[0] = catch2[1];//更新
      if(judge2 == GPIO_PIN_SET)//正转
      {
          now_car_state.x_right+=wheel_len/13/30;
          *(mean_V2+index_V2) = wheel_len/catch2[2]*Speed_ratio;
          index_V2++;
      }
      else//反转
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

// 小车状态结构体初始化函数 传入小车状态结构体、设定初始的目标公共速度、初始的差速度
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
  now_car_state->state=0;//初始化为循迹模式
  now_car_state->task=0;//初始化为倒车入库任务
  now_car_state->x_left_targetArray = x_left_targetArray1;
}

//pid结构体初始化函数，分别传入kp ki kd值，以及要初始化的pid结构体的指针
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

//pid输出值计算函数 传入已经初始化好的pid结构体的指针 此时的误差值 输出值（电压）的最大值 输出值的最小值 输出PID计算出的输出值
float PID(struct pidstruct * pidStruct_inited,float err,float outhigh,float outlow,unsigned char refresh_i_flag, unsigned char i_seprate_flag)
{
  float out_value;
  pidStruct_inited->thiserr = err;
  pidStruct_inited->p_value = pidStruct_inited->kp * pidStruct_inited->thiserr;//对当前误差值进行线性放大
  //积分项和微分项本应该还要乘以（除以）驱动周期T 但由于一般采用的是定时器中断进行驱动 所以T为常数 可以归为Ki Kd的值里面
  if((i_seprate_flag & 0x10 == 0x80) && (abs(err) > i_seprate_flag & 0x7f)) // 当采用积分分离模式并且误差值太大时，积分项为0
  {
    pidStruct_inited->i_value = 0.0;
  }
  else // 当不采用积分分离模式或者采用积分分离模式但是误差值较小时 进行积分项的计算
  {
    pidStruct_inited->i_integral += err;
    pidStruct_inited->i_value = pidStruct_inited->ki * (pidStruct_inited->i_integral);//对误差值进行累加和
    if(refresh_i_flag == 1)
    {
      if(pidStruct_inited->i_value > 0.5*outhigh || pidStruct_inited->i_value < 0.5*outlow)
      {
          init_pidStruct(pidStruct_inited->kp,pidStruct_inited->ki,pidStruct_inited->kd,pidStruct_inited->i_refresh_flag, pidStruct_inited->i_seprate_flag, pidStruct_inited);
      }
    } 
  }
  pidStruct_inited->d_value = pidStruct_inited->kd * (pidStruct_inited->thiserr - pidStruct_inited->lasterr);//对误差值进行差分
  pidStruct_inited->lasterr = pidStruct_inited->thiserr;//更新误差
  out_value = pidStruct_inited->p_value + pidStruct_inited->i_value + pidStruct_inited->d_value;
  //对输出值进行限幅
  
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

//定时器2中断回调函数 进行PID更新
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int pidOUTpwm1_v,pidOUTpwm2_v;
	if(htim == &htim3)//定时器中断 更新pid
	{
    /*
    if(control_flag == 0)
    {
      Diff_Velocity=(int)PID( &Angle_pidStruct , now_car_state.angle_z_target-now_car_state.angle_z,now_car_state.v_target,-now_car_state.v_target,0);//差值最大为600
    }
    if(Diff_Velocity<2.5 && Diff_Velocity>-2.5)
    {
      Diff_Velocity=0;
    }
    pidOUTpwm3_v=(int)PID(  &Velocity_pidStruct_right , now_car_state.v_target+Diff_Velocity-now_car_state.v_right ,v_pid_outhigh,v_pid_outlow,0);
    pidOUTpwm4_v=(int)PID(  &Velocity_pidStruct_left , now_car_state.v_target-Diff_Velocity-now_car_state.v_left ,v_pid_outhigh,0.0,0);
    Wheel_Drive(3, pidOUTpwm3_v);
    Wheel_Drive(4, pidOUTpwm4_v);
    //每6s初始化一次角度pid
    if(pid_fresh_num == 300)
    {
      pid_fresh_num=0;
      init_pidStruct(Angle_pidStruct.kp,Angle_pidStruct.ki,Angle_pidStruct.kd,&Angle_pidStruct);
    }
    */
    if(Control_Flag == 0)
    {
      if(now_car_state.state == 5)//最后再向右后方行驶 完成入库操作
      {
        if(now_car_state.task == 1)
        {
          now_car_state.v_target = now_car_state.v_target > -20 ? now_car_state.v_target -2 : -20;
          now_car_state.diff_v = 0;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[5]) < 1) //如果向右后方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 6;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
            judge_open_flag = 0;
          }
        }
        
      }
      else if(now_car_state.state == 4)
      {
        if(now_car_state.task == 1)// 倒车入库 右后方倒车完成 再次向左前方行驶 
        {
          now_car_state.v_target = now_car_state.v_target < 20 ? now_car_state.v_target + 2 : 20;
          now_car_state.diff_v = now_car_state.diff_v < 5 ? now_car_state.diff_v + 0.5 : 5;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[4]) < 1) //如果向左前方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 5;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
          }
        }
        else if(now_car_state.task == 2)//侧方停车 向右前方行驶一小段距离
        {
          now_car_state.v_target = now_car_state.v_target < 20 ? now_car_state.v_target + 2 : 20;
          now_car_state.diff_v = now_car_state.diff_v > -5 ? now_car_state.diff_v - 0.5 : -5;
          
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[4]) < 1) //如果向左前方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 6;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
            judge_open_flag = 0;
          }
        }
      }
      else if(now_car_state.state == 3)
      {
        if(now_car_state.task == 1)// 倒车入库 左前方行驶至目标点 开始向右后方倒车
        {
          now_car_state.v_target = now_car_state.v_target > -20 ? now_car_state.v_target - 2 : -20;
          now_car_state.diff_v = now_car_state.diff_v < 4 ? now_car_state.diff_v + 0.4 : 4; 
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[3]) < 1) //如果向右后方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 5;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
          }
        }
        else if(now_car_state.task == 2) //侧方停车 车体大部分进入车库 向右后方倒车 摆正车体
        {
          // now_car_state.v_target = now_car_state.v_target > -25 ? now_car_state.v_target - 2.5 :-25;
          // now_car_state.diff_v = now_car_state.diff_v > -8 ? now_car_state.diff_v -0.8 : -8;
          now_car_state.v_target = -25;
          now_car_state.diff_v = -8;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[3]) < 1) //如果向右后方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 4;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
            init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
            init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
          }
        }
        
      }
      else if(now_car_state.state == 2)
      {
        if(now_car_state.task == 1)//倒车入库 小车倒至车库开始线 开始向左前方行驶
        {
          now_car_state.v_target = now_car_state.v_target < 20 ? now_car_state.v_target + 2 : 20;
          now_car_state.diff_v = now_car_state.diff_v < 4 ? now_car_state.diff_v + 0.8 : 4;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[2]) < 1) //如果向左前方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 3;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
          }
        }
        else if(now_car_state.task == 2)//侧方停车 小车在车库前面一段距离处开始向右后方倒车
        {
          now_car_state.v_target = now_car_state.v_target > -25 ? now_car_state.v_target - 2.5 : -25;
          now_car_state.diff_v = now_car_state.diff_v < 8 ? now_car_state.diff_v + 0.8 : 8;
          if(abs(now_car_state.x_left - now_car_state.x_left_targetArray[2]) < 1) //如果向右后方方行驶了设定的距离
          {
            now_car_state.v_target = 0;
            now_car_state.diff_v =0;
            now_car_state.state = 3;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
            init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
            init_pidStruct(35, 1.2, 0.0, 1, 0x0,&Velocity_pidStruct_right);
          }
        }
      }
      else if(now_car_state.state == 1)//循迹完成
      {
        if(now_car_state.task == 1)//倒车入库 循迹完成 开始倒车至车库开始线
        {
          now_car_state.v_target =  -20;//-15;
          now_car_state.diff_v = 0;
          if(((infrared_value(3) & 0x80) == 0x80) && (now_car_state.x_left<-25)) //再次检测到T字型 则改变状态
          {
            now_car_state.v_target =0;
            now_car_state.diff_v = 0 ;
            now_car_state.state = 2;
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
          }          
        }
        else if(now_car_state.task == 2)//侧方停车 循迹完成 继续前进一定距离
        {
          now_car_state.v_target = 20;
          //now_car_state.diff_v = 0;
          uint8_t dis_temp = infrared_value(3);
          if(((dis_temp & 0x80) == 0x00) )//如果读到的距边线的距离不为0，则进行距离的更新，否则不进行距离更新，防止出现黑线在两个传感器中间的情况
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
            now_car_state.x_left = 0; //在开始左转之前将位移值清零
          }
        }
      }
      else if( now_car_state.state == 0) // 循迹模式
      {
        now_car_state.v_target = 15;
        uint8_t dis_temp = infrared_value(3);

        if(((dis_temp & 0x80) == 0x00) )//如果读到的距边线的距离不为0，则进行距离的更新，否则不进行距离更新，防止出现黑线在两个传感器中间的情况
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
        else if((dis_temp & 0x80) == 0x80 ) //检测到T字型 改变状态 
        {
          if(judge_open_flag == 3 ) // 已经开启过了超声波，此时碰到的T字形不是第一个,也就是经过了一个车库 judge == 1 表示车位空闲 可以进行停车
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


//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    UART1_DataGet(rx_buf1);
    HAL_UART_AbortReceive_IT(&huart1);//终止正在进行的Rx传输
    HAL_UART_Receive_IT(&huart1,rx_buf1,2);//开启新一轮串口 等待2个字节数据 并写入
  }

  if(huart == &huart3)
  {
    UART1_DataGet(rx_buf3);
    HAL_UART_AbortReceive_IT(&huart3);//终止正在进行的Rx传输
    HAL_UART_Receive_IT(&huart3,rx_buf3,2);//开启新一轮串口 等待2个字节数据 并写入
  }
}

void UART1_DataGet(uint8_t * databuf)
{
  if(*databuf == 'c')
  {
    Control_Flag = 1;
    if(*(databuf+1) == 'l' ) // 左转
    {
      // Wheel_Drive(1,200);
      // Wheel_Drive(2,600);
      now_car_state.v_target = 20;
      now_car_state.diff_v = 5;
      // init_pidStruct(40, 0.8, 0.0,0, 0x0,  &Velocity_pidStruct_left);
      // init_pidStruct(20, 0.8, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'r' ) // 右转
    {
      // Wheel_Drive(1,600);
      // Wheel_Drive(2,200);
      now_car_state.v_target = 20;
      now_car_state.diff_v = -5;
      //now_car_state.diff_v += 5;
    }
    else if(*(databuf+1) == 'f' ) // 前进
    {
      // Wheel_Drive(1,400);
      // Wheel_Drive(2,400);
      now_car_state.v_target = 20;
      now_car_state.diff_v = 0;
      init_pidStruct(25, 0.8, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.7, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'b' ) // 后退
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
    else if(*(databuf+1) == 's' ) // 停止
    {
      // Wheel_Drive(1,0);
      // Wheel_Drive(2,0);
      now_car_state.v_target = 0;
      now_car_state.diff_v = 0;
      init_pidStruct(25, 0.7, 0.0,1, 0x0,  &Velocity_pidStruct_left);
      init_pidStruct(25, 0.9, 0.0, 1, 0x0,&Velocity_pidStruct_right);
    }
    else if(*(databuf+1) == 'c' ) // 转圈
    {
      // Wheel_Drive(1,-300);
      // Wheel_Drive(2,300);
      now_car_state.v_target = 0;
      now_car_state.diff_v = 30;
    }

  }
  else if(*databuf == 't')//完成任务模式
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
{/*mode1代表车头单路，mode2代表车尾单路，mode3代表五路*/
  uint8_t value =0x00;//返回值 
  uint8_t num = 0;//5路检测到黑线的传感器数量
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
		distance=(__HAL_TIM_GetCounter(&htim4)*340)/2000.0;//因为计数器每加一是10us，转化为cm为单位*声速340*100/100000/2
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
