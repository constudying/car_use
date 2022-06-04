/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技TC264DA核心板
 【编    写】ZYF/chiusir
 【E-mail  】chiusir@163.com
 【软件版本】V1.1 版权所有，单位使用请先联系授权
 【最后更新】2020年10月28日
 【相关信息参考下列地址】
 【网    站】http:// www.lqist.cn
 【淘宝店铺】http:// longqiu.taobao.com
 ------------------------------------------------
 【dev.env.】AURIX Development Studio1.2.2及以上版本
 【Target 】 TC264DA/TC264D
 【Crystal】 20.000Mhz
 【SYS PLL】 200MHz
 ________________________________________________________________
 基于iLLD_1_0_1_11_0底层程序,

 使用例程的时候，建议采用没有空格的英文路径，
 除了CIF为TC264DA独有外，其它的代码兼容TC264D
 本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
 工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。
 ________________________________________________________________

 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
 *  备    注：TC264 有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_CCU6.h"

#include <CompilerTasking.h>
#include "LQ_PID.h"
#include "../APP/LQ_GPIO_LED.h"
#include "../APP/LQ_UART_Bluetooth.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_MotorServo.h"
#include "LQ_ImageProcess.h"
#include "LQ_TFT18.h"
#include "LQ_IIC_Gyro.h"

volatile sint16 ECPULSE1 = 0;          // 速度全局变量
volatile sint16 ECPULSE2 = 0;          // 速度全局变量

volatile float motor = 0;
volatile float servo_mid = 1430;//左偏极限：1580；右偏极限：1280
volatile sint32 RAllPulse = 0;          // 速度全局变量

//volatile double right_cs = 0;
//volatile double left_cs = 0;


volatile sint16 Target_Speed1 = 2000;          // 速度全局变量
volatile sint16 Target_Speed2 = 2000;          // 速度全局变量


IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);

/** CCU6中断CPU标号 */
const uint8 Ccu6IrqVectabNum[2] = {CCU60_VECTABNUM, CCU61_VECTABNUM};

/** CCU6中断优先级 */
const uint8 Ccu6IrqPriority[4] = {CCU60_CH0_PRIORITY, CCU60_CH1_PRIORITY, CCU61_CH0_PRIORITY, CCU61_CH1_PRIORITY};

/** CCU6中断服务函数地址 */
const void *Ccu6IrqFuncPointer[4] = {&CCU60_CH0_IRQHandler, &CCU60_CH1_IRQHandler, &CCU61_CH0_IRQHandler,
        &CCU61_CH1_IRQHandler};


extern pid_param_t LSpeed_PID;
extern pid_param_t RSpeed_PID;

/***************以上PID参数部分，有待依据不同道路特征以细化**********************/
volatile float mot_kp = 0;
volatile float mot_kd = 0;
volatile float mot_ki = 0;
volatile float dir_kp = 0;
volatile float dir_kd = 0;
volatile float dir_ki = 0;
volatile float dir_errorlas = 0;
volatile float dir_errorder = 0;
/***************以上PID参数部分，有待依据不同道路特征以细化**********************/



extern sint16 MotorDuty1;
extern sint16 MotorDuty2;
/***********************************************************************************************/
/********************************CCU6外部中断  服务函数******************************************/
/***********************************************************************************************/

/*************************************************************************
 *  函数名称：void CCU60_CH0_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU60_CH0使用的中断服务函数
 *************************************************************************/
void CCU60_CH0_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
//   IfxCpu_enableInterrupts();
// 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    //元素信息处理
    //CameraCar();
}

/*************************************************************************
 *  函数名称：void CCU60_CH1_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU60_CH1使用的中断服务函数
 *************************************************************************/
extern int data[4];
extern volatile int ios;
extern volatile int counter;
void CCU60_CH1_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t13PeriodMatch);



     /* 获取编码器值 */
    ECPULSE1 = -ENC_GetCounter(ENC2_InPut_P33_7); // 左电机 母板上编码器1，小车前进为负值
    ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); // 右电机 母板上编码器2，小车前进为正值

    float tar_speed_tim = 0;

    tar_speed_tim = (float)((float)motor/25 - 12);


//    printf("%.2f, %.2f\n",(float)(ECPULSE1),(float)(ECPULSE2));




//    printf("image:0 ,19200 ,120 ,160 ,Format_JPG\n");//“图像：IMG_ID， IMG_SIZE， IMG_WIDTH， IMG_HEIGHT， IMG_FORMAT\n”
////    printf(“image:%d,%d,%d,%d,%d,%d\n”,
////    IMG_ID, // 此ID用于标识不同图片通道
////    IMG_SIZE, // 图片数据大小
////    IMG_WIDTH, // 图片宽度
////    IMG_HEIGHT, // 图片高度
////           IMG_FORMAT   // 图片格式
////           );
//    printf()
    /**************************以下为输出保护*****************************************/
           int l_block = 0;
           int r_block = 0;
           unsigned char block_flag = 0;

           if( (l_block - ECPULSE1 > 55 && ECPULSE1 < 20) || (r_block - ECPULSE2 > 55 && ECPULSE2 < 20) )
           {
               block_flag = 1;
           }
           if(abs(ECPULSE1) > 200 || abs(ECPULSE2) > 200)
               block_flag = 1;

           l_block = ECPULSE1;
           r_block = ECPULSE2;
    /**************************以上为输出保护*****************************************/



    float w_error = 0;//道路中线获取error
    float w_error_last = 0;
    float g_error = 0;//陀螺仪角速度偏差，暂定x轴一个
    float z_error = 0;//总error

//    /**************************以下陀螺仪偏差********************************/
//    signed short gyro_data[6] = {0};//0-x,1-y(车水平面，前进方向为x轴正向，左边为y轴正向),2-z(车身俯视逆时针为正);
//    if(!MPU_Get_Raw_data(&gyro_data[0] ,&gyro_data[1] ,&gyro_data[2] ,&gyro_data[3] ,&gyro_data[4] ,&gyro_data[5]))
//        g_error = gyro_data[2];
//    else g_error = 0;
//    /**************************以下陀螺仪偏差********************************/

//    w_error = (float)(79-midline[24]);//+ 左边；-右边
//    z_error = mot_kp*w_error + mot_kd*(w_error - w_error_last) - g_error;

    float l_error = 0;
    float l_error_last = 0;
    float l_error_int = 0;

    float r_error = 0;
    float r_error_last= 0;
    float r_error_int = 0;


    //电机采用位置式pid
    r_error = (float)((float)tar_speed_tim - (float)ECPULSE2);
    r_error_int += r_error;
    r_Duty = mot_kp * r_error + mot_ki * r_error_int + mot_kd * (r_error - r_error_last);
    r_error_last = r_error;

    l_error = (float)((float)tar_speed_tim - (float)ECPULSE1);
    l_error_int += l_error;
    l_Duty = mot_kp * l_error + mot_ki * l_error_int + mot_kd * (l_error - l_error_last);
    l_error_last = l_error;

//       //差速处理，差速比例（/10）可自己修改
//       if(ServoDuty > 0)
//       {
//           r_Duty = 1000 - ServoDuty/10;
//           l_Duty = 1000 ;
//       }
//       else
//       {
//           r_Duty = 1000 ;
//           l_Duty = 1000 + ServoDuty/10;
//       }


       //电机限幅
       if(r_Duty > 150)r_Duty = 150;else if(r_Duty < -150)r_Duty = -150;
       if(l_Duty > 150)l_Duty = 150;else if(l_Duty < -150)l_Duty = -150;
//       if(LSpeed_PID.out > 8000)LSpeed_PID.out = 1200;else if(LSpeed_PID.out < -8000)LSpeed_PID.out = -1200;
//       if(RSpeed_PID.out > 8000)RSpeed_PID.out = 1200;else if(RSpeed_PID.out < -8000)RSpeed_PID.out = -1200;


//       printf(" %.1f , %.1f , %.1f , %.0f , %.2f, %.2f\n",mot_kp,mot_kd,mot_ki,motor,(float)(ECPULSE1),(float)(ECPULSE2));

       printf(" %d , %d , %d , %d , %d , %d \n",ios,counter,data[0],data[1],data[2],data[3]);

/**************************以下为舵机模块*****************************************/
       float servo_duty = 0;

       if(servo_duty>150+1430) servo_duty = 150+1430;
       if(servo_duty<1430-150) servo_duty = 1430-150;

/**************************以上为舵机模块*****************************************/


       if(block_flag == 1)
       {
           MotorCtrl(0,0);
       }
       else
           MotorCtrl((-(25 * (r_Duty + 12))) ,25 * (l_Duty + 12));

}

/*************************************************************************
 *  函数名称：void CCU61_CH0_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU61_CH0使用的中断服务函数
 *************************************************************************/
void CCU61_CH0_IRQHandlerXXXXXXXXXXXXXXX (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    /* 获取编码器值 */
}



void CCU61_CH0_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    /* 获取编码器值 */
    ECPULSE1 = ENC_GetCounter(ENC2_InPut_P33_7); // 左电机 母板上编码器1，小车前进为负值
    ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); // 右电机 母板上编码器2，小车前进为正值
    RAllPulse += ECPULSE2;                       //
}
/*************************************************************************
 *  函数名称：void CCU61_CH1_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU61_CH1使用的中断服务函数
 *************************************************************************/
void CCU61_CH1_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* 用户代码 */
    LED_Ctrl(LED0, RVS);        // 电平翻转,LED闪烁
}

/*************************************************************************
 *  函数名称：CCU6_InitConfig CCU6
 *  功能说明：定时器周期中断初始化
 *  参数说明：ccu6    ： ccu6模块            CCU60 、 CCU61
 *  参数说明：channel ： ccu6模块通道  CCU6_Channel0 、 CCU6_Channel1
 *  参数说明：us      ： ccu6模块  中断周期时间  单位us
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：    CCU6_InitConfig(CCU60, CCU6_Channel0, 100);  // 100us进入一次中断
 *************************************************************************/
void CCU6_InitConfig (CCU6_t ccu6, CCU6_Channel_t channel, uint32 us)
{
    IfxCcu6_Timer_Config timerConfig;

    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);

    uint8 Index = ccu6 * 2 + channel;

    uint32 period = 0;

    uint64 clk = 0;

    /* 关闭中断 */
    boolean interrupt_state = disableInterrupts();

    IfxCcu6_Timer_initModuleConfig(&timerConfig, module);

    clk = IfxScuCcu_getSpbFrequency();

    /* 设置时钟频率  */
    uint8 i = 0;
    while (i++ < 16)
    {
        period = (uint32) (clk * us / 1000000);
        if (period < 0xffff)
        {
            break;
        }
        else
        {
            clk = clk / 2;
        }
    }
    switch (channel)
    {
        case CCU6_Channel0 :
            timerConfig.timer = IfxCcu6_TimerId_t12;
            timerConfig.interrupt1.source = IfxCcu6_InterruptSource_t12PeriodMatch;
            timerConfig.interrupt1.serviceRequest = IfxCcu6_ServiceRequest_1;
            timerConfig.base.t12Frequency = (float) clk;
            timerConfig.base.t12Period = period;                                  // 设置定时中断
            timerConfig.clock.t12countingInputMode = IfxCcu6_CountingInputMode_internal;
            timerConfig.timer12.counterValue = 0;
            timerConfig.interrupt1.typeOfService = Ccu6IrqVectabNum[ccu6];
            timerConfig.interrupt1.priority = Ccu6IrqPriority[Index];
            break;

        case CCU6_Channel1 :
            timerConfig.timer = IfxCcu6_TimerId_t13;
            timerConfig.interrupt2.source = IfxCcu6_InterruptSource_t13PeriodMatch;
            timerConfig.interrupt2.serviceRequest = IfxCcu6_ServiceRequest_2;
            timerConfig.base.t13Frequency = (float) clk;
            timerConfig.base.t13Period = period;
            timerConfig.clock.t13countingInputMode = IfxCcu6_CountingInputMode_internal;
            timerConfig.timer13.counterValue = 0;
            timerConfig.interrupt2.typeOfService = Ccu6IrqVectabNum[ccu6];
            timerConfig.interrupt2.priority = Ccu6IrqPriority[Index];
            break;
    }

    timerConfig.trigger.t13InSyncWithT12 = FALSE;

    IfxCcu6_Timer Ccu6Timer;

    IfxCcu6_Timer_initModule(&Ccu6Timer, &timerConfig);

    IfxCpu_Irq_installInterruptHandler((void*) Ccu6IrqFuncPointer[Index], Ccu6IrqPriority[Index]);          // 配置中断函数和中断号

    restoreInterrupts(interrupt_state);

    IfxCcu6_Timer_start(&Ccu6Timer);
}

/*************************************************************************
 *  函数名称：CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  功能说明：停止CCU6通道中断
 *  参数说明：ccu6    ： ccu6模块            CCU60 、 CCU61
 *  参数说明：channel ： ccu6模块通道  CCU6_Channel0 、 CCU6_Channel1
 *  函数返回：无
 *  修改时间：2020年5月6日
 *  备    注：
 *************************************************************************/
void CCU6_DisableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_disableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

/*************************************************************************
 *  函数名称：CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  功能说明：使能CCU6通道中断
 *  参数说明：ccu6    ： ccu6模块            CCU60 、 CCU61
 *  参数说明：channel ： ccu6模块通道  CCU6_Channel0 、 CCU6_Channel1
 *  函数返回：无
 *  修改时间：2020年5月6日
 *  备    注：
 *************************************************************************/
void CCU6_EnableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_enableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

