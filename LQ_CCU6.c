/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 ��ƽ    ̨�������������ܿƼ�TC264DA���İ�
 ����    д��ZYF/chiusir
 ��E-mail  ��chiusir@163.com
 ������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
 �������¡�2020��10��28��
 �������Ϣ�ο����е�ַ��
 ����    վ��http:// www.lqist.cn
 ���Ա����̡�http:// longqiu.taobao.com
 ------------------------------------------------
 ��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
 ��Target �� TC264DA/TC264D
 ��Crystal�� 20.000Mhz
 ��SYS PLL�� 200MHz
 ________________________________________________________________
 ����iLLD_1_0_1_11_0�ײ����,

 ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
 ����CIFΪTC264DA�����⣬�����Ĵ������TC264D
 ����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
 ������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
 ________________________________________________________________

 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
 *  ��    ע��TC264 ������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
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

volatile sint16 ECPULSE1 = 0;          // �ٶ�ȫ�ֱ���
volatile sint16 ECPULSE2 = 0;          // �ٶ�ȫ�ֱ���

volatile float motor = 0;
volatile float servo_mid = 1430;//��ƫ���ޣ�1580����ƫ���ޣ�1280
volatile sint32 RAllPulse = 0;          // �ٶ�ȫ�ֱ���

//volatile double right_cs = 0;
//volatile double left_cs = 0;


volatile sint16 Target_Speed1 = 2000;          // �ٶ�ȫ�ֱ���
volatile sint16 Target_Speed2 = 2000;          // �ٶ�ȫ�ֱ���


IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);

/** CCU6�ж�CPU��� */
const uint8 Ccu6IrqVectabNum[2] = {CCU60_VECTABNUM, CCU61_VECTABNUM};

/** CCU6�ж����ȼ� */
const uint8 Ccu6IrqPriority[4] = {CCU60_CH0_PRIORITY, CCU60_CH1_PRIORITY, CCU61_CH0_PRIORITY, CCU61_CH1_PRIORITY};

/** CCU6�жϷ�������ַ */
const void *Ccu6IrqFuncPointer[4] = {&CCU60_CH0_IRQHandler, &CCU60_CH1_IRQHandler, &CCU61_CH0_IRQHandler,
        &CCU61_CH1_IRQHandler};


extern pid_param_t LSpeed_PID;
extern pid_param_t RSpeed_PID;

/***************����PID�������֣��д����ݲ�ͬ��·������ϸ��**********************/
volatile float mot_kp = 0;
volatile float mot_kd = 0;
volatile float mot_ki = 0;
volatile float dir_kp = 0;
volatile float dir_kd = 0;
volatile float dir_ki = 0;
volatile float dir_errorlas = 0;
volatile float dir_errorder = 0;
/***************����PID�������֣��д����ݲ�ͬ��·������ϸ��**********************/



extern sint16 MotorDuty1;
extern sint16 MotorDuty2;
/***********************************************************************************************/
/********************************CCU6�ⲿ�ж�  ������******************************************/
/***********************************************************************************************/

/*************************************************************************
 *  �������ƣ�void CCU60_CH0_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU60_CH0ʹ�õ��жϷ�����
 *************************************************************************/
void CCU60_CH0_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
//   IfxCpu_enableInterrupts();
// ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */
    //Ԫ����Ϣ����
    //CameraCar();
}

/*************************************************************************
 *  �������ƣ�void CCU60_CH1_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU60_CH1ʹ�õ��жϷ�����
 *************************************************************************/
extern int data[4];
extern volatile int ios;
extern volatile int counter;
void CCU60_CH1_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t13PeriodMatch);



     /* ��ȡ������ֵ */
    ECPULSE1 = -ENC_GetCounter(ENC2_InPut_P33_7); // ���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
    ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); // �ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ

    float tar_speed_tim = 0;

    tar_speed_tim = (float)((float)motor/25 - 12);


//    printf("%.2f, %.2f\n",(float)(ECPULSE1),(float)(ECPULSE2));




//    printf("image:0 ,19200 ,120 ,160 ,Format_JPG\n");//��ͼ��IMG_ID�� IMG_SIZE�� IMG_WIDTH�� IMG_HEIGHT�� IMG_FORMAT\n��
////    printf(��image:%d,%d,%d,%d,%d,%d\n��,
////    IMG_ID, // ��ID���ڱ�ʶ��ͬͼƬͨ��
////    IMG_SIZE, // ͼƬ���ݴ�С
////    IMG_WIDTH, // ͼƬ���
////    IMG_HEIGHT, // ͼƬ�߶�
////           IMG_FORMAT   // ͼƬ��ʽ
////           );
//    printf()
    /**************************����Ϊ�������*****************************************/
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
    /**************************����Ϊ�������*****************************************/



    float w_error = 0;//��·���߻�ȡerror
    float w_error_last = 0;
    float g_error = 0;//�����ǽ��ٶ�ƫ��ݶ�x��һ��
    float z_error = 0;//��error

//    /**************************����������ƫ��********************************/
//    signed short gyro_data[6] = {0};//0-x,1-y(��ˮƽ�棬ǰ������Ϊx���������Ϊy������),2-z(��������ʱ��Ϊ��);
//    if(!MPU_Get_Raw_data(&gyro_data[0] ,&gyro_data[1] ,&gyro_data[2] ,&gyro_data[3] ,&gyro_data[4] ,&gyro_data[5]))
//        g_error = gyro_data[2];
//    else g_error = 0;
//    /**************************����������ƫ��********************************/

//    w_error = (float)(79-midline[24]);//+ ��ߣ�-�ұ�
//    z_error = mot_kp*w_error + mot_kd*(w_error - w_error_last) - g_error;

    float l_error = 0;
    float l_error_last = 0;
    float l_error_int = 0;

    float r_error = 0;
    float r_error_last= 0;
    float r_error_int = 0;


    //�������λ��ʽpid
    r_error = (float)((float)tar_speed_tim - (float)ECPULSE2);
    r_error_int += r_error;
    r_Duty = mot_kp * r_error + mot_ki * r_error_int + mot_kd * (r_error - r_error_last);
    r_error_last = r_error;

    l_error = (float)((float)tar_speed_tim - (float)ECPULSE1);
    l_error_int += l_error;
    l_Duty = mot_kp * l_error + mot_ki * l_error_int + mot_kd * (l_error - l_error_last);
    l_error_last = l_error;

//       //���ٴ������ٱ�����/10�����Լ��޸�
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


       //����޷�
       if(r_Duty > 150)r_Duty = 150;else if(r_Duty < -150)r_Duty = -150;
       if(l_Duty > 150)l_Duty = 150;else if(l_Duty < -150)l_Duty = -150;
//       if(LSpeed_PID.out > 8000)LSpeed_PID.out = 1200;else if(LSpeed_PID.out < -8000)LSpeed_PID.out = -1200;
//       if(RSpeed_PID.out > 8000)RSpeed_PID.out = 1200;else if(RSpeed_PID.out < -8000)RSpeed_PID.out = -1200;


//       printf(" %.1f , %.1f , %.1f , %.0f , %.2f, %.2f\n",mot_kp,mot_kd,mot_ki,motor,(float)(ECPULSE1),(float)(ECPULSE2));

       printf(" %d , %d , %d , %d , %d , %d \n",ios,counter,data[0],data[1],data[2],data[3]);

/**************************����Ϊ���ģ��*****************************************/
       float servo_duty = 0;

       if(servo_duty>150+1430) servo_duty = 150+1430;
       if(servo_duty<1430-150) servo_duty = 1430-150;

/**************************����Ϊ���ģ��*****************************************/


       if(block_flag == 1)
       {
           MotorCtrl(0,0);
       }
       else
           MotorCtrl((-(25 * (r_Duty + 12))) ,25 * (l_Duty + 12));

}

/*************************************************************************
 *  �������ƣ�void CCU61_CH0_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU61_CH0ʹ�õ��жϷ�����
 *************************************************************************/
void CCU61_CH0_IRQHandlerXXXXXXXXXXXXXXX (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */
    /* ��ȡ������ֵ */
}



void CCU61_CH0_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */
    /* ��ȡ������ֵ */
    ECPULSE1 = ENC_GetCounter(ENC2_InPut_P33_7); // ���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
    ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); // �ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ
    RAllPulse += ECPULSE2;                       //
}
/*************************************************************************
 *  �������ƣ�void CCU61_CH1_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU61_CH1ʹ�õ��жϷ�����
 *************************************************************************/
void CCU61_CH1_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* �û����� */
    LED_Ctrl(LED0, RVS);        // ��ƽ��ת,LED��˸
}

/*************************************************************************
 *  �������ƣ�CCU6_InitConfig CCU6
 *  ����˵������ʱ�������жϳ�ʼ��
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  ����˵����us      �� ccu6ģ��  �ж�����ʱ��  ��λus
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��    CCU6_InitConfig(CCU60, CCU6_Channel0, 100);  // 100us����һ���ж�
 *************************************************************************/
void CCU6_InitConfig (CCU6_t ccu6, CCU6_Channel_t channel, uint32 us)
{
    IfxCcu6_Timer_Config timerConfig;

    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);

    uint8 Index = ccu6 * 2 + channel;

    uint32 period = 0;

    uint64 clk = 0;

    /* �ر��ж� */
    boolean interrupt_state = disableInterrupts();

    IfxCcu6_Timer_initModuleConfig(&timerConfig, module);

    clk = IfxScuCcu_getSpbFrequency();

    /* ����ʱ��Ƶ��  */
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
            timerConfig.base.t12Period = period;                                  // ���ö�ʱ�ж�
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

    IfxCpu_Irq_installInterruptHandler((void*) Ccu6IrqFuncPointer[Index], Ccu6IrqPriority[Index]);          // �����жϺ������жϺ�

    restoreInterrupts(interrupt_state);

    IfxCcu6_Timer_start(&Ccu6Timer);
}

/*************************************************************************
 *  �������ƣ�CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  ����˵����ֹͣCCU6ͨ���ж�
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  �������أ���
 *  �޸�ʱ�䣺2020��5��6��
 *  ��    ע��
 *************************************************************************/
void CCU6_DisableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_disableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

/*************************************************************************
 *  �������ƣ�CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  ����˵����ʹ��CCU6ͨ���ж�
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  �������أ���
 *  �޸�ʱ�䣺2020��5��6��
 *  ��    ע��
 *************************************************************************/
void CCU6_EnableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_enableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

