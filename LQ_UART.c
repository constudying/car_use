/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��ZYF/chiusir
��E-mail  ��chiusir@163.com
�������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��10��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
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
ASC�첽����ͨ�ţ�������ΪUART������LIN����ʹ�ã�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_UART.h"
#include "stdio.h"
#include <Compilers.h>
#include <CompilerTasking.h>
#include <stddef.h>
#include "stdio.h"
#include "LQ_GPIO.h"
#include "LQ_CCU6.h"

#define ASC_TX_BUFFER_SIZE 64        //���ͻ���������
#define ASC_RX_BUFFER_SIZE 64        //���ջ���������

//����ͨ�Žṹ��
IfxAsclin_Asc g_UartConfig[4];

static uint8 s_AscTxBuffer[4][ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 s_AscRxBuffer[4][ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

/* UART�ж� */
IFX_INTERRUPT(UART0_RX_IRQHandler, UART0_VECTABNUM, UART0_RX_PRIORITY);
IFX_INTERRUPT(UART1_RX_IRQHandler, UART1_VECTABNUM, UART1_RX_PRIORITY);
IFX_INTERRUPT(UART2_RX_IRQHandler, UART2_VECTABNUM, UART2_RX_PRIORITY);
IFX_INTERRUPT(UART3_RX_IRQHandler, UART3_VECTABNUM, UART3_RX_PRIORITY);
IFX_INTERRUPT(UART0_TX_IRQHandler, UART0_VECTABNUM, UART0_TX_PRIORITY);
IFX_INTERRUPT(UART1_TX_IRQHandler, UART1_VECTABNUM, UART1_TX_PRIORITY);
IFX_INTERRUPT(UART2_TX_IRQHandler, UART2_VECTABNUM, UART2_TX_PRIORITY);
IFX_INTERRUPT(UART3_TX_IRQHandler, UART3_VECTABNUM, UART3_TX_PRIORITY);
IFX_INTERRUPT(UART0_ER_IRQHandler, UART0_VECTABNUM, UART0_ER_PRIORITY);
IFX_INTERRUPT(UART1_ER_IRQHandler, UART1_VECTABNUM, UART1_ER_PRIORITY);
IFX_INTERRUPT(UART2_ER_IRQHandler, UART2_VECTABNUM, UART2_ER_PRIORITY);
IFX_INTERRUPT(UART3_ER_IRQHandler, UART3_VECTABNUM, UART3_ER_PRIORITY);


/** UART�ж�CPU��� */
const uint8 UartIrqVectabNum[4] = {UART0_VECTABNUM, UART1_VECTABNUM, UART2_VECTABNUM, UART3_VECTABNUM};

/** UART�ж����ȼ� */
const uint8 UartIrqPriority[12] = {UART0_RX_PRIORITY, UART0_TX_PRIORITY, UART0_ER_PRIORITY, UART1_RX_PRIORITY, UART1_TX_PRIORITY, UART1_ER_PRIORITY,
		                           UART2_RX_PRIORITY, UART2_TX_PRIORITY, UART2_ER_PRIORITY, UART3_RX_PRIORITY, UART3_TX_PRIORITY, UART3_ER_PRIORITY};

/** UART�жϷ�������ַ */
const void *UartIrqFuncPointer[12] = {&UART0_RX_IRQHandler, &UART0_TX_IRQHandler, &UART0_ER_IRQHandler,
									   &UART1_RX_IRQHandler, &UART1_TX_IRQHandler, &UART1_ER_IRQHandler,
									   &UART2_RX_IRQHandler, &UART2_TX_IRQHandler, &UART2_ER_IRQHandler,
									   &UART3_RX_IRQHandler, &UART3_TX_IRQHandler, &UART3_ER_IRQHandler,};

/*************************************************************************
*  �������ƣ�void UART0_RX_IRQHandler(void)
*  ����˵����UART0_RX_IRQHandler�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��
*************************************************************************/
extern volatile float motor;
int data[4]={0,0,0,0};//ĩλ���ݼ���
volatile int ios = 0;
volatile int counter = 0;
void tune_para(int *ch,int m);
void UART0_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[0]);


	if(ios == 0)
	{
	    ios = ((int)UART_GetChar(UART0) - 48);
	    if(ios>4 || ios<1)
	        ios = 0;
	}
	else
	{
	    data[counter] = ((int)UART_GetChar(UART0) - 48);
	    if(data[counter] <= 9 && data[counter] >= 0)
	    {
	        counter++;
	        if(counter == 4)
	        {
	            tune_para((int *)data,ios);
	            counter = 0;
	            ios = 0;
	        }
	    }
	    else
	        data[counter] = 0;
	}


//    static int count = 0;
//    float numt = 0;
//
//	data[count] = UART_GetChar(UART0);
//
////    UART_GetBuff(UART0 ,(unsigned char*)data ,10);
//
//	count += 1;
//
//	if(count == 10)
//	{
//	    data[count] = UART_GetCount(UART0);
//
//	    //�����ٶ�motor
//	    numt = (float)((float)((int)data[0] - 48)*1000 + (float)((int)data[1] - 48)*100 + (float)((int)data[2] - 48)*10 + (float)((int)data[3] - 48)*1);
//	    motor = numt;
//
//	    //���pid����������
//	    numt = (float)((float)((float)((int)data[4] - 48)*1 + (float)((int)data[5] - 48)*0.1));
//	    if(numt == (unitp+0.0) || numt == (unitp+0.1) || numt == (unitp+0.2) || numt ==(unitp+0.3) || numt ==(unitp+0.4)
//	            || numt == (unitp+0.5) || numt == (unitp+0.6) || numt == (unitp+0.7) || numt == (unitp+0.8) || numt == (unitp+0.9))
//	        mot_kp = numt;
//	    numt = (float)((float)((float)((int)data[6] - 48)*1 + (float)((int)data[7] - 48)*0.1));
//	    if(numt == (unitd+0.0) || numt == (unitd+0.1) || numt == (unitd+0.2) || numt ==(unitd+0.3) || numt ==(unitd+0.4)
//	            || numt == (unitd+0.5) || numt == (unitd+0.6) || numt == (unitd+0.7) || numt == (unitd+0.8) || numt == (unitd+0.9))
//	        mot_kd = numt;
//	    numt = (float)((float)((float)((int)data[8] - 48)*1 + (float)((int)data[9] - 48)*0.1));
//	    if(numt == (uniti+0.0) || numt == (uniti+0.1) || numt == (uniti+0.2) || numt ==(uniti+0.3) || numt ==(uniti+0.4)
//	            || numt == (uniti+0.5) || numt == (uniti+0.6) || numt == (uniti+0.7) || numt == (uniti+0.8) || numt == (uniti+0.9))
//	        mot_ki = numt;
//
//	    count = 0;
//	}

	/* �û����� */
//	UART_PutChar(UART0, UART_GetChar(UART0));
}


void tune_para(int *ch,int m)
{
    switch(m)
    {
        case 1://mot_kp
           mot_kp = (float)((float)ch[0]*10 + (float)ch[1]*1 + (float)ch[2]*0.1 + (float)ch[3]*0.01);
           break;
        case 2://mot_kd
            mot_kd = (float)((float)ch[0]*10 + (float)ch[1]*1 + (float)ch[2]*0.1 + (float)ch[3]*0.01);
            break;
        case 3://mot_ki
            mot_ki = (float)((float)ch[0]*10 + (float)ch[1]*1 + (float)ch[2]*0.1 + (float)ch[3]*0.01);
            break;
        case 4://motor
            motor = (float)(ch[0]*1000 + ch[1]*100 + ch[2]*10 + ch[3]*1);
            break;
        default:
            break;
    }
}


void UART0_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[0]);

//	printf("%.2f, %.2f\n",(float)(ECPULSE1),(float)(ECPULSE2));
	//  char letter = 0;
	//  unsigned char re_flag = 0;
	//  unsigned char data[4] = {0};
	//
	////    UART_GetBuff(UART0 , (unsigned char *)data, 6);
	//
	//  letter = UART_GetChar(UART0);
	//  if(letter == "A")
	//  {
	//      if(re_flag == 0)
	//      {
	//          re_flag = 1;
	//      }
	//      else if(re_flag == 1)
	//      {
	//          re_flag = 0;
	//      }
	//  }
	//  else if(re_flag == 1 && UART_GetChar(UART0) != "A")
	//  {
	//      counter++;
	//      if(counter == 4)
	//      {
	//          motor = (float)((int)data[0]*1000 + (int)data[1]*100 + (int)data[2]*10 + (int)data[3]*1);
	//          counter = 0;
	//          re_flag = 0;
	//      }
	//  }

	/* �û����� */
}

void UART0_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[0]);

	/* �û����� */

}

void UART1_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[1]);

	/* �û����� */
	UART_PutChar(UART1, UART_GetChar(UART1));
}

void UART1_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[1]);

	/* �û����� */
}

void UART1_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[1]);

	/* �û����� */
}

void UART2_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[2]);

	/* �û����� */
	UART_PutChar(UART2, UART_GetChar(UART2));
}

void UART2_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[2]);

	/* �û����� */
}

void UART2_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[2]);

	/* �û����� */
}

void UART3_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[3]);

	/* �û����� */
	UART_PutChar(UART3, UART_GetChar(UART3));
}

void UART3_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[3]);

	/* �û����� */
}

void UART3_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[3]);

	/* �û����� */
}

/*************************************************************************
*  �������ƣ�void UART_InitConfig(UART_RX_t RxPin, UART_TX_t TxPin, unsigned long baudrate)
*  ����˵��������ģ���ʼ��
*  ����˵����
  * @param    RxPin   �� ���ڽ��չܽ�
  * @param    TxPin   �� ���ڷ��͹ܽ�
  * @param    baudrate�� ������
*  �������أ��ֽ�
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);   //��ʼ������0 ������ 115200 ����żУ�� 1ֹͣλ ʹ�ùܽ�P14_0 P14_1
*************************************************************************/
void UART_InitConfig(UART_RX_t RxPin, UART_TX_t TxPin, unsigned long baudrate)
{
	int i,j;
	//�ر�CPU�ж�
	IfxCpu_disableInterrupts();

	Ifx_P *portRx = PIN_GetModule(RxPin);
	uint8 pinIndexRx = PIN_GetIndex(RxPin);

	Ifx_P *portTx = PIN_GetModule(TxPin);
	uint8 pinIndexTx = PIN_GetIndex(TxPin);

	IfxAsclin_Rx_In  * IfxAsclin_Rx = NULL_PTR;
	IfxAsclin_Tx_Out * IfxAsclin_Tx = NULL_PTR;

	for( i = 0; i < IFXASCLIN_PINMAP_NUM_MODULES; i++)
	{
		for( j = 0; j < IFXASCLIN_PINMAP_RX_IN_NUM_ITEMS; j++)
		{
			if(IfxAsclin_Rx_In_pinTable[i][j] == NULL_PTR)
			{

			}
			else if((unsigned long)portRx == (unsigned long)(IfxAsclin_Rx_In_pinTable[i][j]->pin.port) && pinIndexRx == IfxAsclin_Rx_In_pinTable[i][j]->pin.pinIndex)
			{
				IfxAsclin_Rx = IfxAsclin_Rx_In_pinTable[i][j];
			}
		}

		for(j = 0; j < IFXASCLIN_PINMAP_TX_OUT_NUM_ITEMS; j++)
		{
			if(IfxAsclin_Tx_Out_pinTable[i][j] == NULL_PTR)
			{

			}
			else if((unsigned long)portTx == (unsigned long)(IfxAsclin_Tx_Out_pinTable[i][j]->pin.port) && pinIndexTx == IfxAsclin_Tx_Out_pinTable[i][j]->pin.pinIndex)
			{
				IfxAsclin_Tx = IfxAsclin_Tx_Out_pinTable[i][j];
			}
		}
	}

	if(IfxAsclin_Rx->module != IfxAsclin_Tx->module)
	{
#pragma warning 557         // ���ξ���
		while (1);          // ��� RX��TX �Ƿ�ΪͬһUART
#pragma warning default     // �򿪾���
	}

	//�½��������ýṹ��
	IfxAsclin_Asc_Config ascConfig;

	//��ʼ��ģ��
	IfxAsclin_Asc_initModuleConfig(&ascConfig, IfxAsclin_Tx->module);

	ascConfig.baudrate.baudrate  = (float)baudrate;        //������
	ascConfig.frame.frameMode    = IfxAsclin_FrameMode_asc;//����֡ģʽ
	ascConfig.frame.dataLength   = IfxAsclin_DataLength_8; //���ݳ���
	ascConfig.frame.stopBit      = IfxAsclin_StopBit_1;    //ֹͣλ
	ascConfig.frame.shiftDir     = IfxAsclin_ShiftDirection_lsbFirst;//��λ����
	ascConfig.frame.parityBit    = FALSE;//����żУ��

	unsigned char uartNum = IfxAsclin_getIndex(IfxAsclin_Tx->module);

	//�ж����ȼ�����
	ascConfig.interrupt.rxPriority = UartIrqPriority[uartNum * 3];
	ascConfig.interrupt.txPriority = UartIrqPriority[uartNum * 3 + 1];
	ascConfig.interrupt.erPriority = UartIrqPriority[uartNum * 3 + 2];
	ascConfig.interrupt.typeOfService = UartIrqVectabNum[uartNum];

	//���պͷ���FIFO������
	ascConfig.txBuffer     = &s_AscTxBuffer[uartNum][0];
	ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
	ascConfig.rxBuffer     = &s_AscRxBuffer[uartNum][0];
	ascConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;

	IfxAsclin_Asc_Pins pins =
	{
		NULL,                     IfxPort_InputMode_pullUp,        /* CTS pin not used */
		IfxAsclin_Rx,             IfxPort_InputMode_pullUp,        /* Rx pin */
		NULL,                     IfxPort_OutputMode_pushPull,     /* RTS pin not used */
		IfxAsclin_Tx,             IfxPort_OutputMode_pushPull,     /* Tx pin */
		IfxPort_PadDriver_cmosAutomotiveSpeed1
	};
	ascConfig.pins = &pins;

	//��������ṹ����Ԥ��Ĳ��������ģ��ĳ�ʼ��
	IfxAsclin_Asc_initModule(&g_UartConfig[uartNum], &ascConfig);

	//���գ����ͺʹ����ж�����
	IfxCpu_Irq_installInterruptHandler((void*)UartIrqFuncPointer[uartNum * 3],     UartIrqPriority[uartNum * 3]);
	IfxCpu_Irq_installInterruptHandler((void*)UartIrqFuncPointer[uartNum * 3 + 1], UartIrqPriority[uartNum * 3 + 1]);
	IfxCpu_Irq_installInterruptHandler((void*)UartIrqFuncPointer[uartNum * 3 + 2], UartIrqPriority[uartNum * 3 + 2]);

	//����CPU�ж�
	IfxCpu_enableInterrupts();
}

/*************************************************************************
*  �������ƣ�void UART_PutChar(UART_t  uratn, char ch)
*  ����˵����UART�����ֽں���,ʹ��ǰ���ȳ�ʼ����Ӧ����
*  ����˵����uratn �� UART0 - UART3/ ch    �� Ҫ��ӡ���ַ�
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_PutChar(UART0, 'a');  //��ӡ�ַ�a
*************************************************************************/
void UART_PutChar(UART_t  uratn, char ch)
{
	IfxAsclin_Asc_blockingWrite(&g_UartConfig[uratn], ch);
}

/*************************************************************************
*  �������ƣ�void UART_PutStr(UART_t  uratn, char *str)
*  ����˵����UART�����ַ�������(�� NULL ֹͣ����),ʹ��ǰ���ȳ�ʼ����Ӧ����
*  ����˵����uratn �� UART0 - UART3/ str   �� Ҫ��ӡ���ַ�����ַ
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_PutStr(UART3, "123456789"); //����9���ֽ�
*************************************************************************/
void UART_PutStr(UART_t  uratn, char *str)
{
   while(*str)
    {
        UART_PutChar(uratn, *str++);
    }
}

/*************************************************************************
*  �������ƣ�void UART_PutBuff(UART_t  uratn, unsigned char *buff, unsigned long len)
*  ����˵����UART�����ֽں���,ʹ��ǰ���ȳ�ʼ����Ӧ����
*  ����˵����
* @param    uratn �� UART0 - UART3
* @param    buff  �� Ҫ��ӡ���ַ�����ַ
* @param    len   �� Ҫ��ӡ�ĳ���
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_PutBuff(UART4, "123456789",5);//ʵ�ʷ���5���ֽڡ�1����2����3����4����5��
*************************************************************************/
void UART_PutBuff(UART_t  uratn, unsigned char *buff, unsigned long len)
{
    while(len--)
    {
        UART_PutChar(uratn, *buff);
        buff++;
    }
}

/*************************************************************************
*  �������ƣ�Ifx_SizeT UART_GetCount(UART_t  uratn)
*  ����˵������ȡ ���ջ����� ��ŵ�ǰ�������ݸ���
*  ����˵���� uratn �� UART0 - UART3
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
uint8 UART_GetCount(UART_t  uratn)
{
	return (uint8)IfxAsclin_Asc_getReadCount(&g_UartConfig[uratn]);
}

/*************************************************************************
*  �������ƣ�char UART_GetChar(UART_t  uratn)
*  ����˵����UART��ȡ�ֽ� ʹ��ǰ���ȳ�ʼ����Ӧ���� ����ǰ��ȷ���н��յ����� �����ȴ����ݵ���
*  ����˵����uratn �� UART0 - UART3
*  �������أ���ȡ�ֽ�
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_GetChar(UART3); //����һ���ַ�
*************************************************************************/
char UART_GetChar(UART_t  uratn)
{
	uint8 data=0;
	Ifx_SizeT count = 1;

	/* ��ȡ���յ���һ���ֽ����� ע�����û�н��յ�����һֱ�ȴ� */
	IfxAsclin_Asc_read(&g_UartConfig[uratn], &data, &count, TIME_INFINITE);

	return 	data;
}

/*************************************************************************
*  �������ƣ�char UART_GetBuff(UART_t  uratn, unsigned char *data, unsigned char len)
*  ����˵��������һ�������ַ� ʹ��ǰ���ȳ�ʼ����Ӧ���� ����ǰ��ȷ���н��յ����� ��������ʧ��
*  ����˵����uratn �� UART0 - UART3
*  �������أ�0:��ȡ����  ���� ��ȡʧ��
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��ART_GetChar(UART0, data, 10); //����10���ַ�
*************************************************************************/
char UART_GetBuff(UART_t  uratn, unsigned char *data, unsigned char len)
{
	Ifx_SizeT count = len;

	if(UART_GetCount(uratn) < len)
	{
		return 1;  //�жϵ�ǰ���յ����������� ������ȡ����
	}

	/* ��ȡ���յ�����  */
	IfxAsclin_Asc_read(&g_UartConfig[uratn], data, &count, TIME_INFINITE);
	return 	0;
}



/*******************************************************************************
* Function Name  : _write
* Description    : Support Printf Function
* Input          : *buf: UART send Data.
*                  size: Data length
* Return         : size: Data length
*******************************************************************************/
int _write(int fd, char *buf, int size)
{
  int i;

  for(i=0; i<size; i++)
  {
    UART_PutChar(UART0, *buf++);
  }
  return size;
}

//int _write(int fd, char *buf, int size)
//{
//  int i;
//
//  for(i=0; i<size; i++)
//  {
//#if (DEBUG == DEBUG_UART1)
//    while (USART_GetFlagStatus(UART1, USART_FLAG_TC) == RESET);
//    UART_PutChar(UART1, *buf++);
//#elif (DEBUG == DEBUG_UART2)
//    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//    UART_PutChar(USART2, *buf++);
//#elif (DEBUG == DEBUG_UART3)
//    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
//    UART_PutChar(USART3, *buf++);
//#endif
//  }
//
//  return size;
//}






/////////////////////////////////////////////////////////////////////////////////////