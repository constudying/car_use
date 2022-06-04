/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��zyf/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
����iLLD_1_0_1_11_0�ײ����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_
#define SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_
#include "include.h"
#include <Platform_Types.h>
#include <stdio.h>
#include <stdint.h>
#include "LQ_CAMERA.h"

extern uint8_t UpdowmSide[2][LCDW];
extern uint8_t ImageSide[LCDH][2];

extern uint8_t rightline[80];
extern uint8_t leftline[80];
extern uint8_t midline[80];


void CameraCar(void);//��⺯��

void Shin(void);
uint8_t shin_feature(uint8_t imageInput[LCDH][LCDW], uint8_t image_l[80], uint8_t image_r[80], uint8_t image_m[80]);
//���ұ��߻����ж�
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num,uint8_t* index);

//���ߴ���
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t mode, uint8_t lineIndex);

//��ʾ
void TFT_Show_Camera_Info(void);

void track_init(void);

void TFT_RoadSide(unsigned char *l_line,unsigned char *r_line,unsigned char *m_line);

//��ȡ���±���
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW]);

//��ȡ���ұ���
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2]);

//���»�ȡ��/�ұ���
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t status);

//���»�ȡ��/�±���
void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[2][LCDW], uint8_t status);

//���ߴ���
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);


#endif /* SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_ */
