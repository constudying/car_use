/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】zyf/chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2020年4月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Hightec4.9.3/Tasking6.3及以上版本
【Target 】 TC264DA
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
基于iLLD_1_0_1_11_0底层程序
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


void CameraCar(void);//检测函数

void Shin(void);
uint8_t shin_feature(uint8_t imageInput[LCDH][LCDW], uint8_t image_l[80], uint8_t image_r[80], uint8_t image_m[80]);
//左右边线弧形判断
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num,uint8_t* index);

//丢线处理
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t mode, uint8_t lineIndex);

//显示
void TFT_Show_Camera_Info(void);

void track_init(void);

void TFT_RoadSide(unsigned char *l_line,unsigned char *r_line,unsigned char *m_line);

//获取上下边线
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW]);

//获取左右边线
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2]);

//重新获取左/右边线
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t status);

//重新获取上/下边线
void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[2][LCDW], uint8_t status);

//补线处理
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);


#endif /* SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_ */
