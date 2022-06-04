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

#include "LQ_ImageProcess.h"

#include <Platform_Types.h>
#include <stdio.h>
#include "LQ_GPT12_ENC.h"
#include "../APP/LQ_CAMERA.h"
#include "../APP/LQ_GPIO_KEY.h"
#include "../APP/LQ_GPIO_LED.h"
#include "../APP/LQ_TFT18.h"
#include "../Driver/LQ_GPIO.h"
#include "../Driver/LQ_ADC.h"
#include "../Driver/LQ_CCU6.h"
#include "../Driver/LQ_STM.h"
#include "LQ_Inductor.h"
#include "LQ_MotorServo.h"
#include "LQ_PID.h"
#include "LQ_UART.h"

/**  @brief    转向误差  */
sint16 g_sSteeringError = 0;
/**  @brief    丢线标志位  */
uint8_t g_ucIsNoSide = 0;

/**  @brief    主跑行  */
#define ROAD_MAIN_ROW      40

/**  @brief    使用起始行  */
#define ROAD_START_ROW     115

/**  @brief    使用结束行  */
#define ROAD_END_ROW       10


/**  @brief    环岛标志位  */
uint8_t g_ucFlagRoundabout  = 0;

/**  @brief    十字标志位  */
uint8_t g_ucFlagCross  = 0;

/**  @brief    斑马线标志位  */
uint8_t g_ucFlagZebra  = 0;

/**  @brief    Y型岔口标志位  */
uint8_t g_ucFlagFork  = 0;
uint8_t g_ucForkNum  = 0;

/**  @brief    T型岔口标志位  */
uint8_t g_ucFlagT = 0;

pid_param_t BalDirgyro_PID;  // 方向PID

uint8_t Servo_P = 12;



char txt[30];
/*!
  * @brief    画边线
  *
  * @param
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/28 星期日
  */
void TFTSPI_BinRoadSide(uint8_t imageOut[LCDH][2])
{
    uint8_t i = 0;

    for(i = 0; i < ROAD_START_ROW; i++)
    {
        TFTSPI_Draw_Dot(imageOut[i][0], i, u16RED);

        TFTSPI_Draw_Dot(imageOut[i][1], i, u16GREEN);

    }

}

void TFTSPI_BinRoad_UpdownSide(uint8_t imageOut[2][LCDW])
{
    uint8_t i = 0;

    for(i = 0; i < LCDW; i++)
    {
        TFTSPI_Draw_Dot(i, imageOut[0][i], u16YELLOW);

        TFTSPI_Draw_Dot(i, imageOut[1][i], u16ORANGE);

    }

}

unsigned char stop_pos = 0;
void TFT_RoadSide(unsigned char *l_line,unsigned char *r_line,unsigned char *m_line)
{
    uint8_t i = 0;

    if(!stop_pos)
    {
        for(i = 110;i>30; i--)
        {
            TFTSPI_Draw_Dot(r_line[110-i] ,i ,u16RED);

            TFTSPI_Draw_Dot(l_line[110-i] ,i ,u16RED);

            TFTSPI_Draw_Dot(m_line[110-i] ,i ,u16BLACK);
        }
    }
    else
    {
        for(i = 110;i>stop_pos; i--)
        {
            TFTSPI_Draw_Dot(r_line[110-i] ,i ,u16RED);

            TFTSPI_Draw_Dot(l_line[110-i] ,i ,u16RED);

            TFTSPI_Draw_Dot(m_line[110-i] ,i ,u16BLACK);
        }
        stop_pos = 0;
    }

}

/*!
  * @brief    判断上边线是否单调
  * @param    X1 :起始X点
  * @param    X2 :终止X点              X1 < X2
  * @param    imageIn ： 边线数组
  *
  * @return   0：不单调or错误， 1：单调递增， 2：单调递减
  *
  * @note
  *
  * @see
  *
  * @date     2021/11/30 星期二
  */
uint8_t RoadUpSide_Mono(uint8_t X1, uint8_t X2, uint8_t imageIn[2][LCDW])
{
    uint8_t i = 0, num = 0;

    for(i = X1; i < X2-1; i++)
    {
        if(imageIn[0][i] >= imageIn[0][i+1])
            num++;
        else
            num = 0;
        if (num >= (X2-X1)*4/5)
            return 1;
    }

    for(i = X1; i < X2-1; i++)
    {
        if(imageIn[0][i] <= imageIn[0][i+1])
            num++;
        else
            num = 0;
        if (num >= (X2-X1)*4/5)
            return 2;
    }
    return 0;
}

/*!
  * @brief    判断是否是直道
  *
  * @param    image ： 二值图像信息
  *
  * @return   0：不是直道， 1：直道
  *
  * @note     思路：两边边线都单调
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2])
{
    uint8_t i = 0;
    uint8_t leftState = 0, rightState = 0;

    /* 左边线是否单调 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][0] + 5 < imageSide[i+1][0])
        {
            leftState = 1;
            break;
        }
    }

    /* 右边线是否单调 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][1] - 5 > imageSide[i+1][1])
        {
            rightState = 1;
            break;
        }
    }

    if(leftState == 1 && rightState == 1)
    {
        return 1;
    }

    return 0;
}


/*!
  * @brief    判断是否是斑马线
  *
  * @param    image ： 二值图像信息
  *
  * @return   0：不是， 1：是
  *
  * @note     思路：
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsZebra(uint8_t image[LCDH][LCDW], uint8_t *flag)
{
    int i = 0, j = 0;
    int count = 0;

    for(i = ROAD_MAIN_ROW - 30; i >  ROAD_MAIN_ROW + 30; i++)
    {
        for(j = 1; j < LCDW; j++)
        {
            if(image[i][j] == 1 && image[i][j-1] == 0)
            {
                count ++;
            }
        }
        if(count > 5)
        {
            *flag = 1;
            return 1;
        }
    }


    return 0;
}

/*!
  * @brief    判断是否是T字
  *
  * @param    imageSide ： 图像边线信息
  * @param    flag      ： T字状态信息
  *
  * @return   0：不是， 1：是
  *
  * @note     思路：上线0-80单调增， 80-159单调减 ，整体是一个大弧，右边线全丢，左边线115-50单调增
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsT(uint8_t imageUp[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t *flag)
{
    uint8_t i = 0;
    uint8_t errU1 = 0, errU2 = 0, errR1 = 0, errL1 = 0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t count = 0, num = 0, py;
    uint8_t index = 0;

    /* 检测左侧边线距离车头近的行丢线 --  */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][1] == 159)
            num++;
        if(num >= 130)
        {
            rightState = 1;//右为丢线
            break;
        }
    }

    for(i = ROAD_START_ROW-1; i > 20; i--)
    {
        if(imageSide[i][0] <= imageSide[i+1][0])
        {
            if(index < 4)index=0;
            count++;
        }else{
            if(count <15)count = 0;
            index++;
        }
        if(count >= 15 && index >= 4)
        {
            leftState = 1;   // 左hu标志
            break;
        }
    }
    errL1 = RoundaboutGetArc(imageSide, 1, 5, &py);    //左线有弧
    errR1 = RoundaboutGetArc(imageSide, 2, 5, &py);    //右线有弧
    errU1 = RoadUpSide_Mono(10, 70, imageUp);       //上单调增
    errU2 = RoadUpSide_Mono(80, 150, imageUp);     //上单调减
    if(rightState==1 && errU2==2 && errL1 == 1 )
    {
        *flag=1;
        return 1;
    }
    return 0;

}

uint8_t TProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageUp[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t *flag)
{
    uint8_t py, i, num=0;
    uint8_t errU1 = 0, errU2 = 0, errL1=0;
    switch(*flag)
    {
        case 1:
            //重新确定上边线
            Roundabout_Get_UpDowmSide(imageInput, imageUp, 1);
            errL1 = RoundaboutGetArc(imageSide, 1, 5, &py);    //左线有弧
            errU1 = RoadUpSide_Mono(10, 140, imageUp);      //上单调增

            if(errU1 == 2 && errL1 == 0)
                *flag = 2;

            //补线左转，增加转弯半径
            ImageAddingLine(imageSide, 1, 90, 30, 0, ROAD_START_ROW);
            break;

        case 2:
            errU2 = RoundaboutGetArc(imageSide, 2, 5, &py);//检查右边线是否有弧
            for(i = 0; i < 159; i ++)
            {
                if(imageUp[1][i] <= 118)
                    num++;
                if(num >= 140)
                    errU2 = 1;
            }
            if(errU2){
                Servo_P = 12;
                *flag = 0;
                break;
            }
            ImageAddingLine(imageSide, 2, 60, 30, 159, ROAD_START_ROW);//参数自行修改
            break;
    }
    return 0;
}

/*!
  * @brief    判断是否是十字
  *
  * @param    imageSide ： 图像边线信息
  * @param    flag      ： 十字状态信息
  *
  * @return   0：不是， 1：是
  *
  * @note     思路：两条边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 则证明有十字
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsCross(uint8_t imageSide[LCDH][2], uint8_t *flag)
{
    int i = 0;
    uint8_t errR = 0, errF = 0;
    uint8_t  rightState = 0, leftState = 0;
    int start[5] = {0, 0, 0, 0, 0}, end[5] = {0, 0, 0, 0, 0};
    uint8_t count = 0;
    uint8_t index = 0;

    /* 检测右侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][1] == 159)
            count++;
        else
        {
            if(count > 10 && index < 5)
            {
                start[index] = i + count;
                end[index]   = i;
                index++;
            }
            count = 0;
        }
    }
    if(index > 1)
    {
        if(end[0] - start[1] > 10)
            rightState = 1;
    }
    index = 0;

    /* 检测左侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][0] == 0)
            count++;
        else
        {
            if(count > 10 && index < 5)
            {
                start[index] = i + count;
                end[index]   = i;
                index++;
            }
            count = 0;
        }
    }
    if(index > 1)
    {
        if(end[0] - start[1] > 10)
            leftState = 1;
    }

    if(errR && errF)
    {
        count = 0;
        index = 0;
        //上线是否右突起
        for(i = 159-1; i > 0; i--)
        {
          if(UpdowmSide[0][i] != 1 && UpdowmSide[0][i+1] != 1)
          {
              if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                  index++;
              else
                  count++;
              /* 有弧线 */
              if(index > 20 && count > 20)
              {
                  *flag = 1;
                  return 1;
              }
          }
          else
          {
              index = 0;
              count = 0;
          }
        }
    }

    return 0;

}

uint8_t RoadIsRoundabout(uint8_t Upimage[2][LCDW], uint8_t imageInput[LCDH][LCDW], uint8_t image[LCDH][2], uint8_t *flag)
{
    uint8_t i = 0;
    uint8_t errL=0, errR=0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t count = 0;
    uint8_t num = 0, py;

    if(RoadUpSide_Mono(5, 120, Upimage))
        return 0;
    /* 从车头往前 左边线是否单调 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(image[i][0] == 0)
            continue;
        if(image[i][0] >= image[i+1][0])    // i是Y坐标值  0 是图像左线X坐标
        {
            num++;
            if(num == 50)
            {
                num = 0;
                leftState = 1;   // 左单调标志
                break;
            }
        }
        else
        {
            num = 0;
        }
        if(i == ROAD_END_ROW+1)  // Y加到11  清0
            num = 0;
    }
    errL = RoundaboutGetArc(image, 1, 5, &py);
    errR = RoundaboutGetArc(image, 1, 5, &py);

    /* 右边线是否单调 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(image[i][1] + 3 < image[i+1][1])
        {
            num++;
            if(num == 50)
            {
                num = 0;
                rightState = 1;
                break;
            }
        }
        if(i == ROAD_END_ROW+1)
            num = 0;
    }

    /* 左边单调， 检测右侧是否是环岛 */
    if(leftState == 1 && rightState == 0 && errL == 0)
    {
        count = 0;

        if(RoundaboutGetArc(image, 2, 5, &count))//左圆弧检测 (5个连续增 且 5个连续减)
        {
            *flag = 1;
            return 1;
        }
        else
        {
            return 0;
        }
    }

    /* 右边单调， 检测左侧是否是环岛 */
    if(rightState == 1 && leftState == 0)
    {
        count = 0;
        if(RoundaboutGetArc(image, 1, 5, &count))//左圆弧检测 (5个连续增 且 5个连续减)
        {
            *flag = 2;
            return 2;
        }
    }
    return 0;
}
/*!
  * @brief    获取环岛边线
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    status     ： 1：左环岛(边线)  2：右环岛(边线)
  *
  * @return
  *
  * @note     思路：环岛一边边线严格单调，根据一边边线，获取另一边线
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t status)
{
    uint8_t i = 0, j = 0;

    switch(status)
    {

        /* 左环岛 */
      case 1:
        {
            /* 重新确定左边界 */
            for(i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
            {
                for(j = LCDW/2; j > 0; j--)
                {
                    if(!imageInput[i][j])
                    {
                        imageSide[i][0] = j;
                        break;
                    }
                }
            }
            break;
        }

      case 2:
        {
            /* 重新确定右边界 */
            for(i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
            {
                for(j = LCDW/2; j < LCDW; j++)
                {
                    if(!imageInput[i][j])
                    {
                        imageSide[i][1] = j;
                        break;
                    }
                }
            }
            break;
        }
    }
}

void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[2][LCDW], uint8_t status)
{
    uint8_t i = 0, j = 0;

    switch(status)
    {
      case 1:
        {
            /* 重新确定上边界 */
            for(i = 159; i > 0; i--)
            {
                for(j = LCDH/2; j > 0; j--)
                {
                    if(!imageInput[j][i])
                    {
                        imageSide[0][i] = j;
                        break;
                    }
                }
            }
            break;
        }

      case 2:
        {
            /* 重新确定下边界 */
            for(i = 159; i > 0; i--)
            {
                for(j = LCDH/2; j < LCDH; j++)
                {
                    if(!imageInput[j][i])
                    {
                        imageSide[1][i] = j;
                        break;
                    }
                }
            }
            break;
        }
    }
}
/*!
  * @brief    判断边线是否存在弧形
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    status     ： 1：左边线  2：右边线
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num,uint8_t* index)
{
    int i = 0;
    uint8_t inc = 0, dec = 0, n = 0;
    switch(status)
    {
      case 1:
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][0] != 0 && imageSide[i+1][0] != 0)
            {
                if(imageSide[i][0] == imageSide[i+1][0]){
                    n++;
                    continue;
                }
                if(imageSide[i][0] > imageSide[i+1][0])
                {
                    inc++;
                    inc+=n;
                    n=0;
                }
                else
                {
                    dec++;
                    dec+=n;
                    n=0;
                }

                /* 有弧线 */
                if(inc > num && dec > num)
                {
                    *index = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;n=0;
            }
        }

        break;

      case 2:
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][1] != 159 && imageSide[i+1][1] != 159)
            {
                if(imageSide[i][1] == imageSide[i+1][1])
                {
                    n++;
                    continue;
                }
                if(imageSide[i][1] > imageSide[i+1][1])
                {
                    inc++;
                    inc+=n;
                    n = 0;
                }
                else
                {
                    dec++;
                    dec+=n;
                    n=0;
                }

                /* 有弧线 */
                if(inc > num && dec > num)
                {
                    *index = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;n=0;
            }
        }

        break;
    }

    return 0;
}

/*!
  * @brief    判断边线是否存在弧形
  *
  * @param    SideInput ： 上边线数组
  * @param    num       ： 检测幅度
  * @param    index     ： 最低点
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2021/12/01 星期三
  */
uint8_t UpSideErr(uint8_t SideInput[2][LCDW],uint8_t status ,uint8_t num, uint8_t * index)
{
    uint8_t dec = 0, inc = 0, i;
    //上线是否右突起
    switch(status)
    {
        case 1:
            for(i = 159-1; i > 0; i--)
                {
                  if(UpdowmSide[0][i] > 1 && UpdowmSide[0][i+1] > 1)
                  {
                      if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                          inc++;
                      else
                          dec++;
                      /* 有弧线 */
                      if(inc > num && dec > num)
                      {
                          * index = i + num;
                          return 1;
                      }
                  }
                  else
                  {
                      inc = 0;
                      dec = 0;
                  }
                }
            break;
        //下边线
        case 2:
            for(i = 159-1; i > 0; i--)
                {
                  if(UpdowmSide[1][i] != 1 && UpdowmSide[1][i+1] != 1)
                  {
                      if(UpdowmSide[1][i] >= UpdowmSide[1][i+1])
                          inc++;
                      else
                          dec++;
                      /* 有弧线 */
                      if(inc > num && dec > num)
                      {
                          * index = i + num;
                          return 1;
                      }
                  }
                  else
                  {
                      inc = 0;
                      dec = 0;
                  }
                }
            break;
    }

    return 0;
}

/*!
  * @brief    补线处理
  *
  * @param    imageSide  : 边线
  * @param    status     : 1：左边线补线   2：右边线补线
  * @param    startX     : 起始点 列数
  * @param    startY     : 起始点 行数
  * @param    endX       : 结束点 列数
  * @param    endY       : 结束点 行数
  *
  * @return
  *
  * @note     endY 一定要大于 startY
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
    int i = 0;

    /* 直线 x = ky + b*/
    float k = 0.0f, b = 0.0f;
    switch(status)
    {
      case 1://左补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(i = startY; i < endY; i++)
            {
                imageSide[i][0] = (uint8_t)(k * i + b);
            }
            break;
        }

      case 2://右补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(i = startY; i < endY; i++)
            {
                imageSide[i][1] = (uint8_t)(k * i + b);
            }
            break;
        }

    }
}

/*!
  * @brief    寻找跳变点
  *
  * @param    imageSide   ： 边线数组
  * @param    status      ：1：左边界   2：右边界
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x, uint8_t *y)
{
    int i = 0;
    uint8_t px = 0, py = 0;
    uint8_t count = 0;
    switch(state)
    {
      case 1:
        /* 寻找跳变点 */
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][0] == 0 && i > (ROAD_END_ROW + 5))
            {
                count++;

                if(count > 5)
                {
                    if(imageSide[i-1][0] > (imageSide[i][0] + 20))
                    {
                        py = i - 1;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-2][0] > (imageSide[i-1][0] + 20))
                    {
                        py = i - 2;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-3][0] > (imageSide[i-2][0] + 20))
                    {
                        py = i - 3;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-4][0] > (imageSide[i-3][0] + 20))
                    {
                        py = i - 4;
                        px = imageSide[py][0];
                        break;
                    }

                }

            }
            else
            {
                count = 0;
            }
        }

        if(py != 0)
        {
            *x = px;
            *y = py;
            return 1;
        }

        break;


      case 2:
        /* 寻找跳变点 */
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][1] == 159 && i > (ROAD_END_ROW + 5))
            {
                count++;

                if(count > 5)
                {
                    if(imageSide[i-1][1] < (imageSide[i][1] - 20))
                    {
                        py = i - 1;
                        px = imageSide[py][1];
                        break;
                    }
                    if(imageSide[i-2][1] < (imageSide[i-1][1] - 20))
                    {
                        py = i - 2;
                        px = imageSide[py][1];
                        break;
                    }
                    if(imageSide[i-3][1] < (imageSide[i-2][1] - 20))
                    {
                        py = i - 3;
                        px = imageSide[py][1];
                        break;
                    }
                    if(imageSide[i-4][1] < (imageSide[i-3][1] - 20))
                    {
                        py = i - 4;
                        px = imageSide[py][1];
                        break;
                    }

                }

            }
            else
            {
                count = 0;
            }
        }

        if(py != 0)
        {
            *x = px;
            *y = py;
            return 1;
        }

        break;
    }

    return 0;

}


/*!
  * @brief    环岛补线处理
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageSide  ： 边线数组
  * @param    status     ：环岛标志位   奇数为右环岛，偶数为左环岛（0为结束，左环岛没有修改）
  *
  * @return
  *
  * @note     这里只写了左环岛，右环岛大家可以参考左环岛自己完善
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
void RoundaboutProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t UpdowmSide[2][LCDW], uint8_t* state)
{
    uint8_t i = 0, err5 = 0;
    uint8_t pointX = 0, pointY = 0, inc = 0, dec = 0;
    uint8_t flag= 0, Down_flag = 0;
    static uint8_t finderr = 0, Up_flag = 0, err1 = 0;
    switch(*state)
    {
        /* 发现右环岛 环岛出口处补线 */
      case 1:

        /* 重新确定右边界 */
        RoundaboutGetSide(imageInput, imageSide, 2);

        /* 检查弧线 */
        err1 = RoundaboutGetArc(imageSide, 2, 5, &pointY);

        /* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
        if(err1)
        {
            pointX = imageSide[pointY][1];
//            UART_PutStr(UART0, "err\r\n");
//
//            /* 准备入环岛 */
//            if((pointY + 10) > ROAD_MAIN_ROW)
//            {
//                * state = 3;
//            }
            //补线
            ImageAddingLine(imageSide, 2, pointX, pointY, 159, ROAD_START_ROW);
            finderr = 1;
        }
        else
        {
            if(finderr)
                *state = 3;//准备进入环岛
        }

        break;

        /* 发现左环岛 环岛出口处补线 */
      case 2:

          /* 重新确定左边界 */
          RoundaboutGetSide(imageInput, imageSide, 1);

          /* 检查弧线 */
          err1 = RoundaboutGetArc(imageSide, 1, 5, &pointY);

          /* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
          if(err1)
          {
              pointX = imageSide[pointY][0];
              ImageAddingLine(imageSide, 1, 160-pointX, 160-pointY, 0, ROAD_START_ROW);
              finderr = 1;
          }
          else
          {
              if(finderr)
                  *state = 4;
          }
        break;


        /* 准备进入环岛， 左补线 */
      case 3:
        /* 重新确定上边界 */
        Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
        pointY = 0;
        pointX = 0;

        /* 上边界最低点 */
        for(i = 40; i < 100; i++)
        {
            if(UpdowmSide[0][i] > pointY)
            {
                pointX = i;
                pointY = UpdowmSide[0][i];
            }
        }
        if(pointY >= 50)//最低点大于50（根据自己实际情况修改）
        {
            if(RoadUpSide_Mono(5, 100,UpdowmSide) == 1)//上线单调增进入下一步
                *state = 5;
            ImageAddingLine(imageSide, 1, 100+30, 40-10,0, ROAD_START_ROW);//补线（自行修改）
        }
        else
            ImageAddingLine(imageSide, 1, 60, 40-15,0, ROAD_START_ROW); //补线（补线角度自行修改）
        break;

      case 4:
          /* 重新确定上边界 */
          Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
          pointY = 0;
          pointX = 0;

          /* 上边界最低点 */
          for(i = 40; i < 100; i++)
          {
              if(UpdowmSide[0][i] > pointY)
              {
                  pointX = i;
                  pointY = UpdowmSide[0][i];
              }
          }
          if(pointY >= 50)
          {
              if(RoadUpSide_Mono(5, 100,UpdowmSide) == 1)
                  *state = 6;
              ImageAddingLine(imageSide, 2, 10, 40-10, 159, ROAD_START_ROW);
          }
          else
              ImageAddingLine(imageSide, 2, 100, 40-15, 159, ROAD_START_ROW);
        break;
        /* 出环岛， 直道处补线 */
      case 5:
          flag = 0;
          /* 检查弧线 */
          for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
          {
              if(imageSide[i][0] != 0 && imageSide[i+1][0] != 0)
              {
                  if(imageSide[i][0] >= imageSide[i+1][0])
                      inc++;
                  else
                      dec++;
                  /* 有弧线 */
                  if(inc > 10 && dec > 10)err5 = 1;//（参数10：弧的幅度，可自行修改）
              }
              else
              {
                  inc = 0;
                  dec = 0;
              }
          }

          //下线为119
          for(i = 159; i > 0; i--)
            {
                if(UpdowmSide[1][i] == 119)
                    inc++;
                else
                    dec++;
                if( dec <= 15)
                {
                    Down_flag = 1;
                    break;
                }

            }

          //检查上线单调性
          flag = RoadUpSide_Mono(20, 155,UpdowmSide);

          if(flag && err5 && Down_flag)
          {
              *state = 7;
          }
          break;

          /* 出环岛， 直道处补线 */
      case 6:
          flag = 0;
        /* 检查弧线 */
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][1] != 159 && imageSide[i+1][1] != 159)
            {
                if(imageSide[i][1] > imageSide[i+1][1])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if(inc > 8 && dec > 8)err5 = 1;
            }
            else
            {
                inc = 0;
                dec = 0;
            }
        }

        //下线为119
        for(i = 159; i > 0; i--)
          {
              if(UpdowmSide[1][i] == 119)
                  inc++;
              else
                  dec++;
              if( dec <= 15)
              {
                  Down_flag = 1;
                  break;
              }

          }

        //检查上线单调性
        flag = RoadUpSide_Mono(20, 155,UpdowmSide);

        if(flag && err5 && Down_flag)
        {
            *state = 8;
//                  ImageAddingLine(imageSide, 1, 145, 30,0, ROAD_START_ROW);
        }
        break;
        //出环
      case 7:

          ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW);//参数自行修改

          //判断上线是否有突起
            for(i = 159-1; i > 0; i--)
            {
                if(UpdowmSide[0][i] != 0 && UpdowmSide[0][i+1] != 0){
                    if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                        inc++;
                    else
                        dec++;
                    if(inc > 20 && dec > 20){
                        finderr = 0; Up_flag = 0; err1 = 0; //清空静态变量以便下次使用
//                        Target_Speed1 = 25;               //速度回复
//                        Target_Speed2 = 25;
//                        Servo_P = 18;                     //转向回复
                        *state = 0;                         //环岛结束
                        break;
                    }
                }else{
                    inc = 0;
                    dec = 0;
                }

            }
            break;

      case 8:
        Servo_P = 20;
        ImageAddingLine(imageSide, 1, 30, 30,159, ROAD_START_ROW);
//          Up_flag = RoadUpSide_Mono(20, 155,UpdowmSide);
//          if(flag == 1)
//          {
          for(i = 159-1; i > 0; i--)
          {
              if(UpdowmSide[0][i] != 0 && UpdowmSide[0][i+1] != 0){
                  if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                      inc++;
                  else
                      dec++;
                  if(inc > 20 && dec > 20){
                      finderr = 0; Up_flag = 0; err1 = 0; //清空静态变量以便下次使用
                      Servo_P = 15;                      //转向回复
                      *state = 0;
                      break;
                  }
              }else{
                  inc = 0;
                  dec = 0;
              }

          }
          break;
//          }
    }
}

/*!
  * @brief    获取十字边线
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  *
  * @return
  *
  * @note     思路：从中间向两边搜线
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
void CrossGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2])
{
    uint8_t i = 0, j = 0;

    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        for(j = 78; j > 1; j--)
        {
            if(imageInput[i][j])
            {
                imageSide[i][0] = j;
                break;
            }
        }

        for(j = 78; j < 159; j++)
        {
            if(imageInput[i][j])
            {
                imageSide[i][1] = j;
                break;
            }
        }
    }

}

/*!
  * @brief    十字补线处理
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageSide  ： 边线数组
  * @param    status     ：十字标志位   1：发现十字    2：进入十字   3：出十字
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
void CrossProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t* state)
{

    uint8_t pointX = 0, pointY = 0;
    uint8_t leftIndex = 0;
    static uint8_t count  = 0;
    switch(*state)
    {
      case 1:
        {
            /* 重新获取边线 */
            CrossGetSide(imageInput, imageSide);

            /* 寻找跳变点 */
            if(ImageGetHop(imageSide, 1, &pointX, &pointY))
            {
                /* 补线 */
                ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
            }

            leftIndex = pointY;
            pointX = 0;
            pointY = 0;

            /* 寻找跳变点 */
            if(ImageGetHop(imageSide, 2, &pointX, &pointY))
            {
                /* 补线 */
                ImageAddingLine(imageSide, 2, pointX, pointY, (LCDW - 1), ROAD_START_ROW);
            }

            if(leftIndex != 0 && pointY != 0 && leftIndex >= ROAD_MAIN_ROW && pointY >= ROAD_MAIN_ROW)
            {
                * state = 2;
                count = 0;
            }

            if(count++ > 20)
            {
                * state = 2;
                count = 0;
            }

            break;
        }

      case 2:
        {

            /* 检查弧线 */
            if(RoundaboutGetArc(imageSide, 1, 5, &leftIndex))
            {
                /* 重新确定左边界 */
                RoundaboutGetSide(imageInput, imageSide, 1);

                if(ImageGetHop(imageSide, 1, &pointX, &pointY))
                {
                    /* 补线 */
                    ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);

                    * state = 3;

                    count = 0;
                }
                else
                {
                    imageSide[ROAD_MAIN_ROW][0] = LCDW/2;
                    imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
                }
            }

            break;
        }

      case 3:
        {

            /* 重新确定左边界 */
            RoundaboutGetSide(imageInput, imageSide, 1);


            if(ImageGetHop(imageSide, 1, &pointX, &pointY))
            {
                /* 检查弧线 */
                if(RoundaboutGetArc(imageSide, 1, 5, &leftIndex))
                {
                    /* 补线 */
                    ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
                }
                else
                {
                    /* 补线 */
                    ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
                }

                if(pointY >= ROAD_MAIN_ROW)
                {
                    * state = 0;
                    count = 0;
                }
            }
            else
            {
                imageSide[ROAD_MAIN_ROW][0] = 120;
                imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
            }

            if(count++ > 10)
            {
                *state = 0;
                count = 0;
            }

            break;
        }
    }

}



/*!
  * @brief    判断是否是Y型岔口
  *
  * @param    imageSide ： 图像边线信息
  * @param    flag      ： Y型状态信息
  *
  * @return   0：不是， 1：是
  *
  * @note     思路：三线成弧
  *
  * @see
  *
  * @date     2021/12/8 星期三
  */
uint8_t RoadIsFork(uint8_t imageInput[2][LCDW],uint8_t imageSide[LCDH][2], uint8_t *flag, uint8_t * pY )
{

    uint8_t i = 0, errR = 0, errF = 0;
    uint8_t inc = 0, dec = 0, num = 0;
    uint8_t pointY;

    /* 检查弧线 */
    errR = RoundaboutGetArc(imageSide, 2, 5, &pointY);
    errF = RoundaboutGetArc(imageSide, 1, 5, &pointY);

    if(errR)
    {
        if(UpSideErr(imageInput, 1, 20, &pointY))
        {
            for(i = 110; i>40; i--)
            {
                if(imageSide[i][0] == 0)
                    num++;
                if(num==65){
                    *flag=1;
                    return 1;
                }
            }
        }
    }
    num = 0;
    if(errR && errF)
    {
        //判断上线是否有弧 （推荐判断弧的用这种办法，可自己封装成一个函数，之前的办法有一定的局限性，可自己替换）
        for(i = 159-1; i > 0; i--)
            {
              if(UpdowmSide[0][i] != 0 && UpdowmSide[0][i+1] != 0)
              {
                  if(UpdowmSide[0][i] == UpdowmSide[0][i+1]){
                      num++;continue;
                  }
                  if(UpdowmSide[0][i] > UpdowmSide[0][i+1]){
                      inc++;
                      inc+=num;
                      num=0;
                  }
                  if(UpdowmSide[0][i] < UpdowmSide[0][i+1]){
                      dec++;
                      dec+=num;
                      num=0;
                  }
                  /* 有弧线 */
                  if(inc > 15 && dec > 15)
                  {
                      *flag = 1;
                      return 1;
                  }
              }
              else
              {
                  inc = 0;
                  dec = 0;
                  num = 0;
              }
            }
    }
    return 0;
}


/*!
  * @brief    Y字补线处理
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageSide  ： 边线数组
  * @param    status     ：十字标志位   1：发现十字    2：进入十字   3：出十字
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */

sint32 RAllFork = 0;
void ForkProcess(uint8_t UpSideInput[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t* state)
{

  uint8_t pointY, pointX;
  static uint8_t D_flag = 0, dou_flag;

    //重新获取上边线
    UpdownSideGet(Bin_Image, UpdowmSide);

    switch(*state)
    {
        case 1://判断拐点 进入拐点
            UpSideErr(UpSideInput, 1, 15, &pointY);
            if((UpSideInput[0][pointY] > 30) || (D_flag))
            {
                ImageAddingLine(imageSide, 1, 110, 35, 0, ROAD_START_ROW);  // 屏幕左下角连拐点（可自行修改）
                D_flag = 1;
            }
            if(D_flag == 1 && RoadUpSide_Mono(30, 150, UpSideInput) == 2)
            {
                    *state = 2;
            }
            break;
        case 2://出 补线

            if((dou_flag == 1) && (!RoundaboutGetArc(imageSide, 2, 5, &pointY)))
                *state = 3;
            if(RoundaboutGetArc(imageSide, 2, 5, &pointY))
                dou_flag = 1;
            break;
        case 3://出 补线
            ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW);//可自行修改
            if(RoadUpSide_Mono(5, 90, UpSideInput)) //判断出口结束三岔口
            {
                Servo_P = 12;
                if(g_ucForkNum == 2)
                {
                    Servo_P = 12;
                }
                D_flag=0;
                dou_flag=0;
                *state = 0;
            }
            break;
    }
}

/*!
  * @brief    停车线处理
  *
  * @param    imageSide  ： 边线数组
  * @param    state      ： 停车状态  1：车库在左侧   2：车库在右侧
  * @param    speed      ： 速度
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
void ZebraProcess(uint8_t imageSide[LCDH][2], uint8_t state, int16_t* speed)
{
    static uint16_t count = 0;

    count++;

    if(state == 1)
    {
        imageSide[ROAD_MAIN_ROW][0] = 0;
        imageSide[ROAD_MAIN_ROW][1] = LCDW/2;
    }
    else
    {
        imageSide[ROAD_MAIN_ROW][0] = LCDW/2;
        imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
    }

    if(count > 100)
    {
        * speed = 0;
        while(1);
    }

}

/*!
  * @brief    根据主跑行，求取舵机偏差
  *
  * @param
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex)
{

    return imageSide[lineIndex][0] + imageSide[lineIndex][1] - 158;

}

/*!
  * @brief    判断是否丢线
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    lineIndex  ： 行
  *
  * @return   0：没有丢线   1:左边丢线  2：右边丢线  3： 左右都丢线   4：错误
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t lineIndex)
{
    uint8_t state = 0;
    uint8_t i = 0;
    static uint8_t last = 78;

    imageOut[lineIndex][0] = 0;
    imageOut[lineIndex][1] = 159;
    /* 用距离小车比较近的行 判断是否丢线 */
    for(i = last; i > 1; i--)
    {
        if(imageInput[lineIndex][i])
        {
            imageOut[lineIndex][0] = i;
            break;
        }
    }

    if(i == 1)
    {
        /* 左边界丢线 */
        state = 1;
    }


    for(i = last; i < 159; i++)
    {
        if(imageInput[lineIndex][i])
        {
            imageOut[lineIndex][1] = i;
            break;
        }
    }

    if(i == 159)
    {
        /* 左右边界丢线 */
        if(state == 1)
        {
            state = 3;
        }

        /* 右边界丢线 */
        else
        {
            state = 2;
        }

    }
    if(imageOut[lineIndex][1] <= imageOut[lineIndex][0])
    {
        state = 4;
    }
    return state;

}


/*!
  * @brief    丢线处理
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    mode       ： 那边丢线？   1：左边丢线  2：右边丢线
  * @param    lineIndex  ： 丢线行数
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t mode, uint8_t lineIndex)
{
    uint8_t i = 0, j = 0, count = 0;

    switch(mode)
    {
      case 1:
        for(i = imageOut[lineIndex][1]; i > 1; i--)
        {
            count++;
            for(j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
            {
                if(imageInput[j][i])
                {
                    imageOut[lineIndex - count][0] = 0;
                    imageOut[lineIndex - count][1] = i;
                    break;
                }

            }
        }
        break;


      case 2:
        for(i = imageOut[lineIndex][0]; i < 159; i++)
        {
            count++;
            for(j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
            {
                if(imageInput[j][i])
                {
                    imageOut[lineIndex - count][0] = i;
                    imageOut[lineIndex - count][1] = 159;
                    break;
                }

            }
        }
        break;

    }

}



/*!
  * @brief    获取边线
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  *
  * @return   是否丢线
  *
  * @note     思路：从距离车头较近的行开始从中间向两边搜线
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2])
{
    uint8_t i = 0, j = 0;

    RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);

    /* 离车头近的40行 寻找边线 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        imageOut[i][0] = 0;
        imageOut[i][1] = 159;

        /* 根据边界连续特性 寻找边界 */
        for(j = imageOut[i+1][0] + 10; j > 0; j--)
        {
            if(!imageInput[i][j])
            {
                imageOut[i][0] = j;
                break;
            }
        }
        for(j = imageOut[i+1][1] - 10; j < 160; j++)
        {
            if(!imageInput[i][j])
            {
                imageOut[i][1] = j;
                break;
            }
        }
        /* 如果左边界 即将超出中线 则检查是否右丢线 */
        if(imageOut[i][0] > (LCDW/2 - 10) && imageOut[i][1] >  (LCDW - 5))
        {
            /* 右丢线处理 */
            RoadNoSideProcess(imageInput, imageOut, 2, i);

            if(i > 70)
            {
                imageOut[i][0] += 50;
            }
            return 1;
        }

        /* 如果右边界 即将超出中线 则检查是否左丢线 */
        if(imageOut[i][1] < (LCDW/2 + 10) && imageOut[i][0] <  (5))
        {
            /* 左丢线处理 */
            RoadNoSideProcess(imageInput, imageOut, 1, i);

            if(i > 70)
            {
                imageOut[i][1] -= 50;
            }
            return 2;

        }
    }
    return 0;
}

/*!
  * @brief    获取边线
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  *
  * @return   是否丢线
  *
  * @note     思路：从距离车头较近的行开始从中间向两边搜线
  *
  * @see
  *
  * @date     2021/11/30 星期二
  */
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW])
{
    uint8_t i = 0, j = 0;
    uint8_t last = 60;

    imageOut[0][159] = 0;
    imageOut[1][159] = 119;
    /* 用中线比较近的行 判断是否丢线 */
    for(i = last; i >= 0; i--)
    {
        if(!imageInput[i][80])
        {
            imageOut[0][80] = i;
            break;
        }
    }

    for(i = last; i < 120; i++)
    {
        if(!imageInput[i][80])
        {
            imageOut[1][80] = i;
            break;
        }
    }

    /* 中线往左 寻找边线 */
    for(i = 80-1; i > 0; i--)
    {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* 根据边界连续特性 寻找边界 */
        for(j = imageOut[0][i+1] + 10; j > 0; j--)
        {
            if(!imageInput[j][i])
            {
                imageOut[0][i] = j;
                break;
            }
        }
        for(j = imageOut[1][i+1] - 10; j < 120; j++)
        {
            if(!imageInput[j][i])
            {
                imageOut[1][i] = j;
                break;
            }
        }
    }
    /*中线往右 寻找边线*/
    for(i = 80+1; i < 159; i++)
        {
            imageOut[0][i] = 0;
            imageOut[1][i] = 119;

            /* 根据边界连续特性 寻找边界 */
            for(j = imageOut[0][i-1] + 10; j > 0; j--)
            {
                if(!imageInput[j][i])
                {
                    imageOut[0][i] = j;
                    break;
                }
            }
            for(j = imageOut[1][i-1] - 10; j < 120; j++)
            {
                if(!imageInput[j][i])
                {
                    imageOut[1][i] = j;
                    break;
                }
            }
        }
    return 0;
}

/*!
  * @brief    除单一的噪点
  *
  * @param
  *
  * @return
  *
  * @note     思路： 检查边沿邻域内的9个点，如果大于设置值，则保留该点
  *
  * @see
  *
  * @date     2020/6/24 星期三
  */
void ImagePortFilter(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][LCDW])
{
    uint8_t temp = 0;

    for(int i = 1; i < LCDH - 1; i++)
    {
        for(int j = 1; j < LCDW - 1; j++)
        {
            temp = imageInput[i-1][j-1] + imageInput[i-1][j] + imageInput[i-1][j+1] +
                   imageInput[i  ][j-1] + imageInput[i  ][j] + imageInput[i  ][j+1] +
                   imageInput[i+1][j-1] + imageInput[i+1][j] + imageInput[i+1][j+1];

            /* 邻域内5个点是边沿 则保留该点 可以调节这里优化滤波效果 */
            if(temp > 4)
            {
                imageOut[i][j] = 1;
            }
            else
            {
                imageOut[i][j] = 0;
            }

        }
    }
}


/*!
  * @brief    边线数组
  *
  * @note
  */
uint8_t ImageSide[LCDH][2];
uint8_t UpdowmSide[2][LCDW];
uint8_t spinodalX;
uint8_t spinodalY;


uint8_t rightline[80];
uint8_t leftline[80];
uint8_t midline[80];

float Change_Key(float min);



float bat = 0;
int l_motor = 0;
int r_motor = 0;

unsigned char hang = 0;
unsigned char lie = 0;

void TFT_Show_Camera_Info(void)
{
    static int scan_flag = 0;//界面选择标识符
    static unsigned lock_flag = 1;//界面锁定标识符
    static int change_flag = 0;//其他界面操作拓展用标识符
    char txt[32];

    ServoCtrl(servo_mid);

    //K0进行界面选择
    if(KEY_Read(KEY0) == 0 && lock_flag == 1)
    {
        delayms(200);
        while(!KEY_Read(KEY0));
        scan_flag +=1;
        change_flag = 0;
        TFTSPI_CLS(u16BLACK);
        PIN_Write(P33_8, 1);
        delayms(100);
        PIN_Write(P33_8, 0);
    }
    if(scan_flag<0)scan_flag = 3;
    else if(scan_flag>=4)scan_flag = 0;

    if(scan_flag == 0)
    {
        /* 显示二值化图像信息 */
            //按下K2锁死屏幕画面，不可恢复
//            if(KEY_Read(KEY2)==0)
//            {
//                while(1)
//                {
//                    if(KEY_Read(KEY0)==0)
//                    {
//                        hang++;
//                    }
//
//                    TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);//显示中线
//                    TFT_RoadSide((unsigned char *)rightline,(unsigned char *)leftline,(unsigned char *)midline);
//                    TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);
//                    TFTSPI_Draw_Line(0, hang, 159, hang, u16RED);
//                    sprintf(txt ,"hang=%d         ",hang);
//                    TFTSPI_P8X8Str(0, 15, txt, u16WHITE, u16BLACK);
//                    if(KEY_Read(KEY1)==0)
//                    {
//                        while(1)
//                        {
//                            if(KEY_Read(KEY0)==0)
//                            {
//                                lie++;
//                            }
//                            if(lie>=160) lie = 0;
//
//                            TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);//显示中线
//                            TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);
//                            TFT_RoadSide((unsigned char *)rightline,(unsigned char *)leftline,(unsigned char *)midline);
//                            TFTSPI_Draw_Line(0, hang, 159, hang, u16RED);
//                            sprintf(txt ,"hang=%d   ",hang);
//                            TFTSPI_P8X8Str(0, 15, txt, u16WHITE, u16BLACK);
//                            sprintf(txt ,"lie=%d         ",lie);
//                            TFTSPI_P8X8Str(8, 15, txt, u16WHITE, u16BLACK);
//                            TFTSPI_Draw_Line(lie, 0, lie, 119, u16RED);
//                        }
//                    }
//
//                }
//                TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);                       //显示中线
//            }

            TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);  //显示边缘提取图像
//            TFTSPI_BinRoadSide(ImageSide);                                  //左右边线
//            TFTSPI_BinRoad_UpdownSide(UpdowmSide);                          //上下边线
            TFT_RoadSide((unsigned char *)rightline,(unsigned char *)leftline,(unsigned char *)midline);
//            TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 159, ROAD_MAIN_ROW, u16RED); //领跑行显示
            TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);                       //显示中线

//            TFT_line();                                 //左右边线
//            TFTSPI_CLS(u16BLACK);                   // 清屏
//            TFTSPI_BinRoadSide(ImageSide);          //左右边线
//            TFTSPI_BinRoad_UpdownSide(UpdowmSide);  //上下边线
//            TFT_line();                                     //左右边线

    }
    else if(scan_flag == 1)
    {
        if(KEY_Read(KEY1) == 0)
        {
            change_flag += 1;
            delayms(500);//延时等待，防止在下面的程序进行中按键操作混淆
            if(change_flag != 0)
            {
                lock_flag = 0;
                TFTSPI_P8X8Str(12, 0, "unlock ", u16WHITE, u16BLACK);
            }
            else
            {
                lock_flag = 1;
                TFTSPI_P8X8Str(12, 0, "locking", u16WHITE, u16BLACK);
            }
        }

        if(lock_flag == 1)
        {
            TFTSPI_P8X8Str(12, 0, "locking", u16WHITE, u16BLACK);
//           TFTSPI_CLS(u16BLACK);
            TFTSPI_P8X8Str(0, 1, "speed_pid", u16WHITE, u16BLACK);//速度界面提示
            sprintf(txt ,"kp=%.2f",mot_kp);
            TFTSPI_P8X8Str(1, 2, txt, u16WHITE, u16BLACK);

            sprintf(txt ,"kd=%.2f",mot_kd);
            TFTSPI_P8X8Str(1, 3, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"ki=%.2f",mot_ki);
            TFTSPI_P8X8Str(1, 4, txt, u16WHITE, u16BLACK);

            TFTSPI_P8X8Str(0, 6, "dire_pid", u16WHITE, u16BLACK);//方向界面提示
            sprintf(txt ,"kp=%.2f",dir_kp);
            TFTSPI_P8X8Str(1, 7, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"kd=%.2f",dir_kd);
            TFTSPI_P8X8Str(1, 8, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"ki=%.2f",dir_ki);
            TFTSPI_P8X8Str(1, 9, txt, u16WHITE, u16BLACK);

        }
        else
        {
            switch(change_flag)
            {
                case 1:
                    mot_kp += Change_Key(0.1);
                    sprintf(txt ," ki=%.2f ",dir_ki);
                    TFTSPI_P8X8Str(0, 9, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">kp=%.2f+",mot_kp);
                    TFTSPI_P8X8Str(0, 2, txt, u16WHITE, u16BLACK);
                    break;
                case 2:
                    mot_kd += Change_Key(0.1);
                    sprintf(txt ," kp=%.2f ",mot_kp);
                    TFTSPI_P8X8Str(0, 2, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">kd=%.2f+",mot_kd);
                    TFTSPI_P8X8Str(0, 3, txt, u16WHITE, u16BLACK);
                    break;
                case 3:
                    mot_ki += Change_Key(0.1);
                    sprintf(txt ," kd=%.2f ",mot_kd);
                    TFTSPI_P8X8Str(0, 3, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">ki=%.2f+",mot_ki);
                    TFTSPI_P8X8Str(0, 4, txt, u16WHITE, u16BLACK);
                    break;
                case 4:
                    dir_kp += Change_Key(0.1);
                    sprintf(txt ," ki=%.2f ",mot_ki);
                    TFTSPI_P8X8Str(0, 4, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">kp=%.2f+",dir_kp);
                    TFTSPI_P8X8Str(0, 7, txt, u16WHITE, u16BLACK);
                    break;
                case 5:
                    dir_kd += Change_Key(0.1);
                    sprintf(txt ," kp=%.2f ",dir_kp);
                    TFTSPI_P8X8Str(0,7, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">kd=%.2f+",dir_kd);
                    TFTSPI_P8X8Str(0, 8, txt, u16WHITE, u16BLACK);
                    break;
                case 6:
                    dir_ki += Change_Key(0.1);
                    sprintf(txt ," kd=%.2f ",dir_kd);
                    TFTSPI_P8X8Str(0, 8, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">ki=%.2f+",dir_ki);
                    TFTSPI_P8X8Str(0,9, txt, u16WHITE, u16BLACK);
                    break;
                default:
                    change_flag = 0;
                    lock_flag = 1;//锁定界面
                    TFTSPI_P8X8Str(12, 0, "locking", u16WHITE, u16BLACK);
                    break;
            }
        }
    }
    else if(scan_flag == 2)
    {
        if(KEY_Read(KEY1) == 0)
        {
            delayms(200);
            while(!KEY_Read(KEY1));
            change_flag += 1;
            //延时等待，防止在下面的程序进行中按键操作混淆
            if(change_flag != 0)
            {
                lock_flag = 0;
                TFTSPI_P8X8Str(12, 0, "unlock ", u16WHITE, u16BLACK);
            }
            else
            {
                lock_flag = 1;
                TFTSPI_P8X8Str(12, 0, "locking", u16WHITE, u16BLACK);
            }
        }

        if(lock_flag == 1)
        {
//            servo_mid += Change_Key(5);

            bat = ADC_ReadAverage(ADC7,5)/220.8;
            sprintf(txt ,"bat=%.1f V   ",bat);
            TFTSPI_P8X8Str(0, 4, txt, u16WHITE, u16BLACK);
            sprintf(txt ," servo_mid=%0.1f   ",servo_mid);
            TFTSPI_P8X8Str(0, 6, txt, u16WHITE, u16BLACK);
            sprintf(txt ," motor=%0.1f   ",motor);
            TFTSPI_P8X8Str(0, 8, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"leftspeed=%0.1f m/s    ",(float)((((float)ECPULSE1)/512.0)*17.65));
            TFTSPI_P8X8Str(0, 10, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"rightspeed=%0.1f m/s    ",(float)((((float)ECPULSE2)/512.0)*17.65));
            TFTSPI_P8X8Str(0, 12, txt, u16WHITE, u16BLACK);
        }
        else
        {
            bat = ADC_ReadAverage(ADC7,5)/220.8;
            sprintf(txt ,"bat=%.1f V   ",bat);
            TFTSPI_P8X8Str(0, 4, txt, u16WHITE, u16BLACK);

            sprintf(txt ,"leftspeed=%d m/s    ",ECPULSE1);
//            sprintf(txt ,"leftspeed=%0.1f m/s    ",(float)((((float)ECPULSE1)/512.0)*17.65));
            TFTSPI_P8X8Str(0, 10, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"rightspeed=%d m/s    ",ECPULSE2);
//            sprintf(txt ,"rightspeed=%0.1f m/s    ",(float)((((float)ECPULSE2)/512.0)*17.65));
            TFTSPI_P8X8Str(0, 12, txt, u16WHITE, u16BLACK);

            switch(change_flag)
            {
                case 1:
                    servo_mid += (float)Change_Key(10.0);
                    sprintf(txt ,">servo_mid=%0.1f   ",servo_mid);
                    TFTSPI_P8X8Str(0, 6, txt, u16WHITE, u16BLACK);
                    sprintf(txt ," motor=%0.1f   ",motor);
                    TFTSPI_P8X8Str(0, 8, txt, u16WHITE, u16BLACK);
                    break;
                case 2:
                    motor += (float)Change_Key(500);
                    sprintf(txt ," servo_mid=%0.1f   ",servo_mid);
                    TFTSPI_P8X8Str(0, 6, txt, u16WHITE, u16BLACK);
                    sprintf(txt ,">motor=%0.1f   ",motor);
                    TFTSPI_P8X8Str(0, 8, txt, u16WHITE, u16BLACK);
                    break;
                case 3:
                default:
                    change_flag = 0;
                    lock_flag = 1;//锁定界面
                    TFTSPI_P8X8Str(12, 0, "locking", u16WHITE, u16BLACK);
                    break;
            }
        }

//        MotorCtrl((sint32)(-motor),(sint32)(motor));
    }
    else if(scan_flag == 3)
    {

    }
//    //按下K2锁死屏幕画面，不可恢复
//    if(KEY_Read(KEY2)==0) while(1);
//       /* 调试时可以打开这里 */
//    if(KEY_Read(KEY0)==0)
//    {
//        //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (uint8_t*)Image_Use);        //图像显示
//        TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);  //显示边缘提取图像
//        TFTSPI_BinRoadSide(ImageSide);                                  //左右边线
//        TFTSPI_BinRoad_UpdownSide(UpdowmSide);                          //上下边线
//        TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 159, ROAD_MAIN_ROW, u16RED); //领跑行显示
//        TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);                       //显示中线
//    }
//    else
//    {
//        TFTSPI_CLS(u16BLACK);                   // 清屏
//

    sprintf(txt, "%05d", g_sSteeringError);         //误差值
    TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);

//    sprintf(txt, "%02d", g_ucFlagRoundabout);       //环岛标志
//    TFTSPI_P6X8Str(8, 15, txt, u16RED, u16BLUE);
//
//    sprintf(txt, "%02d", g_ucFlagT);                //T口标志
//    TFTSPI_P6X8Str(12, 15, txt, u16RED, u16BLUE);
//
//    sprintf(txt, "%02d", g_ucFlagFork);             //Y口标志
//    TFTSPI_P6X8Str(16, 15, txt, u16RED, u16BLUE);

//    sprintf(txt, "%02d", g_ucFlagZebra);            //斑马线
//    TFTSPI_P6X8Str(20, 15, txt, u16RED, u16BLUE);


//    sprintf(txt, "%d", );
//    TFTSPI_P6X8Str(23, 15, txt, u16RED, u16BLUE);

}

float Change_Key(float min)
{
    if(KEY_Read(KEY0) == 0)
    {
        delayms(200);
        while(!KEY_Read(KEY0));
        return min;
    }
    else if(KEY_Read(KEY2) == 0)
    {
        delayms(200);
        while(!KEY_Read(KEY2));
        return (-min);
    }
    else
        return 0;
}



unsigned char track_flag = 0;
unsigned char right_w[40] = {0};
unsigned char left_w[40] = {0};
unsigned char mid_w[40] = {0};

unsigned char right_lost = 0;
unsigned char left_lost = 0;
unsigned char all_lost = 0;

//赛道宽度初始化
void track_init(void)
{
    unsigned char i;
    if(track_flag == 0)
    {
        for(i=110;i>30;i++)
        {
            right_w[i] = rightline[110-i];
            left_w[i] = leftline[110-i];
            mid_w[i] = (right_w[i] + left_w[i])/2;
        }
        track_flag = 1;
    }
}

//unsigned char t_leftline[16];
//unsigned char t_rightline[16];



//unsigned char image_transform(unsigned char hang ,unsigned char lie ,unsigned char *Pixle)
//{
//    float Hd[3][3] = { {7.8264 ,0.0000 ,-133.9017 },{4.4812 ,3.0380 ,-240.8244 },{0.0564 ,0.0000 ,0.0347 }};
//    unsigned char i = 0,j = 0;
//    for(i=0;i<120;i++)
//    {
//        for(j=0;j<160)
//    }
////    (float)((float)hang * Hd[0][0] + (float)lie * Hd[0][1] + Hd[0][2])/((float)hang * Hd[2][0] + (float)lie * Hd[2][1] + Hd[2][2]);
////    (float)((float)hang * Hd[1][0] + (float)lie * Hd[1][1] + Hd[1][2])/((float)hang * Hd[2][0] + (float)lie * Hd[2][1] + Hd[2][2]);
//}



//
//void Set_line(void)
//{
//    unsigned char i,j = 0;//循环用变量
//    //左中右边线数据转出,转入use组进行修改
//    for(i=120-1;i>20;i-=4)
//    {
//        rightline_use[i] = rightline[i];
//        leftline_use[i] = leftline[i];
//        midline_use[i] = midline[i];
//    }
//
//    /************************************边线位置数据继承****************************************************/
//               //由丢线位置临近判断使用历史继承
//    //daixiugai
//               if(right_break>=116)//开始使用历史边线数组
//               {
//                   //右边线迭代
//                   for(i=120-1;i>110;i-=4)
//                   {
//                       rightline[i] = rightline_use[i-4];
//                   }
//               }
//               if(left_break > 110)//开始使用历史边线数组
//               {
//                   //zuo边线迭代
//                   for(i=120-1;i>110;i-=4)
//                   {
//                       leftline_use[i] = leftline_use[i-4];
//                   }
//               }
//    /************************************以上边线位置数据继承****************************************************/
//
//    if(right_break != 120 || left_break !=120)
//    {
//        if(right_break < 120)
//        {
//            for(i=120-1;i>right_break;i-=4)
//            {
//                rightline_use[i] = rightline[i-4];
//            }
//            //右边线原始图像补齐
//            right_ck = (rightline[right_break] - rightline[right_break+4])/(float)(right_break-(right_break+4));
//            for(i=right_break;i>20;i--)
//            {
//                for(j=rightline[right_break];j>0;j--)
//                {
//                    if(((j-rightline[right_break+4])/(i-(right_break+4)))/right_ck >0.9 &&
//                            ((j-rightline[right_break+4])/(i-(right_break+4)))/right_ck < 1.1)
//                    {
//                        rightline_use[i] = j;
//                    }
//                }
//            }
//        }
//        if(left_break < 120)
//        {
//            for(i=120-1;i>right_break;i-=4)
//            {
//                leftline_use[i] = leftline[i-4];
//            }
//            //左边线切入线原始图像补齐
//            left_ck = (float)(leftline[left_break] - leftline[left_break+4])/(left_break-(left_break+4));
//            for(i=left_break;i>20;i--)
//            {
//                for(j=leftline[left_break];j<160;j++)
//                {
//                    if(((j-leftline[left_break+4])/(i-(left_break+4)))/left_ck >0.9 &&
//                            ((j-leftline[left_break+4])/(i-(left_break+4)))/left_ck < 1.1)
//                    {
//                        leftline_use[i] = j;
//                    }
//                }
//            }
//        }
//    }
//    //圆环补线,use组数据修正
//       if(in_cur_flag != 0)
//       {
//           if(right_break != 120 || left_break !=120)
//           {
//               if(right_break < 120)
//               {
//                   right_ck = (float)(str_curlie - rightline[right_break])/(str_curhang-right_break);
//                   for(i=right_break;i>str_curhang;i--)
//                   {
//                       for(j=rightline[right_break];j>str_curlie;j--)
//                       {
//                           if(((j-rightline[right_break])/(i-right_break))/right_ck >0.9 &&
//                                   ((j-rightline[right_break])/(i-right_break))/right_ck < 1.1)
//                           {
//                               rightline_use[i] = j;
//                           }
//                       }
//                   }
//               }
//               if(left_break<120)
//               {
//                   left_ck = (float)(str_curlie - leftline[left_break])/(str_curhang-left_break);
//                   for(i=left_break;i>str_curhang;i--)
//                   {
//                       for(j=leftline[left_break];j<str_curlie;j++)
//                       {
//                           if(((j-leftline[left_break])/(i-left_break))/left_ck >0.9 &&
//                                   ((j-leftline[left_break])/(i-left_break))/left_ck < 1.1)
//                           {
//                               leftline_use[i] = j;
//                           }
//                       }
//                   }
//               }
//               if(tangent_expoint(str_curhang,str_curlie,in_cur_flag))//尝试找出外圆切入点
//               {
//                   //进入该分支说明切入点补线已经完成
//                   //可以由use组计算得中线了
//               }
//           }
//           if(right_break == 120 && left_break == 120)
//           {
//               Shin();
//           }
//
//       }
//
//
//
//
//   /************************************边线位置数据继承****************************************************/
//       //常规补线，用于非特殊道路特征补线
//       if(right_break != 120)
//       {
//           //右边线补线
//           right_ck = (float)(rightline_use[right_break] - rightline_use[right_break+4])/(right_break-(right_break+4));
//           for(i=right_break;i>20;i--)
//           {
//               for(j=rightline_use[right_break];j>rightline_use[right_break]-10;j--)
//               {
//                   if(((j-rightline_use[right_break])/(i-right_break))/right_ck >0.9 &&
//                           ((j-rightline_use[right_break])/(i-right_break))/right_ck < 1.1)
//                   {
//                       rightline_use[i] = j;
//                   }
//               }
//           }
//       }
//       if(left_break != 120)
//       {
//           //左边线补线
//           left_ck = (float)(leftline_use[left_break] - leftline_use[left_break+4])/(left_break-(left_break+4));
//           for(i=left_break;i>20;i--)
//           {
//               for(j=leftline_use[left_break];j>leftline_use[left_break]-10;j--)
//               {
//                   if(((j-leftline_use[left_break])/(i-left_break))/left_ck >0.9 &&
//                           ((j-leftline_use[left_break])/(i-left_break))/left_ck < 1.1)
//                   {
//                       leftline_use[i] = j;
//                   }
//               }
//           }
//       }
//
//}
//    /*****************************以上圆环处理********************************/
//
//
//
//void Feature_Way(void)
//{
///****************************************************************以下是各种道路特征判断*************************************************/
//
//    unsigned char i,j = 0;//循环用变量
//    //边线检查，采用边线的跳变实现空缺道路特征的判断
//    for(i=120-1;i>20+4;i-=4)
//    {
//        if((abs((int)((leftline[i-4]-leftline[i])/(leftline[i]-leftline[i+4])))>10) && left_break == 120)
//        {
//            left_break = i;
//            break;
//        }
//        else
//            left_break = 120;
//        if((abs((int)((rightline[i-4]-rightline[i])/(rightline[i]-rightline[i+4])))<10) && right_break == 120)
//        {
//            right_break = i;
//            break;
//        }
//        else
//            right_break  = 120;
//    }
//    //左右边线转折检查
//    Line_ck(1);
//    Line_ck(-1);
//
//    //    //根据跳变情况判断
//       //    if(!left_break && right_break && !in_tri_flag && !upfill_flag)//判断进入圆环
//       //    {
//       //        in_cur_flag = 1;
//       //    }
//       //    else if(left_break && right_break && !in_cur_flag)//判断进入三叉和坡道
//       //    {
//       //        for(i=120;i>20;i++)
//       //        {
//       //            if(Bin_Image[i][midline[i]] == 1)
//       //            {
//       //                mid_break = i;
//       //                break;
//       //            }
//       //        }
//       //        if(mid_break)//判断为三叉
//       //        {
//       //            in_tri_flag = 1;
//       //        }
//       //        else         //判断为坡道
//       //            upfill_flag = 1;
//       //
//
//
//
//
//    //检查圆环特征，set——line用于圆环补线，识别圆环重点在于内圆环的检测，这里识别出圆环特征之后记录内圆环贴近点
//        //基于原始左中右图像数据
//    unsigned char counter = 0;
//    unsigned char line_ex[2] = {0,0};
//    if(right_break!=120 || left_break!=120)
//    {
//        if(right_break < 120)
//        {
//            for(i=right_break;i>20;i--,counter++)
//            {
//                for(j=rightline[right_break];j<160-1;j++)
//                {
//                    if(Bin_Image[i][j+1] - Bin_Image[i][j] == 1)
//                    {
//                        line_ex[1] = line_ex[0];
//                        line_ex[0] = j;
//                        if(!line_down && line_ex[0] - line_ex[1] < 0)
//                        {
//                            line_down = 1;
//                        }
//                        else if(line_down == 1 && line_ex[0] - line_ex[1] > 0)
//                        {
//                            line_down = line_ex[1];//对应hang信息 right_break+counter
//                            str_curlie = line_down;
//                            str_curhang = right_break+counter;
//                            line_down = 0;
//                            in_cur_flag = 1;//you圆flag
//                        }
//                        else if(!line_down && line_ex[0] - line_ex[1] > 0 && line_ex[1] == 0)
//                        {
//                            str_curlie = j;
//                            str_curhang = i;
//                        }
//                    }
//                }
//            }
//        }
//        else if(left_break < 120)
//        {
//            for(i=left_break;i>20;i--,counter++)
//            {
//                for(j=leftline[left_break];j>1;j--)
//                {
//                    if(Bin_Image[i][j-1] - Bin_Image[i][j] == 1)
//                    {
//                        line_ex[1] = line_ex[0];
//                        line_ex[0] = j;
//                        if(!line_down && line_ex[0] - line_ex[1] < 0)
//                        {
//                            line_down = 1;
//                        }
//                        else if(line_down == 1 && line_ex[0] - line_ex[1] > 0)
//                        {
//                            line_down = line_ex[1];//对应hang信息 right_break+counter
//                            str_curlie = line_down;
//                            str_curhang = left_break+counter;
//                            line_down = 0;
//                            in_cur_flag = 2;//左圆flag
//                        }
//                    }
//                }
//            }
//        }
//    }
//}
//
//
//
///*
// * 圆环切入时的顶部点的位置信息
// *
// * 原先该函数类型void但是考虑到该点对于小车位置信息的提示作用
// * 设置为unsigned char 一次判断小车是否已经拐进圆环
// * 返回值位置是否合适，有待后期整体调整
// *
// * 摄像头位置要求，判定圆环之前切入点进入屏幕范围
// */
//unsigned char tangent_expoint(unsigned char hang, unsigned char lie,unsigned char x)
//{
//    static unsigned char i,j,m,n,s;
//    if(x == 1)
//    {
//        for(i=hang-1;i>0;i--)
//        {
//            if(i != 0 && Bin_Image[i][lie] == 1)
//                hang = i;
//            else if(i == 0) hang = 0;
//        }
//        for(i=hang;!tangent_exlie;i+=5)
//        {
//            for(j=lie;j>0;j--)//j>midline_use[hang]
//            {
//                if(Bin_Image[i+5][j-1] - Bin_Image[i+5][j] == 1)
//                {
//                    for(s=j;s>0;s--)
//                    {
//                        if(Bin_Image[i+8][s-1] - Bin_Image[i+8][s] == 1)
//                        {
//                            if(j-s < 20) continue;
//                            else
//                            {
//                                tangent_exhang = i;
//                                tangent_exlie = j;
//                                //找到切入点后，根据视野情况完成补线,
//                                left_ck = (float)(tangent_exlie - leftline[str_curhang])/(tangent_exhang - str_curhang);
//                                for(m=str_curhang;m>20;m--)
//                                {
//                                    for(n=leftline[str_curhang];n<150;n++)
//                                    {
//                                        //左边线切入线原始图像补齐
//                                        if(((n-leftline[str_curhang])/(m-str_curhang))/left_ck<1.1 &&
//                                                ((n-leftline[str_curhang])/(m-str_curhang))/left_ck>0.9)
//                                        {
//                                            leftline_use[m] = n;
//                                        }
//                                    }
//                                }
//                                return 1;
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        return 0;
//    }
//    //左圆的相应检查，未修改
////    if(x == 2)
////    {
////        for(i=hang-1;i>0;i--)
////        {
////            if(i != 0 && Bin_Image[i][lie] == 1)
////                hang = i;
////            else if(i == 0) hang = 0;
////        }
////        for(i=hang;!tangent_exlie;i+=5)
////        {
////            for(j=lie;j>0;j--)//j>midline_use[hang]
////            {
////                if(Bin_Image[i+5][j-1] - Bin_Image[i+5][j] == 1)
////                {
////                    for(s=j;s>0;s--)
////                    {
////                        if(Bin_Image[i+8][s-1] - Bin_Image[i+8][s] == 1)
////                        {
////                            if(j-s < 20) continue;
////                            else
////                            {
////                                tangent_exhang = i;
////                                tangent_exlie = j;
////                                return 1;
////                            }
////                        }
////                    }
////                }
////            }
////        }
////        return 0;
////    }
//    return 0;
//}
//

//
#include<stdlib.h>
//char line_back(char x,unsigned char hang,unsigned char lie);

uint8_t shin_feature(uint8_t imageInput[LCDH][LCDW], uint8_t image_l[80], uint8_t image_r[80], uint8_t image_m[80])
{
    uint8_t i=0,j=0;
    //边线初始化

        for(i=110;i>30;i--)
        {
            if(i==110) j = 79;
            else j = image_m[110-i-1];

            for(;j>=0;j--)
            {
                if(!imageInput[i][j])
                {
                    image_l[110-i] = j;
                    break;
                }
                else
                {
                    if(j>0)
                        continue;
                    else image_l[110-i] = 0;
                }
            }

            if(i==110) j = 79;
            else j = image_m[110-i-1];

            for(;j<160;j++)
            {
                if(!imageInput[i][j])
                {
                    image_r[110-i] = j;
                    break;
                }
                else
                {
                    if(j<159)
                        continue;
                    else image_r[110-i] = 159;
                }
            }
            image_m[110-i] = (image_l[110-i] + image_r[110-i] + 2)/2 - 1;
            if(image_m[110-i] == 1 || image_m[110-i] == 158)
            {
                stop_pos = i;
                break;
            }
        }
        return 0;
}






/*
 * 边线攀爬程序（左）,这部分日后优化待用（可以删了，
 * 目前思路是由起始行获取起始中心线的位置信息，
 * 然后计算已有中线部分的导数信息获取中心线延伸方向以及法向信息，
 * 避免应该可以避免左右边线位置的重复使用，避免弯道，圆环的图像拉伸导致的偏差问题）
 * 暂时用二维数组，三维数组有待以后偏差情况再考虑
 */
//
//void Shin(void)
//{
//    //首先寻找起始点，由于靠近小车的位置，中线大概率在车道上，所以由临近点引出左边线攀爬起始点
//    unsigned char i,j;
//    if(right_break != 119 || left_break != 119)//如果已经有中心线继承，跳过原始起始边线获取
//    {
//        if(right_break != 119)
//        {
//            for(i=119;i>115;i--)
//            {
//                rightline[i] = rightline_use[i];
//            }
//        }
//        if(left_break != 119)
//        {
//            for(i=119;i>115;i--)
//            {
//              leftline[i] = leftline_use[i];
//            }
//
//        }
//    }
//    else if(right_break == 119 || left_break == 119)
//    {
//        if(right_break ==119)
//        {
//            for(i=119;i>115;i--)//找到起始部分的右线位置
//            {
//                for(j=80;j+1<160;j++)
//                {
//                    if(Bin_Image[i][j+1] - Bin_Image[i][j] == 1)
//                    {
//                        rightline[i] = j+1;
//                        break;
//                    }
//                    else
//                        rightline[i] = 160;
//                }
//            }
//        }
//        if(left_break ==119)
//        {
//            for(i=120-1;i>=115;i--)//找到起始部分的左线位置
//            {
//                for(j=80;j-1>0;j--)
//                {
//                    if(Bin_Image[i][j-1] - Bin_Image[i][j] == 1)
//                    {
//                        leftline[i] = j-1;
//                        break;
//                    }
//                    else
//                        leftline[i] = 0;
//                }
//            }
//        }
//    }
//    for(i=120-1;i>=115;i--)//取得起始中线信息
//    {
//        midline[i] = (leftline[i] + rightline[i])/2;
//    }
//    //由起始部分开始攀爬,
//    //考虑用曲率会更精确一点（还未验证）
////    unsigned char m,n,p,q;
////    float search_k = 0;
////    float shin_k=0;
////    float shin_ck=0;
////    unsigned char righthang_last[3] = {0};
////    unsigned char lefthang_last[3] = {0};
////    for(i=120-1;i>28; )
////    {
////        if(i == 119)
////        {
////            righthang_last[2] = i;
////            righthang_last[1] = i-4;
////            lefthang_last[2] = i;
////            lefthang_last[1] = i-4;
////        }
////        else
////        {
////            righthang_last[2] = righthang_last[1];
////            righthang_last[1] = righthang_last[0];
////            lefthang_last[2] = lefthang_last[1];
////            lefthang_last[1] = lefthang_last[0];
////        }
////        shin_k = (float)(midline[i-4] - midline[i])/(-4);//分子：列  分母：行（一定注意这一点）
////        shin_ck = (-1)/shin_k;
////        midline[i-8] = midline[i-4] + (unsigned char)shin_k * (-4);
////        if(shin_k < 0 && shin_k > -20)//曲线右偏
////        {
////            //其中穿插道路特征检查
////            for(m=i-8+1;m>119;m++)//右边线hang开始
////            {
////                for(n=midline[i-8];n<160;n++)//列循环
////                {
////                    search_k = (float)(n - midline[i-8])/(m - (i-8));
////                    if(search_k/shin_ck < 1.1 && search_k/shin_ck > 0.9)
////                    {
////                        if(Bin_Image[m][n] == 1)
////                        {
////                            if(right_break == 119 || right_ret != 0)
////                            {
////                                rightline[m] = n;
////                                if((abs(((int)(rightline[m]-rightline[righthang_last[0]])/(rightline[righthang_last[0]]-rightline[righthang_last[1]])))<10) && right_break == 120)
////                                {
////                                    if(right_break == 119)
////                                        right_break = i;
////                                    rightline[righthang_last[0]-4] = rightline[righthang_last[0]] + (-4)*(float)(rightline[righthang_last[0]] - rightline[righthang_last[1]])/(righthang_last[0]-righthang_last[1]);
////                                    righthang_last[0] = righthang_last[0]-4;
////                                    line_back(1,righthang_last[0],rightline[righthang_last[0]-4]);
////                                }
////                                else
////                                    righthang_last[0] = m;
////                            }
////                            else
////                            {
////                                rightline[righthang_last[0]-4] = rightline[righthang_last[0]] + (-4)*(float)(rightline[righthang_last[0]] - rightline[righthang_last[1]])/(righthang_last[0]-righthang_last[1]);
////                                righthang_last[0] = righthang_last[0]-4;
////                                line_back(1,righthang_last[0],rightline[righthang_last[0]-4]);
////                            }
////                        }
////                        for(p=i-8-1;p>20;p--)//左边线hang开始
////                        {
////                            for(q=midline[i-8];q>0;q--)//列循环
////                            {
////                                search_k = (q - midline[i-8])/(float)(p - (i-8));
////                                if(search_k/shin_ck < 1.1 && search_k/shin_ck > 0.9)
////                                {
////                                    if(Bin_Image[p][q] == 1)
////                                    {
////                                        if(left_break == 119 || left_ret != 0)
////                                        {
////                                            leftline[p] = q;
////                                            if((abs((int)((leftline[p]-leftline[lefthang_last[0]])/(leftline[lefthang_last[0]]-leftline[lefthang_last[1]])))>10) && left_break == 119)
////                                            {
////                                                if(left_break == 119)
////                                                    left_break = i;
////                                                leftline[lefthang_last[0]-4] = leftline[lefthang_last[0]] + (-4)*(float)(leftline[lefthang_last[0]] - leftline[lefthang_last[1]])/(lefthang_last[0]-lefthang_last[1]);
////                                                lefthang_last[0] = lefthang_last[0]-4;
////                                                line_back(-1,lefthang_last[0],leftline[lefthang_last[0]-4]);
////                                            }
////                                            else
////                                                lefthang_last[0] = p;
////                                        }
////                                        else
////                                        {
////                                            leftline[lefthang_last[0]-4] = leftline[lefthang_last[0]] + (-4)*(float)(leftline[lefthang_last[0]] - leftline[lefthang_last[1]])/(lefthang_last[0]-lefthang_last[1]);
////                                            lefthang_last[0] = lefthang_last[0]-4;
////                                            line_back(-1,lefthang_last[0],leftline[lefthang_last[0]-4]);
////                                        }
////                                        midline[(righthang_last[0]+lefthang_last[0])/2] = (rightline[righthang_last[0]] + leftline[lefthang_last[0]])/2;//对应中心线
////                                        i=(righthang_last[0]+lefthang_last[0])/2;
////                                        //与上一个中心线点连线获取中间缺的位置信息，当然考虑到已经获取已知的中心线点的位置，其实可以直接取点获取偏差
////                                        //所以这里暂时不写，有空再搞
////                                    }
////                                }
////                            }
////                        }
////                    }
////                }
////            }
////        }
////        else if(shin_k > 0 && shin_k < 20)//曲线左偏
////        {
////            for(m=i-8-1;m>20;m--)//hang开始
////            {
////                for(n=midline[i-8];n<160;n++)//列循环
////                {
////                    search_k = (float)(n - midline[i-8])/(m - (i-8));
////                    //左边线
////                    if(search_k/shin_ck < 1.1 && search_k/shin_ck > 0.9)
////                    {
////                        if(Bin_Image[m][n] == 1)
////                        {
////                            if(right_break == 119 || right_ret != 0)
////                            {
////                                rightline[m] = n;
////                                if((abs((int)((rightline[m]-rightline[righthang_last[0]])/(rightline[righthang_last[0]]-rightline[righthang_last[1]])))<10) && right_break == 120)
////                                {
////                                    if(right_break == 119)
////                                        right_break = i;
////                                    rightline[righthang_last[0]-4] = rightline[righthang_last[0]] + (-4)*(float)(rightline[righthang_last[0]] - rightline[righthang_last[1]])/(righthang_last[0]-righthang_last[1]);
////                                    righthang_last[0] = righthang_last[0]-4;
////                                    line_back(1,righthang_last[0],rightline[righthang_last[0]-4]);
////                                }
////                                else
////                                    righthang_last[0] = m;
////                            }
////                            else
////                            {
////                                rightline[righthang_last[0]-4] = rightline[righthang_last[0]] + (-4)*(float)(rightline[righthang_last[0]] - rightline[righthang_last[1]])/(righthang_last[0]-righthang_last[1]);
////                                righthang_last[0] = righthang_last[0]-4;
////                                line_back(1,righthang_last[0],rightline[righthang_last[0]-4]);
////                            }
////                        }
////                        for(p=i-8+1;p>20;p--)//hang开始
////                        {
////                            for(q=midline[i-8];q>0;q--)//列循环
////                            {
////                                search_k = (q - midline[i-8])/(float)(p - (i-8));
////                                if(search_k/shin_ck < 1.1 && search_k/shin_ck > 0.9)
////                                {
////                                    if(Bin_Image[p][q] == 1)
////                                    {
////                                        if(left_break == 119 || left_ret != 0)
////                                        {
////                                            leftline[p] = q;
////                                            if((abs((int)((leftline[i-4]-leftline[i])/(leftline[i]-leftline[i+4])))>10) && left_break == 120)
////                                            {
////                                                if(left_break == 119)
////                                                    left_break = i;
////                                                leftline[lefthang_last[0]-4] = leftline[lefthang_last[0]] + (-4)*(float)(leftline[lefthang_last[0]] - leftline[lefthang_last[1]])/(lefthang_last[0]-lefthang_last[1]);
////                                                lefthang_last[0] = lefthang_last[0]-4;
////                                                line_back(-1,lefthang_last[0],leftline[lefthang_last[0]-4]);
////                                            }
////                                            else
////                                                lefthang_last[0] = p;
////                                        }
////                                        else
////                                        {
////                                            leftline[lefthang_last[0]-4] = leftline[lefthang_last[0]] + (-4)*(float)(leftline[lefthang_last[0]] - leftline[lefthang_last[1]])/(lefthang_last[0]-lefthang_last[1]);
////                                            lefthang_last[0] = lefthang_last[0]-4;
////                                            line_back(-1,lefthang_last[0],leftline[lefthang_last[0]-4]);
////                                        }
////                                        midline[(righthang_last[0]+lefthang_last[0])/2] = (rightline[righthang_last[0]] + leftline[lefthang_last[0]])/2;//对应中心线
////                                        i=(righthang_last[0]+lefthang_last[0])/2;
////                                        //与上一个中心线点连线获取中间缺的位置信息，当然考虑到已经获取已知的中心线点的位置，其实可以直接取点获取偏差
////                                        //所以这里暂时不写，有空再搞
////                                    }
////                                }
////                            }
////                        }
////                    }
////                }
////            }
////        }
////        else if(shin_k > 20 || shin_k < -20)
////            break;
////  }
//}
//
//简单的边线回归检查
//char line_back(char x,unsigned char hang,unsigned char lie)
//{
//    if(x>0)//向右
//    {
//        if(Bin_Image[hang][lie] == 1) {right_ret = lie; return 1;}
//        else if(Bin_Image[hang][lie+1] - Bin_Image[hang][lie] == 1) {right_ret = lie+1; return 1;}
//        else if(Bin_Image[hang][lie+2] - Bin_Image[hang][lie+1] == 1) {right_ret = lie+2; return 1;}
////        else if(Bin_Image[hang][lie+3] - Bin_Image[hang][lie+2] == 1) {right_ret = lie+3; return 1;}
//    }
//    else if(x<0)//向左
//    {
//        if(Bin_Image[hang][lie] == 1) {left_ret = lie; return 1;}
//        else if(Bin_Image[hang][lie-1] - Bin_Image[hang][lie] == 1) {left_ret = lie-1; return 1;}
//        else if(Bin_Image[hang][lie-2] - Bin_Image[hang][lie-1] == 1) {left_ret = lie-2; return 1;}
////        else if(Bin_Image[hang][lie-3] - Bin_Image[hang][lie-2] == 1) {left_ret = lie-3; return 1;}
//    }
//
//    return 0;
//}
//
//unsigned char lt_hang = 0;
//unsigned char lt_lie = 0;
//unsigned char rt_hang = 0;
//unsigned char rt_lie = 0;
////边线斜率自下而上检查
//unsigned char Line_ck(char f)
//{
//    unsigned char i;
//    double r_k[2];
//    double l_k[2];
//    if(f < 0)//zuo边线检查
//    {
//        for(i=120-1;i>20+4;i-=4)
//        {
//            l_k[1] = l_k[0];
//            l_k[0] = (-4.0)/(leftline[i-4] - leftline[i]);
//            if(l_k[1]*l_k[0] < 0)
//            {
//                lt_hang = i;
//                lt_lie = leftline[i];
//                return 1;
//            }
//            else continue;
//        }
//        return 0;
//    }
//    else if(f > 2)//右边线检查
//    {
//        for(i=120;i>20+4;i-=4)
//        {
//            r_k[1] = r_k[0];
//            r_k[0] = (-4)/(float)(rightline[i-4] - rightline[i]);
//            if(r_k[1]*r_k[0] < 0)
//            {
//                rt_hang = i;
//                rt_lie = rightline[i];
//                return 1;
//            }
//            else continue;
//        }
//        return 0;
//    }
//    else return 0;
//}
//
/*************************************************************************
 *  函数名称：void CameraCar(void)
 *  功能说明：电磁车双电机差速控制
 -->1.入门算法：简单的分段比例控制算法，教学演示控制算法；
 2.进阶算法：PID典型应用控制算法，教学演示控制算法；
 3.高端算法：改进粒子群协同控制算法；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年10月28日
 *  备    注：驱动2个电机
 *************************************************************************/
static uint8_t g_ucFlagRoundabout_flag = 0;

//环岛元素处理，其中自行修改得参数与小车的速度，误差放大比例有关，需要同学们自己根据实际情况来修改
void CameraCar (void)
{
    LED_Ctrl(LED1, RVS);     // LED闪烁 指示程序运行状态
    uint8_t pointY;

     if(g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0)
     {
         // 检测环岛
         RoadIsRoundabout(UpdowmSide, Bin_Image, ImageSide, &g_ucFlagRoundabout);
     }
    if(g_ucFlagRoundabout)
     {
        g_ucFlagRoundabout_flag = 1;
       //   环岛处理

        Servo_P = 12;           //误差放大
        RoundaboutProcess(Bin_Image, ImageSide, UpdowmSide, &g_ucFlagRoundabout);

     }

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
//      //十字部分未用到
//     if(g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagFork == 0)
//     {
//         /* 检测十字 */
//         RoadIsCross(ImageSide, &g_ucFlagCross);
//     }
//     if(g_ucFlagCross)
//     {
//         /* 十字处理 */
//         CrossProcess(Image_Use, ImageSide, &g_ucFlagCross);
//     }

    /********************************************************************************************/
    /********************************T形路口**********************************************/

    if(g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0)
    {
        //检查T字
        RoadIsT(UpdowmSide, ImageSide, &g_ucFlagT);
    }
    if(g_ucFlagT)
    {
        Servo_P = 12;
        //T字处理
        TProcess(Bin_Image, UpdowmSide, ImageSide, &g_ucFlagT);
    }

     /************************************************************************
       2021/7/19测试代码  Y形路口
       ************************************************************************/
     if(g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0)
       {
           RoadIsFork(UpdowmSide, ImageSide, &g_ucFlagFork, &pointY);
       }
     if(g_ucFlagFork == 1)
     {
         g_ucForkNum += 1;
     }

       if(g_ucFlagFork)//遇到岔口
       {

           Servo_P = 10;
           // Y字处理
           ForkProcess(UpdowmSide, ImageSide, &g_ucFlagFork);
       }
       ////////////////////////车库识别代码///////////////////////////
       ////////////////////////这部分未修改///////////////////////////
     //  if(g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0&& g_ucFlagFork == 0)
     //  {
           /* 检测车库 */
     //      RoadIsCross(ImageSide, &g_ucFlagZebra);
     //  }
     //  if(g_ucFlagZebra)
     //  {
     //      ZebraProcess(Image_Use,1,1200);
     //  }


     /* 根据主跑行，求取舵机偏差 */
     g_sSteeringError = RoadGetSteeringError(ImageSide, ROAD_MAIN_ROW);
     //偏差放大
     ServoDuty = g_sSteeringError * Servo_P/10;
     //偏差限幅
     if(ServoDuty>170) ServoDuty=170;
     if(ServoDuty<-170) ServoDuty=-170;
     //舵机打角
     //ServoCtrl(Servo_Center_Mid-ServoDuty);
}
