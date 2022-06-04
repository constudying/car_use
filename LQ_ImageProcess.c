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

/**  @brief    ת�����  */
sint16 g_sSteeringError = 0;
/**  @brief    ���߱�־λ  */
uint8_t g_ucIsNoSide = 0;

/**  @brief    ������  */
#define ROAD_MAIN_ROW      40

/**  @brief    ʹ����ʼ��  */
#define ROAD_START_ROW     115

/**  @brief    ʹ�ý�����  */
#define ROAD_END_ROW       10


/**  @brief    ������־λ  */
uint8_t g_ucFlagRoundabout  = 0;

/**  @brief    ʮ�ֱ�־λ  */
uint8_t g_ucFlagCross  = 0;

/**  @brief    �����߱�־λ  */
uint8_t g_ucFlagZebra  = 0;

/**  @brief    Y�Ͳ�ڱ�־λ  */
uint8_t g_ucFlagFork  = 0;
uint8_t g_ucForkNum  = 0;

/**  @brief    T�Ͳ�ڱ�־λ  */
uint8_t g_ucFlagT = 0;

pid_param_t BalDirgyro_PID;  // ����PID

uint8_t Servo_P = 12;



char txt[30];
/*!
  * @brief    ������
  *
  * @param
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/28 ������
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
  * @brief    �ж��ϱ����Ƿ񵥵�
  * @param    X1 :��ʼX��
  * @param    X2 :��ֹX��              X1 < X2
  * @param    imageIn �� ��������
  *
  * @return   0��������or���� 1������������ 2�������ݼ�
  *
  * @note
  *
  * @see
  *
  * @date     2021/11/30 ���ڶ�
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
  * @brief    �ж��Ƿ���ֱ��
  *
  * @param    image �� ��ֵͼ����Ϣ
  *
  * @return   0������ֱ���� 1��ֱ��
  *
  * @note     ˼·�����߱��߶�����
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2])
{
    uint8_t i = 0;
    uint8_t leftState = 0, rightState = 0;

    /* ������Ƿ񵥵� */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][0] + 5 < imageSide[i+1][0])
        {
            leftState = 1;
            break;
        }
    }

    /* �ұ����Ƿ񵥵� */
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
  * @brief    �ж��Ƿ��ǰ�����
  *
  * @param    image �� ��ֵͼ����Ϣ
  *
  * @return   0�����ǣ� 1����
  *
  * @note     ˼·��
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
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
  * @brief    �ж��Ƿ���T��
  *
  * @param    imageSide �� ͼ�������Ϣ
  * @param    flag      �� T��״̬��Ϣ
  *
  * @return   0�����ǣ� 1����
  *
  * @note     ˼·������0-80�������� 80-159������ ��������һ���󻡣��ұ���ȫ���������115-50������
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsT(uint8_t imageUp[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t *flag)
{
    uint8_t i = 0;
    uint8_t errU1 = 0, errU2 = 0, errR1 = 0, errL1 = 0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t count = 0, num = 0, py;
    uint8_t index = 0;

    /* ��������߾��복ͷ�����ж��� --  */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][1] == 159)
            num++;
        if(num >= 130)
        {
            rightState = 1;//��Ϊ����
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
            leftState = 1;   // ��hu��־
            break;
        }
    }
    errL1 = RoundaboutGetArc(imageSide, 1, 5, &py);    //�����л�
    errR1 = RoundaboutGetArc(imageSide, 2, 5, &py);    //�����л�
    errU1 = RoadUpSide_Mono(10, 70, imageUp);       //�ϵ�����
    errU2 = RoadUpSide_Mono(80, 150, imageUp);     //�ϵ�����
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
            //����ȷ���ϱ���
            Roundabout_Get_UpDowmSide(imageInput, imageUp, 1);
            errL1 = RoundaboutGetArc(imageSide, 1, 5, &py);    //�����л�
            errU1 = RoadUpSide_Mono(10, 140, imageUp);      //�ϵ�����

            if(errU1 == 2 && errL1 == 0)
                *flag = 2;

            //������ת������ת��뾶
            ImageAddingLine(imageSide, 1, 90, 30, 0, ROAD_START_ROW);
            break;

        case 2:
            errU2 = RoundaboutGetArc(imageSide, 2, 5, &py);//����ұ����Ƿ��л�
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
            ImageAddingLine(imageSide, 2, 60, 30, 159, ROAD_START_ROW);//���������޸�
            break;
    }
    return 0;
}

/*!
  * @brief    �ж��Ƿ���ʮ��
  *
  * @param    imageSide �� ͼ�������Ϣ
  * @param    flag      �� ʮ��״̬��Ϣ
  *
  * @return   0�����ǣ� 1����
  *
  * @note     ˼·���������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� ��֤����ʮ��
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsCross(uint8_t imageSide[LCDH][2], uint8_t *flag)
{
    int i = 0;
    uint8_t errR = 0, errF = 0;
    uint8_t  rightState = 0, leftState = 0;
    int start[5] = {0, 0, 0, 0, 0}, end[5] = {0, 0, 0, 0, 0};
    uint8_t count = 0;
    uint8_t index = 0;

    /* ����Ҳ���߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
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

    /* ��������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
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
        //�����Ƿ���ͻ��
        for(i = 159-1; i > 0; i--)
        {
          if(UpdowmSide[0][i] != 1 && UpdowmSide[0][i+1] != 1)
          {
              if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                  index++;
              else
                  count++;
              /* �л��� */
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
    /* �ӳ�ͷ��ǰ ������Ƿ񵥵� */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(image[i][0] == 0)
            continue;
        if(image[i][0] >= image[i+1][0])    // i��Y����ֵ  0 ��ͼ������X����
        {
            num++;
            if(num == 50)
            {
                num = 0;
                leftState = 1;   // �󵥵���־
                break;
            }
        }
        else
        {
            num = 0;
        }
        if(i == ROAD_END_ROW+1)  // Y�ӵ�11  ��0
            num = 0;
    }
    errL = RoundaboutGetArc(image, 1, 5, &py);
    errR = RoundaboutGetArc(image, 1, 5, &py);

    /* �ұ����Ƿ񵥵� */
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

    /* ��ߵ����� ����Ҳ��Ƿ��ǻ��� */
    if(leftState == 1 && rightState == 0 && errL == 0)
    {
        count = 0;

        if(RoundaboutGetArc(image, 2, 5, &count))//��Բ����� (5�������� �� 5��������)
        {
            *flag = 1;
            return 1;
        }
        else
        {
            return 0;
        }
    }

    /* �ұߵ����� �������Ƿ��ǻ��� */
    if(rightState == 1 && leftState == 0)
    {
        count = 0;
        if(RoundaboutGetArc(image, 1, 5, &count))//��Բ����� (5�������� �� 5��������)
        {
            *flag = 2;
            return 2;
        }
    }
    return 0;
}
/*!
  * @brief    ��ȡ��������
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    status     �� 1���󻷵�(����)  2���һ���(����)
  *
  * @return
  *
  * @note     ˼·������һ�߱����ϸ񵥵�������һ�߱��ߣ���ȡ��һ����
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
  */
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t status)
{
    uint8_t i = 0, j = 0;

    switch(status)
    {

        /* �󻷵� */
      case 1:
        {
            /* ����ȷ����߽� */
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
            /* ����ȷ���ұ߽� */
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
            /* ����ȷ���ϱ߽� */
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
            /* ����ȷ���±߽� */
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
  * @brief    �жϱ����Ƿ���ڻ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    status     �� 1�������  2���ұ���
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
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

                /* �л��� */
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

                /* �л��� */
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
  * @brief    �жϱ����Ƿ���ڻ���
  *
  * @param    SideInput �� �ϱ�������
  * @param    num       �� ������
  * @param    index     �� ��͵�
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2021/12/01 ������
  */
uint8_t UpSideErr(uint8_t SideInput[2][LCDW],uint8_t status ,uint8_t num, uint8_t * index)
{
    uint8_t dec = 0, inc = 0, i;
    //�����Ƿ���ͻ��
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
                      /* �л��� */
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
        //�±���
        case 2:
            for(i = 159-1; i > 0; i--)
                {
                  if(UpdowmSide[1][i] != 1 && UpdowmSide[1][i+1] != 1)
                  {
                      if(UpdowmSide[1][i] >= UpdowmSide[1][i+1])
                          inc++;
                      else
                          dec++;
                      /* �л��� */
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
  * @brief    ���ߴ���
  *
  * @param    imageSide  : ����
  * @param    status     : 1������߲���   2���ұ��߲���
  * @param    startX     : ��ʼ�� ����
  * @param    startY     : ��ʼ�� ����
  * @param    endX       : ������ ����
  * @param    endY       : ������ ����
  *
  * @return
  *
  * @note     endY һ��Ҫ���� startY
  *
  * @see
  *
  * @date     2020/6/24 ������
  */
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
    int i = 0;

    /* ֱ�� x = ky + b*/
    float k = 0.0f, b = 0.0f;
    switch(status)
    {
      case 1://����
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(i = startY; i < endY; i++)
            {
                imageSide[i][0] = (uint8_t)(k * i + b);
            }
            break;
        }

      case 2://�Ҳ���
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
  * @brief    Ѱ�������
  *
  * @param    imageSide   �� ��������
  * @param    status      ��1����߽�   2���ұ߽�
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
  */
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x, uint8_t *y)
{
    int i = 0;
    uint8_t px = 0, py = 0;
    uint8_t count = 0;
    switch(state)
    {
      case 1:
        /* Ѱ������� */
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
        /* Ѱ������� */
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
  * @brief    �������ߴ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageSide  �� ��������
  * @param    status     ��������־λ   ����Ϊ�һ�����ż��Ϊ�󻷵���0Ϊ�������󻷵�û���޸ģ�
  *
  * @return
  *
  * @note     ����ֻд���󻷵����һ�����ҿ��Բο��󻷵��Լ�����
  *
  * @see
  *
  * @date     2020/6/24 ������
  */
void RoundaboutProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t UpdowmSide[2][LCDW], uint8_t* state)
{
    uint8_t i = 0, err5 = 0;
    uint8_t pointX = 0, pointY = 0, inc = 0, dec = 0;
    uint8_t flag= 0, Down_flag = 0;
    static uint8_t finderr = 0, Up_flag = 0, err1 = 0;
    switch(*state)
    {
        /* �����һ��� �������ڴ����� */
      case 1:

        /* ����ȷ���ұ߽� */
        RoundaboutGetSide(imageInput, imageSide, 2);

        /* ��黡�� */
        err1 = RoundaboutGetArc(imageSide, 2, 5, &pointY);

        /* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
        if(err1)
        {
            pointX = imageSide[pointY][1];
//            UART_PutStr(UART0, "err\r\n");
//
//            /* ׼���뻷�� */
//            if((pointY + 10) > ROAD_MAIN_ROW)
//            {
//                * state = 3;
//            }
            //����
            ImageAddingLine(imageSide, 2, pointX, pointY, 159, ROAD_START_ROW);
            finderr = 1;
        }
        else
        {
            if(finderr)
                *state = 3;//׼�����뻷��
        }

        break;

        /* �����󻷵� �������ڴ����� */
      case 2:

          /* ����ȷ����߽� */
          RoundaboutGetSide(imageInput, imageSide, 1);

          /* ��黡�� */
          err1 = RoundaboutGetArc(imageSide, 1, 5, &pointY);

          /* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
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


        /* ׼�����뻷���� ���� */
      case 3:
        /* ����ȷ���ϱ߽� */
        Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
        pointY = 0;
        pointX = 0;

        /* �ϱ߽���͵� */
        for(i = 40; i < 100; i++)
        {
            if(UpdowmSide[0][i] > pointY)
            {
                pointX = i;
                pointY = UpdowmSide[0][i];
            }
        }
        if(pointY >= 50)//��͵����50�������Լ�ʵ������޸ģ�
        {
            if(RoadUpSide_Mono(5, 100,UpdowmSide) == 1)//���ߵ�����������һ��
                *state = 5;
            ImageAddingLine(imageSide, 1, 100+30, 40-10,0, ROAD_START_ROW);//���ߣ������޸ģ�
        }
        else
            ImageAddingLine(imageSide, 1, 60, 40-15,0, ROAD_START_ROW); //���ߣ����߽Ƕ������޸ģ�
        break;

      case 4:
          /* ����ȷ���ϱ߽� */
          Roundabout_Get_UpDowmSide(imageInput, UpdowmSide, 1);
          pointY = 0;
          pointX = 0;

          /* �ϱ߽���͵� */
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
        /* �������� ֱ�������� */
      case 5:
          flag = 0;
          /* ��黡�� */
          for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
          {
              if(imageSide[i][0] != 0 && imageSide[i+1][0] != 0)
              {
                  if(imageSide[i][0] >= imageSide[i+1][0])
                      inc++;
                  else
                      dec++;
                  /* �л��� */
                  if(inc > 10 && dec > 10)err5 = 1;//������10�����ķ��ȣ��������޸ģ�
              }
              else
              {
                  inc = 0;
                  dec = 0;
              }
          }

          //����Ϊ119
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

          //������ߵ�����
          flag = RoadUpSide_Mono(20, 155,UpdowmSide);

          if(flag && err5 && Down_flag)
          {
              *state = 7;
          }
          break;

          /* �������� ֱ�������� */
      case 6:
          flag = 0;
        /* ��黡�� */
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][1] != 159 && imageSide[i+1][1] != 159)
            {
                if(imageSide[i][1] > imageSide[i+1][1])
                    inc++;
                else
                    dec++;
                /* �л��� */
                if(inc > 8 && dec > 8)err5 = 1;
            }
            else
            {
                inc = 0;
                dec = 0;
            }
        }

        //����Ϊ119
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

        //������ߵ�����
        flag = RoadUpSide_Mono(20, 155,UpdowmSide);

        if(flag && err5 && Down_flag)
        {
            *state = 8;
//                  ImageAddingLine(imageSide, 1, 145, 30,0, ROAD_START_ROW);
        }
        break;
        //����
      case 7:

          ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW);//���������޸�

          //�ж������Ƿ���ͻ��
            for(i = 159-1; i > 0; i--)
            {
                if(UpdowmSide[0][i] != 0 && UpdowmSide[0][i+1] != 0){
                    if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                        inc++;
                    else
                        dec++;
                    if(inc > 20 && dec > 20){
                        finderr = 0; Up_flag = 0; err1 = 0; //��վ�̬�����Ա��´�ʹ��
//                        Target_Speed1 = 25;               //�ٶȻظ�
//                        Target_Speed2 = 25;
//                        Servo_P = 18;                     //ת��ظ�
                        *state = 0;                         //��������
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
                      finderr = 0; Up_flag = 0; err1 = 0; //��վ�̬�����Ա��´�ʹ��
                      Servo_P = 15;                      //ת��ظ�
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
  * @brief    ��ȡʮ�ֱ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  *
  * @return
  *
  * @note     ˼·�����м�����������
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
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
  * @brief    ʮ�ֲ��ߴ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageSide  �� ��������
  * @param    status     ��ʮ�ֱ�־λ   1������ʮ��    2������ʮ��   3����ʮ��
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
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
            /* ���»�ȡ���� */
            CrossGetSide(imageInput, imageSide);

            /* Ѱ������� */
            if(ImageGetHop(imageSide, 1, &pointX, &pointY))
            {
                /* ���� */
                ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
            }

            leftIndex = pointY;
            pointX = 0;
            pointY = 0;

            /* Ѱ������� */
            if(ImageGetHop(imageSide, 2, &pointX, &pointY))
            {
                /* ���� */
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

            /* ��黡�� */
            if(RoundaboutGetArc(imageSide, 1, 5, &leftIndex))
            {
                /* ����ȷ����߽� */
                RoundaboutGetSide(imageInput, imageSide, 1);

                if(ImageGetHop(imageSide, 1, &pointX, &pointY))
                {
                    /* ���� */
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

            /* ����ȷ����߽� */
            RoundaboutGetSide(imageInput, imageSide, 1);


            if(ImageGetHop(imageSide, 1, &pointX, &pointY))
            {
                /* ��黡�� */
                if(RoundaboutGetArc(imageSide, 1, 5, &leftIndex))
                {
                    /* ���� */
                    ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
                }
                else
                {
                    /* ���� */
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
  * @brief    �ж��Ƿ���Y�Ͳ��
  *
  * @param    imageSide �� ͼ�������Ϣ
  * @param    flag      �� Y��״̬��Ϣ
  *
  * @return   0�����ǣ� 1����
  *
  * @note     ˼·�����߳ɻ�
  *
  * @see
  *
  * @date     2021/12/8 ������
  */
uint8_t RoadIsFork(uint8_t imageInput[2][LCDW],uint8_t imageSide[LCDH][2], uint8_t *flag, uint8_t * pY )
{

    uint8_t i = 0, errR = 0, errF = 0;
    uint8_t inc = 0, dec = 0, num = 0;
    uint8_t pointY;

    /* ��黡�� */
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
        //�ж������Ƿ��л� ���Ƽ��жϻ��������ְ취�����Լ���װ��һ��������֮ǰ�İ취��һ���ľ����ԣ����Լ��滻��
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
                  /* �л��� */
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
  * @brief    Y�ֲ��ߴ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageSide  �� ��������
  * @param    status     ��ʮ�ֱ�־λ   1������ʮ��    2������ʮ��   3����ʮ��
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
  */

sint32 RAllFork = 0;
void ForkProcess(uint8_t UpSideInput[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t* state)
{

  uint8_t pointY, pointX;
  static uint8_t D_flag = 0, dou_flag;

    //���»�ȡ�ϱ���
    UpdownSideGet(Bin_Image, UpdowmSide);

    switch(*state)
    {
        case 1://�жϹյ� ����յ�
            UpSideErr(UpSideInput, 1, 15, &pointY);
            if((UpSideInput[0][pointY] > 30) || (D_flag))
            {
                ImageAddingLine(imageSide, 1, 110, 35, 0, ROAD_START_ROW);  // ��Ļ���½����յ㣨�������޸ģ�
                D_flag = 1;
            }
            if(D_flag == 1 && RoadUpSide_Mono(30, 150, UpSideInput) == 2)
            {
                    *state = 2;
            }
            break;
        case 2://�� ����

            if((dou_flag == 1) && (!RoundaboutGetArc(imageSide, 2, 5, &pointY)))
                *state = 3;
            if(RoundaboutGetArc(imageSide, 2, 5, &pointY))
                dou_flag = 1;
            break;
        case 3://�� ����
            ImageAddingLine(imageSide, 1, 100, 30, 0, ROAD_START_ROW);//�������޸�
            if(RoadUpSide_Mono(5, 90, UpSideInput)) //�жϳ��ڽ��������
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
  * @brief    ͣ���ߴ���
  *
  * @param    imageSide  �� ��������
  * @param    state      �� ͣ��״̬  1�����������   2���������Ҳ�
  * @param    speed      �� �ٶ�
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
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
  * @brief    ���������У���ȡ���ƫ��
  *
  * @param
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
  */
int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex)
{

    return imageSide[lineIndex][0] + imageSide[lineIndex][1] - 158;

}

/*!
  * @brief    �ж��Ƿ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    lineIndex  �� ��
  *
  * @return   0��û�ж���   1:��߶���  2���ұ߶���  3�� ���Ҷ�����   4������
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
  */
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t lineIndex)
{
    uint8_t state = 0;
    uint8_t i = 0;
    static uint8_t last = 78;

    imageOut[lineIndex][0] = 0;
    imageOut[lineIndex][1] = 159;
    /* �þ���С���ȽϽ����� �ж��Ƿ��� */
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
        /* ��߽綪�� */
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
        /* ���ұ߽綪�� */
        if(state == 1)
        {
            state = 3;
        }

        /* �ұ߽綪�� */
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
  * @brief    ���ߴ���
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    mode       �� �Ǳ߶��ߣ�   1����߶���  2���ұ߶���
  * @param    lineIndex  �� ��������
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2020/6/24 ������
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
  * @brief    ��ȡ����
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  *
  * @return   �Ƿ���
  *
  * @note     ˼·���Ӿ��복ͷ�Ͻ����п�ʼ���м�����������
  *
  * @see
  *
  * @date     2020/6/23 ���ڶ�
  */
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2])
{
    uint8_t i = 0, j = 0;

    RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);

    /* �복ͷ����40�� Ѱ�ұ��� */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        imageOut[i][0] = 0;
        imageOut[i][1] = 159;

        /* ���ݱ߽��������� Ѱ�ұ߽� */
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
        /* �����߽� ������������ �����Ƿ��Ҷ��� */
        if(imageOut[i][0] > (LCDW/2 - 10) && imageOut[i][1] >  (LCDW - 5))
        {
            /* �Ҷ��ߴ��� */
            RoadNoSideProcess(imageInput, imageOut, 2, i);

            if(i > 70)
            {
                imageOut[i][0] += 50;
            }
            return 1;
        }

        /* ����ұ߽� ������������ �����Ƿ����� */
        if(imageOut[i][1] < (LCDW/2 + 10) && imageOut[i][0] <  (5))
        {
            /* ���ߴ��� */
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
  * @brief    ��ȡ����
  *
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  *
  * @return   �Ƿ���
  *
  * @note     ˼·���Ӿ��복ͷ�Ͻ����п�ʼ���м�����������
  *
  * @see
  *
  * @date     2021/11/30 ���ڶ�
  */
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW])
{
    uint8_t i = 0, j = 0;
    uint8_t last = 60;

    imageOut[0][159] = 0;
    imageOut[1][159] = 119;
    /* �����߱ȽϽ����� �ж��Ƿ��� */
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

    /* �������� Ѱ�ұ��� */
    for(i = 80-1; i > 0; i--)
    {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* ���ݱ߽��������� Ѱ�ұ߽� */
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
    /*�������� Ѱ�ұ���*/
    for(i = 80+1; i < 159; i++)
        {
            imageOut[0][i] = 0;
            imageOut[1][i] = 119;

            /* ���ݱ߽��������� Ѱ�ұ߽� */
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
  * @brief    ����һ�����
  *
  * @param
  *
  * @return
  *
  * @note     ˼·�� �����������ڵ�9���㣬�����������ֵ�������õ�
  *
  * @see
  *
  * @date     2020/6/24 ������
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

            /* ������5�����Ǳ��� �����õ� ���Ե��������Ż��˲�Ч�� */
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
  * @brief    ��������
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
    static int scan_flag = 0;//����ѡ���ʶ��
    static unsigned lock_flag = 1;//����������ʶ��
    static int change_flag = 0;//�������������չ�ñ�ʶ��
    char txt[32];

    ServoCtrl(servo_mid);

    //K0���н���ѡ��
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
        /* ��ʾ��ֵ��ͼ����Ϣ */
            //����K2������Ļ���棬���ɻָ�
//            if(KEY_Read(KEY2)==0)
//            {
//                while(1)
//                {
//                    if(KEY_Read(KEY0)==0)
//                    {
//                        hang++;
//                    }
//
//                    TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);//��ʾ����
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
//                            TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);//��ʾ����
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
//                TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);                       //��ʾ����
//            }

            TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);  //��ʾ��Ե��ȡͼ��
//            TFTSPI_BinRoadSide(ImageSide);                                  //���ұ���
//            TFTSPI_BinRoad_UpdownSide(UpdowmSide);                          //���±���
            TFT_RoadSide((unsigned char *)rightline,(unsigned char *)leftline,(unsigned char *)midline);
//            TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 159, ROAD_MAIN_ROW, u16RED); //��������ʾ
            TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);                       //��ʾ����

//            TFT_line();                                 //���ұ���
//            TFTSPI_CLS(u16BLACK);                   // ����
//            TFTSPI_BinRoadSide(ImageSide);          //���ұ���
//            TFTSPI_BinRoad_UpdownSide(UpdowmSide);  //���±���
//            TFT_line();                                     //���ұ���

    }
    else if(scan_flag == 1)
    {
        if(KEY_Read(KEY1) == 0)
        {
            change_flag += 1;
            delayms(500);//��ʱ�ȴ�����ֹ������ĳ�������а�����������
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
            TFTSPI_P8X8Str(0, 1, "speed_pid", u16WHITE, u16BLACK);//�ٶȽ�����ʾ
            sprintf(txt ,"kp=%.2f",mot_kp);
            TFTSPI_P8X8Str(1, 2, txt, u16WHITE, u16BLACK);

            sprintf(txt ,"kd=%.2f",mot_kd);
            TFTSPI_P8X8Str(1, 3, txt, u16WHITE, u16BLACK);
            sprintf(txt ,"ki=%.2f",mot_ki);
            TFTSPI_P8X8Str(1, 4, txt, u16WHITE, u16BLACK);

            TFTSPI_P8X8Str(0, 6, "dire_pid", u16WHITE, u16BLACK);//���������ʾ
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
                    lock_flag = 1;//��������
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
            //��ʱ�ȴ�����ֹ������ĳ�������а�����������
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
                    lock_flag = 1;//��������
                    TFTSPI_P8X8Str(12, 0, "locking", u16WHITE, u16BLACK);
                    break;
            }
        }

//        MotorCtrl((sint32)(-motor),(sint32)(motor));
    }
    else if(scan_flag == 3)
    {

    }
//    //����K2������Ļ���棬���ɻָ�
//    if(KEY_Read(KEY2)==0) while(1);
//       /* ����ʱ���Դ����� */
//    if(KEY_Read(KEY0)==0)
//    {
//        //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (uint8_t*)Image_Use);        //ͼ����ʾ
//        TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);  //��ʾ��Ե��ȡͼ��
//        TFTSPI_BinRoadSide(ImageSide);                                  //���ұ���
//        TFTSPI_BinRoad_UpdownSide(UpdowmSide);                          //���±���
//        TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 159, ROAD_MAIN_ROW, u16RED); //��������ʾ
//        TFTSPI_Draw_Line(80, 0, 80, 119, u16RED);                       //��ʾ����
//    }
//    else
//    {
//        TFTSPI_CLS(u16BLACK);                   // ����
//

    sprintf(txt, "%05d", g_sSteeringError);         //���ֵ
    TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);

//    sprintf(txt, "%02d", g_ucFlagRoundabout);       //������־
//    TFTSPI_P6X8Str(8, 15, txt, u16RED, u16BLUE);
//
//    sprintf(txt, "%02d", g_ucFlagT);                //T�ڱ�־
//    TFTSPI_P6X8Str(12, 15, txt, u16RED, u16BLUE);
//
//    sprintf(txt, "%02d", g_ucFlagFork);             //Y�ڱ�־
//    TFTSPI_P6X8Str(16, 15, txt, u16RED, u16BLUE);

//    sprintf(txt, "%02d", g_ucFlagZebra);            //������
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

//������ȳ�ʼ��
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
//    unsigned char i,j = 0;//ѭ���ñ���
//    //�����ұ�������ת��,ת��use������޸�
//    for(i=120-1;i>20;i-=4)
//    {
//        rightline_use[i] = rightline[i];
//        leftline_use[i] = leftline[i];
//        midline_use[i] = midline[i];
//    }
//
//    /************************************����λ�����ݼ̳�****************************************************/
//               //�ɶ���λ���ٽ��ж�ʹ����ʷ�̳�
//    //daixiugai
//               if(right_break>=116)//��ʼʹ����ʷ��������
//               {
//                   //�ұ��ߵ���
//                   for(i=120-1;i>110;i-=4)
//                   {
//                       rightline[i] = rightline_use[i-4];
//                   }
//               }
//               if(left_break > 110)//��ʼʹ����ʷ��������
//               {
//                   //zuo���ߵ���
//                   for(i=120-1;i>110;i-=4)
//                   {
//                       leftline_use[i] = leftline_use[i-4];
//                   }
//               }
//    /************************************���ϱ���λ�����ݼ̳�****************************************************/
//
//    if(right_break != 120 || left_break !=120)
//    {
//        if(right_break < 120)
//        {
//            for(i=120-1;i>right_break;i-=4)
//            {
//                rightline_use[i] = rightline[i-4];
//            }
//            //�ұ���ԭʼͼ����
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
//            //�����������ԭʼͼ����
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
//    //Բ������,use����������
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
//               if(tangent_expoint(str_curhang,str_curlie,in_cur_flag))//�����ҳ���Բ�����
//               {
//                   //����÷�֧˵������㲹���Ѿ����
//                   //������use������������
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
//   /************************************����λ�����ݼ̳�****************************************************/
//       //���油�ߣ����ڷ������·��������
//       if(right_break != 120)
//       {
//           //�ұ��߲���
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
//           //����߲���
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
//    /*****************************����Բ������********************************/
//
//
//
//void Feature_Way(void)
//{
///****************************************************************�����Ǹ��ֵ�·�����ж�*************************************************/
//
//    unsigned char i,j = 0;//ѭ���ñ���
//    //���߼�飬���ñ��ߵ�����ʵ�ֿ�ȱ��·�������ж�
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
//    //���ұ���ת�ۼ��
//    Line_ck(1);
//    Line_ck(-1);
//
//    //    //������������ж�
//       //    if(!left_break && right_break && !in_tri_flag && !upfill_flag)//�жϽ���Բ��
//       //    {
//       //        in_cur_flag = 1;
//       //    }
//       //    else if(left_break && right_break && !in_cur_flag)//�жϽ���������µ�
//       //    {
//       //        for(i=120;i>20;i++)
//       //        {
//       //            if(Bin_Image[i][midline[i]] == 1)
//       //            {
//       //                mid_break = i;
//       //                break;
//       //            }
//       //        }
//       //        if(mid_break)//�ж�Ϊ����
//       //        {
//       //            in_tri_flag = 1;
//       //        }
//       //        else         //�ж�Ϊ�µ�
//       //            upfill_flag = 1;
//       //
//
//
//
//
//    //���Բ��������set����line����Բ�����ߣ�ʶ��Բ���ص�������Բ���ļ�⣬����ʶ���Բ������֮���¼��Բ��������
//        //����ԭʼ������ͼ������
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
//                            line_down = line_ex[1];//��Ӧhang��Ϣ right_break+counter
//                            str_curlie = line_down;
//                            str_curhang = right_break+counter;
//                            line_down = 0;
//                            in_cur_flag = 1;//youԲflag
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
//                            line_down = line_ex[1];//��Ӧhang��Ϣ right_break+counter
//                            str_curlie = line_down;
//                            str_curhang = left_break+counter;
//                            line_down = 0;
//                            in_cur_flag = 2;//��Բflag
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
// * Բ������ʱ�Ķ������λ����Ϣ
// *
// * ԭ�ȸú�������void���ǿ��ǵ��õ����С��λ����Ϣ����ʾ����
// * ����Ϊunsigned char һ���ж�С���Ƿ��Ѿ��ս�Բ��
// * ����ֵλ���Ƿ���ʣ��д������������
// *
// * ����ͷλ��Ҫ���ж�Բ��֮ǰ����������Ļ��Χ
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
//                                //�ҵ������󣬸�����Ұ�����ɲ���,
//                                left_ck = (float)(tangent_exlie - leftline[str_curhang])/(tangent_exhang - str_curhang);
//                                for(m=str_curhang;m>20;m--)
//                                {
//                                    for(n=leftline[str_curhang];n<150;n++)
//                                    {
//                                        //�����������ԭʼͼ����
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
//    //��Բ����Ӧ��飬δ�޸�
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
    //���߳�ʼ��

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
 * ��������������,�ⲿ���պ��Ż����ã�����ɾ�ˣ�
 * Ŀǰ˼·������ʼ�л�ȡ��ʼ�����ߵ�λ����Ϣ��
 * Ȼ������������߲��ֵĵ�����Ϣ��ȡ���������췽���Լ�������Ϣ��
 * ����Ӧ�ÿ��Ա������ұ���λ�õ��ظ�ʹ�ã����������Բ����ͼ�����쵼�µ�ƫ�����⣩
 * ��ʱ�ö�ά���飬��ά�����д��Ժ�ƫ������ٿ���
 */
//
//void Shin(void)
//{
//    //����Ѱ����ʼ�㣬���ڿ���С����λ�ã����ߴ�����ڳ����ϣ��������ٽ������������������ʼ��
//    unsigned char i,j;
//    if(right_break != 119 || left_break != 119)//����Ѿ��������߼̳У�����ԭʼ��ʼ���߻�ȡ
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
//            for(i=119;i>115;i--)//�ҵ���ʼ���ֵ�����λ��
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
//            for(i=120-1;i>=115;i--)//�ҵ���ʼ���ֵ�����λ��
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
//    for(i=120-1;i>=115;i--)//ȡ����ʼ������Ϣ
//    {
//        midline[i] = (leftline[i] + rightline[i])/2;
//    }
//    //����ʼ���ֿ�ʼ����,
//    //���������ʻ����ȷһ�㣨��δ��֤��
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
////        shin_k = (float)(midline[i-4] - midline[i])/(-4);//���ӣ���  ��ĸ���У�һ��ע����һ�㣩
////        shin_ck = (-1)/shin_k;
////        midline[i-8] = midline[i-4] + (unsigned char)shin_k * (-4);
////        if(shin_k < 0 && shin_k > -20)//������ƫ
////        {
////            //���д����·�������
////            for(m=i-8+1;m>119;m++)//�ұ���hang��ʼ
////            {
////                for(n=midline[i-8];n<160;n++)//��ѭ��
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
////                        for(p=i-8-1;p>20;p--)//�����hang��ʼ
////                        {
////                            for(q=midline[i-8];q>0;q--)//��ѭ��
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
////                                        midline[(righthang_last[0]+lefthang_last[0])/2] = (rightline[righthang_last[0]] + leftline[lefthang_last[0]])/2;//��Ӧ������
////                                        i=(righthang_last[0]+lefthang_last[0])/2;
////                                        //����һ�������ߵ����߻�ȡ�м�ȱ��λ����Ϣ����Ȼ���ǵ��Ѿ���ȡ��֪�������ߵ��λ�ã���ʵ����ֱ��ȡ���ȡƫ��
////                                        //����������ʱ��д���п��ٸ�
////                                    }
////                                }
////                            }
////                        }
////                    }
////                }
////            }
////        }
////        else if(shin_k > 0 && shin_k < 20)//������ƫ
////        {
////            for(m=i-8-1;m>20;m--)//hang��ʼ
////            {
////                for(n=midline[i-8];n<160;n++)//��ѭ��
////                {
////                    search_k = (float)(n - midline[i-8])/(m - (i-8));
////                    //�����
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
////                        for(p=i-8+1;p>20;p--)//hang��ʼ
////                        {
////                            for(q=midline[i-8];q>0;q--)//��ѭ��
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
////                                        midline[(righthang_last[0]+lefthang_last[0])/2] = (rightline[righthang_last[0]] + leftline[lefthang_last[0]])/2;//��Ӧ������
////                                        i=(righthang_last[0]+lefthang_last[0])/2;
////                                        //����һ�������ߵ����߻�ȡ�м�ȱ��λ����Ϣ����Ȼ���ǵ��Ѿ���ȡ��֪�������ߵ��λ�ã���ʵ����ֱ��ȡ���ȡƫ��
////                                        //����������ʱ��д���п��ٸ�
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
//�򵥵ı��߻ع���
//char line_back(char x,unsigned char hang,unsigned char lie)
//{
//    if(x>0)//����
//    {
//        if(Bin_Image[hang][lie] == 1) {right_ret = lie; return 1;}
//        else if(Bin_Image[hang][lie+1] - Bin_Image[hang][lie] == 1) {right_ret = lie+1; return 1;}
//        else if(Bin_Image[hang][lie+2] - Bin_Image[hang][lie+1] == 1) {right_ret = lie+2; return 1;}
////        else if(Bin_Image[hang][lie+3] - Bin_Image[hang][lie+2] == 1) {right_ret = lie+3; return 1;}
//    }
//    else if(x<0)//����
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
////����б�����¶��ϼ��
//unsigned char Line_ck(char f)
//{
//    unsigned char i;
//    double r_k[2];
//    double l_k[2];
//    if(f < 0)//zuo���߼��
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
//    else if(f > 2)//�ұ��߼��
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
 *  �������ƣ�void CameraCar(void)
 *  ����˵������ų�˫������ٿ���
 -->1.�����㷨���򵥵ķֶα��������㷨����ѧ��ʾ�����㷨��
 2.�����㷨��PID����Ӧ�ÿ����㷨����ѧ��ʾ�����㷨��
 3.�߶��㷨���Ľ�����ȺЭͬ�����㷨��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע������2�����
 *************************************************************************/
static uint8_t g_ucFlagRoundabout_flag = 0;

//����Ԫ�ش������������޸ĵò�����С�����ٶȣ����Ŵ�����йأ���Ҫͬѧ���Լ�����ʵ��������޸�
void CameraCar (void)
{
    LED_Ctrl(LED1, RVS);     // LED��˸ ָʾ��������״̬
    uint8_t pointY;

     if(g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0)
     {
         // ��⻷��
         RoadIsRoundabout(UpdowmSide, Bin_Image, ImageSide, &g_ucFlagRoundabout);
     }
    if(g_ucFlagRoundabout)
     {
        g_ucFlagRoundabout_flag = 1;
       //   ��������

        Servo_P = 12;           //���Ŵ�
        RoundaboutProcess(Bin_Image, ImageSide, UpdowmSide, &g_ucFlagRoundabout);

     }

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
//      //ʮ�ֲ���δ�õ�
//     if(g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagFork == 0)
//     {
//         /* ���ʮ�� */
//         RoadIsCross(ImageSide, &g_ucFlagCross);
//     }
//     if(g_ucFlagCross)
//     {
//         /* ʮ�ִ��� */
//         CrossProcess(Image_Use, ImageSide, &g_ucFlagCross);
//     }

    /********************************************************************************************/
    /********************************T��·��**********************************************/

    if(g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0)
    {
        //���T��
        RoadIsT(UpdowmSide, ImageSide, &g_ucFlagT);
    }
    if(g_ucFlagT)
    {
        Servo_P = 12;
        //T�ִ���
        TProcess(Bin_Image, UpdowmSide, ImageSide, &g_ucFlagT);
    }

     /************************************************************************
       2021/7/19���Դ���  Y��·��
       ************************************************************************/
     if(g_ucFlagRoundabout == 0 && g_ucFlagFork == 0 && g_ucFlagT == 0)
       {
           RoadIsFork(UpdowmSide, ImageSide, &g_ucFlagFork, &pointY);
       }
     if(g_ucFlagFork == 1)
     {
         g_ucForkNum += 1;
     }

       if(g_ucFlagFork)//�������
       {

           Servo_P = 10;
           // Y�ִ���
           ForkProcess(UpdowmSide, ImageSide, &g_ucFlagFork);
       }
       ////////////////////////����ʶ�����///////////////////////////
       ////////////////////////�ⲿ��δ�޸�///////////////////////////
     //  if(g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0&& g_ucFlagFork == 0)
     //  {
           /* ��⳵�� */
     //      RoadIsCross(ImageSide, &g_ucFlagZebra);
     //  }
     //  if(g_ucFlagZebra)
     //  {
     //      ZebraProcess(Image_Use,1,1200);
     //  }


     /* ���������У���ȡ���ƫ�� */
     g_sSteeringError = RoadGetSteeringError(ImageSide, ROAD_MAIN_ROW);
     //ƫ��Ŵ�
     ServoDuty = g_sSteeringError * Servo_P/10;
     //ƫ���޷�
     if(ServoDuty>170) ServoDuty=170;
     if(ServoDuty<-170) ServoDuty=-170;
     //������
     //ServoCtrl(Servo_Center_Mid-ServoDuty);
}
