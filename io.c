#include "io.h"
#include "1553b.h"
#include "controller.h"

#define RT_GYRO 1
#define RT_STAR 2
#define RT_MW   3

#define SA_GYRO_PLS RT_SA10
#define SA_STAR_Q   RT_SA10
#define SA_MW_CMD   RT_SA11

#define LEN16_GYRO_PLS (1 + 12 + 1)
#define LEN16_STAR_Q   (1 + 8 + 1)
#define LEN16_MW_CMD   (1 + 6 + 1)

unsigned short buf16Rt[32];
unsigned short buf16Gyro[32];
unsigned short buf16StarQ[32];

unsigned int cntInvalidTc = 0;

int global_t;
int global_f;
unsigned int uint_global_result;
unsigned int uint_global_divide;
void printIntLine(int n);
void printLine(char *);

unsigned short CheckSum(int data[], unsigned int len)
{
	unsigned int i;
	unsigned short chSum = 0;
	for (i = 0;i < len; i++)
	{
		chSum = chSum + data[i];
	}
	

	return chSum;
}

int knife_ds_main()
{
    int data;
	unsigned int temp_count=0;
	unsigned int temp_result;
    /* Initialize data */
    data = -1;
    if(global_f)
    {

        data = 10;
		temp_count ++;
    }
    else
    {
        data = 0;
    }
    if(global_t)
    {
        {
            int i;
            int buffer[10] = { 0 };
            if (data >= 1)
            {
                buffer[data] = 1;
                for(i = 0; i < 10; i++)
                {
                    printIntLine(buffer[i]);
                }
            }
            else
            {
            	if(temp_count != 0)
					global_f = (global_f+10) / temp_count;
                printLine("ERROR: Array index is negative.");
            }
        }
    }
    else
    {
        {
            int i;
            int buffer[10] = { 0 };
            if (data >= 0 && data < (10))
            {
                buffer[data] = 1;
                /* Print the array values */
                for(i = 0; i < 10; i++)
                {
                    printIntLine(buffer[i]);
                }
            }
            else
            {
                printLine("ERROR: Array index is out-of-bounds");
            }
        }
		uint_global_result = uint_global_divide - temp_count;
    }
	return data;
}

void InitIO(void)
{
	B1553InitBC(BUSG_MEM_SIZE, BUSG_REG_BASE, BUSG_MEM_BASE);

	return;
}

void PakMwCmd(unsigned short buf16[])
{
	volatile unsigned int convert;
	volatile float *pConvert = (volatile float*)(&convert);
	
	buf16[0] = 0x5A01;

	*pConvert = ctrlOutM[0];
	buf16[1] = (convert >> 16) & 0xFFFF;
	buf16[2] = convert & 0xFFFF;
	
	*pConvert = ctrlOutM[1];
	buf16[3] = (convert >> 16) & 0xFFFF;
	buf16[4] = convert & 0xFFFF;
	
	*pConvert = ctrlOutM[2];
	buf16[5] = (convert >> 16) & 0xFFFF;
	buf16[6] = convert & 0xFFFF;

	buf16[LEN16_MW_CMD - 1] = CheckSum(buf16, LEN16_MW_CMD - 1);

	return;
}

void ParseGyroData(void)
{
	unsigned int msgCmd;
	unsigned int msgLen;
	unsigned int msgRt;
	unsigned int msgSa;
	unsigned int msgDir;
	unsigned short chSum;
	volatile unsigned int convert;
	volatile float *pConvert = (volatile float*)(&convert);

	msgCmd = Read1553Msg(buf16Gyro, 0);

	/* �����Ϣ������ */
	msgRt = B1553_CMDWORD_ADDR(msgCmd);
	msgDir = B1553_CMDWORD_TRCV(msgCmd);
	msgSa = B1553_CMDWORD_SAMC(msgCmd);
	msgLen = B1553_CMDWORD_CNTD(msgCmd);

	if ((msgRt == RT_GYRO) && (msgDir == B1553_RT_TRAN) && (msgSa == SA_GYRO_PLS) && (msgLen == LEN16_GYRO_PLS))
	{
		/* ���У��� */
		chSum = CheckSum(buf16Gyro, LEN16_GYRO_PLS - 1);
		if (chSum == buf16Gyro[LEN16_GYRO_PLS - 1])
		{
			/* ���� */
			convert = U16_MAKE_U32(buf16Gyro[1], buf16Gyro[2]);
			ctrlInSensorGyro[0] = *pConvert;
			
			convert = U16_MAKE_U32(buf16Gyro[3], buf16Gyro[4]);
			ctrlInSensorGyro[1] = *pConvert;
			
			convert = U16_MAKE_U32(buf16Gyro[5], buf16Gyro[6]);
			ctrlInSensorGyro[2] = *pConvert;
			
			convert = U16_MAKE_U32(buf16Gyro[7], buf16Gyro[8]);
			ctrlInSensorGyro[3] = *pConvert;
			
			convert = U16_MAKE_U32(buf16Gyro[9], buf16Gyro[10]);
			ctrlInSensorGyro[4] = *pConvert;
			
			convert = U16_MAKE_U32(buf16Gyro[11], buf16Gyro[12]);
			ctrlInSensorGyro[5] = *pConvert;
		}
	}

	return;
}

void ParseStarData(void)
{
	unsigned int msgCmd;
	unsigned int msgLen;
	unsigned int msgRt;
	unsigned int msgSa;
	unsigned int msgDir;
	unsigned short chSum;
	volatile unsigned int convert;
	volatile float *pConvert = (volatile float*)(&convert);

	msgCmd = Read1553Msg(buf16StarQ, 1);

	/* �����Ϣ������ */
	msgRt = B1553_CMDWORD_ADDR(msgCmd);
	msgDir = B1553_CMDWORD_TRCV(msgCmd);
	msgSa = B1553_CMDWORD_SAMC(msgCmd);
	msgLen = B1553_CMDWORD_CNTD(msgCmd);

	if ((msgRt == RT_STAR) && (msgDir == B1553_RT_TRAN) && (msgSa == SA_STAR_Q) && (msgLen == LEN16_STAR_Q))
	{
		/* ���У��� */
		chSum = CheckSum(buf16StarQ, LEN16_STAR_Q - 1);
		if (chSum == buf16StarQ[LEN16_STAR_Q - 1])
		{
			/* ���� */
			convert = U16_MAKE_U32(buf16StarQ[1], buf16StarQ[2]);
			ctrlInSensorQ[0] = *pConvert;
			
			convert = U16_MAKE_U32(buf16StarQ[3], buf16StarQ[4]);
			ctrlInSensorQ[1] = *pConvert;
			
			convert = U16_MAKE_U32(buf16StarQ[5], buf16StarQ[6]);
			ctrlInSensorQ[2] = *pConvert;
			
			convert = U16_MAKE_U32(buf16StarQ[7], buf16StarQ[8]);
			ctrlInSensorQ[3] = *pConvert;
		}
	}

	return;
}

void QuerySensorData(void)
{
	unsigned int cntMsg;
	unsigned int frameEnd;
	unsigned short cmd;

	cntMsg = 0;

	/* ����ȡ�� */
	cmd = (unsigned short)B1553MsgCmdGen(RT_GYRO, B1553_RT_TRAN, SA_GYRO_PLS, LEN16_GYRO_PLS);
	cntMsg = B1553MsgRegister(cntMsg, B1553_CHNL_A, cmd, buf16Rt);

	/* ����ȡ�� */
	cmd = (unsigned short)B1553MsgCmdGen(RT_STAR, B1553_RT_TRAN, SA_STAR_Q, LEN16_STAR_Q);
	cntMsg = B1553MsgRegister(cntMsg, B1553_CHNL_A, cmd, buf16Rt);

	/* ����ȡ��ָ�� */
	if (cntMsg == 2)
	{
		B1553MsgFrameBuild(cntMsg, BUSG_MEM_BASE);
		B1553MsgFrameStart(cntMsg, BUSG_REG_BASE, BUSG_MEM_BASE);

		/* �ȴ���Ϣ������� */
		frameEnd = B1553MsgFrameWait(cntMsg, BUSG_REG_BASE, BUSG_FRAME_WAIT_TIME);

		if (frameEnd == 0u)
		{
			/* �������ݽ��� */
			ParseGyroData();

			/* �������ݽ��� */
			ParseStarData();
		}
	}

	return;
}

void SendCmdData(void)
{
	unsigned int cntMsg;
	unsigned int frameEnd;
	unsigned short cmd;

	cntMsg = 0;

	/* ������ָ����� */
	PakMwCmd(buf16Rt);

	/* �����ַ��� */
	cmd = (unsigned short)B1553MsgCmdGen(RT_MW, B1553_RT_RECV, SA_MW_CMD, LEN16_MW_CMD);
	cntMsg = B1553MsgRegister(cntMsg, B1553_CHNL_A, cmd, buf16Rt);

	/* ����ȡ��ָ�� */
	if (cntMsg == 1)
	{
		B1553MsgFrameBuild(cntMsg, BUSG_MEM_BASE);
		B1553MsgFrameStart(cntMsg, BUSG_REG_BASE, BUSG_MEM_BASE);

		/* �ȴ���Ϣ������� */
		frameEnd = B1553MsgFrameWait(cntMsg, BUSG_REG_BASE, BUSG_FRAME_WAIT_TIME);
	}

	return;
}

/*
[0]: �̶�Ϊ 0xeb9014f6
[1]: ָ����
	0x01010101: ģʽת��ָ��, [2]ȡֵΪ1����ʾת���������ƣ�ȡֵΪ2����ʾת����Զ���
	0x02020202: ��ͣ����ָ��, [2]ȡֵΪ0/��0����ʾ������/����
	0x03030303: ����ʹ��ָ��, [2]ȡֵΪ0-5����ʾ6�����ݵı�ţ�[3]ȡֵΪ0/��0����ʾ��ʹ��/ʹ��
	����Ƿ�
*/
void ParseTcData(volatile unsigned int buf32[])
{
	if (buf32[0] == 0xeb90146f)
	{
		if (buf32[1] == 0x01010101)
		{
			/* ģʽת��ָ�� */
			if (buf32[2] == 1)
			{
				flgCtrl = 1;
			} 
			else if (buf32[2] == 2)
			{
				flgCtrl = 2;
			}
			else
			{
				flgCtrl = 0;
			}
		} 
		else if (buf32[1] == 0x02020202)
		{
			/* ��ͣ����ָ�� */
			if (buf32[2] == 0)
			{
				gParaPauseEnable = 0;
			} 
			else
			{
				gParaPauseEnable = 1;
			}
		} 
		else if (buf32[1] == 0x03030303)
		{
			/* ����ʹ��ָ�� */
			if (buf32[2] < 6)
			{
				if (buf32[3] == 0)
				{
					gParaGyroUsed[buf32[2]] = 0;
				} 
				else
				{
					gParaGyroUsed[buf32[2]] = 1;
				}
			}
			else
			{
				cntInvalidTc++;
			}
		} 
		else
		{
			/* �Ƿ�ָ�� */
			cntInvalidTc++;
		}
	}
	else
	{
		cntInvalidTc++;
	}

	return;
}

void PakTmData(volatile unsigned int buf32[])
{
	int i;
	volatile float convert;
	volatile unsigned int *pConvert;

	pConvert = (unsigned int *)(&convert);
	/* ģʽ��
	1��ͣ��ģʽ
	2������ģʽ
	3����ȫģʽ
	4�����Զ���ģʽ
	5���ٴν��밲ȫģʽ
	6����ͣģʽ	*/
	buf32[0] = ctrlMode;

	/* �Ƕ� */
	convert = ctrlAngle[0] * 57.295779513082321;
	buf32[1] = *pConvert;

	convert = ctrlAngle[1] * 57.295779513082321;
	buf32[2] = *pConvert;

	convert = ctrlAngle[2] * 57.295779513082321;
	buf32[3] = *pConvert;

	/* ���ٶ� */
	convert = ctrlOmg[0] * 57.295779513082321;
	buf32[4] = *pConvert;

	convert = ctrlOmg[1] * 57.295779513082321;
	buf32[5] = *pConvert;

	convert = ctrlOmg[2] * 57.295779513082321;
	buf32[6] = *pConvert;

	/* �������� */
	convert = ctrlOutM[0];
	buf32[7] = *pConvert;

	convert = ctrlOutM[1];
	buf32[8] = *pConvert;

	convert = ctrlOutM[2];
	buf32[9] = *pConvert;

	/* ��ͣģʽ�����־ - �Ƿ�ң��ָ����� */
	buf32[10] = U16_MAKE_U32(gParaPauseEnable, cntInvalidTc);
	
	/* ����ʹ�ñ�־ */
	buf32[11] = 0;
	for (i = 0; i < 6; i++)
	{
		if (gParaGyroUsed[i] > 0)
		{
			buf32[11] = buf32[11] | (1 << i);
		}
	}

	return;
}
