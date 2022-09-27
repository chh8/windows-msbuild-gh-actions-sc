#ifndef __IO_H__
#define __IO_H__

#define ADDR_CPU_MEC ((volatile unsigned int*)0x80000000)
#define ADDR_TM_CTRL ((volatile unsigned int*)0x22000000)
#define ADDR_TM_DATA ((volatile unsigned int*)0x22000004)
#define ADDR_TC_CTRL ((volatile unsigned int*)0x22100000)
#define ADDR_TC_DATA ((volatile unsigned int*)0x22100004)

/* ��32λ�޷���������ȡ�����ȸ��������ڴ�ֵ */
#define FLT32_TO_MEM(fptr)          (*((volatile unsigned int *)(fptr)))

/* ��32λ�޷���������ȡ�����ȸ��������ڴ�ֵ */
#define MEM_TO_FLT32(uptr)          (*((volatile float *)(uptr)))

/* ��64λ�޷���������ȡ˫���ȸ��������ڴ�ֵ */
#define MEM_TO_FLT64(ulptr)         (*((volatile double *)(ulptr)))

/* ��32λ�޷���������ǿ��ת��Ϊ32λ�޷�����������ַ */
#define ABSUI_TO_VADDR32(ui)        ((volatile unsigned int *)(ui))

/* ��32λ�޷���������ǿ��ת��Ϊ8λ�޷�����������ַ */
#define ABSUI_TO_ADDR08(ui)         ((volatile unsigned char *)(ui))

/* ��32λ�޷���������ǿ��ת��Ϊ��ַ */
#define ABSUI_TO_VADDR(ui)          ((volatile unsigned int *)(ui))
#define ABSUI_TO_ADDR(ui)           ((unsigned int *)(ui))

/* ����ֵַת��Ϊ32λ�޷��������� */
#define ADDR_TO_ABSUI(ptr)          ((unsigned int)(ptr))

/* ��Ӳ����ַд32λ���ݺ꺯�� */
#define ADDR_WRITE(addr, value)     {(*((volatile unsigned int *)(addr))) = (value);}

/* ��Ӳ����ַд32λ���ݺ꺯�� */
#define ADDR_WRITE16(addr, val16)   {(*((volatile unsigned short *)(addr))) = (val16);}

/* ��Ӳ����ַ��32λ���ݺ꺯�� */
#define ADDR_READ(addr)             (*((volatile unsigned int *)(addr)))

/* ��Ӳ����ַ��16λ���ݺ꺯�� */
#define ADDR_READ16(addr)           (ADDR_READ((addr)) & 0xFFFF)

/* ��Ӳ����ַ��8λ���ݺ꺯�� */
#define ADDR_READ08(addr)           (ADDR_READ((addr)) & 0xFF)

#define U16_MAKE_U32(u16hi, u16lo)  (((u16lo) & 0xFFFF) | (((unsigned int)(u16hi)) << 16))

void InitIO(void);
void QuerySensorData(void);
void SendCmdData(void);
void PakTmData(volatile unsigned int buf32[]);
void ParseTcData(volatile unsigned int buf32[]);

#endif
