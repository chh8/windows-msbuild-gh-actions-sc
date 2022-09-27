#ifndef __IO_H__
#define __IO_H__

#define ADDR_CPU_MEC ((volatile unsigned int*)0x80000000)
#define ADDR_TM_CTRL ((volatile unsigned int*)0x22000000)
#define ADDR_TM_DATA ((volatile unsigned int*)0x22000004)
#define ADDR_TC_CTRL ((volatile unsigned int*)0x22100000)
#define ADDR_TC_DATA ((volatile unsigned int*)0x22100004)

/* 按32位无符号整型数取单精度浮点数的内存值 */
#define FLT32_TO_MEM(fptr)          (*((volatile unsigned int *)(fptr)))

/* 按32位无符号整型数取单精度浮点数的内存值 */
#define MEM_TO_FLT32(uptr)          (*((volatile float *)(uptr)))

/* 按64位无符号整型数取双精度浮点数的内存值 */
#define MEM_TO_FLT64(ulptr)         (*((volatile double *)(ulptr)))

/* 将32位无符号整型数强制转换为32位无符号整型数地址 */
#define ABSUI_TO_VADDR32(ui)        ((volatile unsigned int *)(ui))

/* 将32位无符号整型数强制转换为8位无符号整型数地址 */
#define ABSUI_TO_ADDR08(ui)         ((volatile unsigned char *)(ui))

/* 将32位无符号整型数强制转换为地址 */
#define ABSUI_TO_VADDR(ui)          ((volatile unsigned int *)(ui))
#define ABSUI_TO_ADDR(ui)           ((unsigned int *)(ui))

/* 将地址值转换为32位无符号整型数 */
#define ADDR_TO_ABSUI(ptr)          ((unsigned int)(ptr))

/* 往硬件地址写32位数据宏函数 */
#define ADDR_WRITE(addr, value)     {(*((volatile unsigned int *)(addr))) = (value);}

/* 往硬件地址写32位数据宏函数 */
#define ADDR_WRITE16(addr, val16)   {(*((volatile unsigned short *)(addr))) = (val16);}

/* 从硬件地址读32位数据宏函数 */
#define ADDR_READ(addr)             (*((volatile unsigned int *)(addr)))

/* 从硬件地址读16位数据宏函数 */
#define ADDR_READ16(addr)           (ADDR_READ((addr)) & 0xFFFF)

/* 从硬件地址读8位数据宏函数 */
#define ADDR_READ08(addr)           (ADDR_READ((addr)) & 0xFF)

#define U16_MAKE_U32(u16hi, u16lo)  (((u16lo) & 0xFFFF) | (((unsigned int)(u16hi)) << 16))

void InitIO(void);
void QuerySensorData(void);
void SendCmdData(void);
void PakTmData(volatile unsigned int buf32[]);
void ParseTcData(volatile unsigned int buf32[]);

#endif
