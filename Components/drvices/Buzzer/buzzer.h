#ifndef BUZZER_H
#define BUZZER_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_init.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
		DDO=1976,
			bDRE=1868,
		DRE=1760,
			bDMI=1664,
		DMI=1568,
			bDFA=1482,
		DFA=1397,
            bDSO=1357,
		DSO=1318,
            bDLA=1246,
		DLA=1175,
            bDXI=1110,
		DXI=1046,
            
		DO=988,
            bRE=934,
		RE=880,
            bMI=832,
		MI=784,
            bFA=741,
		FA=698,
            bSO=678,
		SO=659,
            bLA=623,
		LA=587,
            bXI=555,
		XI=523,
	
		HDO=494,
            bHRE=467,
		HRE=440,
            bHMI=416,
		HMI=392,
            bHFA=370,
		HFA=349,
            bHSO=339,
		HSO=330,
            bHLA=312,
		HLA=294,
            bHXI=278,
		HXI=262,
}M;

/* 宏定义 --------------------------------------------------------------------*/
#define BEEP_PERIOD   2500
#define BEEP_ON_TIME  100
#define BEEP_OFF_TIME 100

#define BEEP_TUNE_VALUE 500
#define BEEP_CTRL_VALUE 150

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Buzzer_SetBeep(uint16_t tune, uint16_t value);
void BeepHandler(void);
void BeepTimesSet(uint8_t times);
void BeepTimeSet_ON_OFF(uint16_t on_time, uint16_t off_time);

#endif  // BUZZER_H

