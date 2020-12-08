/***********************************************************************
 * @file ntc.h
 * NTC
 * @author	:	ch
 * @brief	:	
 * @version:	v1.0
 * @Copyright (C)  2020-12-07  .cdWFVCEL. all right reserved
***********************************************************************/

#ifndef __NTC__H_
#define __NTC__H_
/* ����ͷ�ļ� ------------------------------------------------------------------*/
#include "ch/bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* �궨�� ----------------------------------------------------------------------*/
#define NTC_TEMP_C 181
/* ���Ͷ��� --------------------------------------------------------------------*/
/* �������� --------------------------------------------------------------------*/
extern double gNtc100K_Mapping[][2]; //100K ntc �¶���ֵӳ���
/* �������� --------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif // __NTC_H_