/***********************************************************************
 * @file BLE_client.h
 * BLE_CLIENT
 * @author	:	ch
 * @brief	:	
 * @version:	v1.0
 * @Copyright (C)  2020-12-10  .cdWFVCEL. all right reserved
***********************************************************************/

#ifndef __BLE_CLIENT_H_
#define __BLE_CLIENT_H_
/* 包含头文件 ------------------------------------------------------------------*/
#include "ch/bsp.h"
#include "BLEDevice.h"
/* 宏定义 ----------------------------------------------------------------------*/
/**
 * 低功耗蓝牙分四级
 * -60 ~ 0      4
 * -70 ~ -60    3
 * -80 ~ -70    2
 * <-80         1
 */

#define JSON_FORMAT "{'temp':%d,'IR':%d,'BPM':%d,'ABPM':%d,'finger':%d}"
/* 类型定义 --------------------------------------------------------------------*/
/* 变量声明 --------------------------------------------------------------------*/
extern BLEAdvertisedDevice *myDevice;
extern BLEClient *pClient;
extern boolean connected;
/* 函数声明 --------------------------------------------------------------------*/
void BLEC_Init();
void BLEC_Prog();
#endif // __BLE_CLIENT_H_
