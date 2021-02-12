/***********************************************************************
 * @file 	:	temperature.cpp
 * @author	:	ch
 * @brief	:	
 * @version:	v1.0
 * @Copyright (C)  2020-12-10  .cdWFVCEL. all right reserved
***********************************************************************/

/* 包含头文件 -------------------------------------------------------------------*/
#include <Arduino.h>
#include "modules/ntc.h"
#include "temperature.h"
/* 宏定义 -----------------------------------------------------------------------*/
/* 类型定义 ---------------------------------------------------------------------*/
/* 私有变量 ---------------------------------------------------------------------*/
/* 扩展变量 ---------------------------------------------------------------------*/
double temperature = 0;
/* 私有函数声明 -----------------------------------------------------------------*/
/* 函数声明 ---------------------------------------------------------------------*/
/* 函数实现 ---------------------------------------------------------------------*/

void Temp_Init()
{
    pinMode(TEMPERATURE_PIN,INPUT);
}
double Temp_Scan()
{
    int sensorValue = analogRead(TEMPERATURE_PIN);
    double voltage = sensorValue * (3.3 / 4096.0);

#define FREG 100000
    double rt = ((3.3 * FREG) - (voltage * FREG)) / voltage;
    // Serial.printf("%d\r\n", (int)rt);
    double retTemperature = 0;

    int ntcNum = NTC_TEMP_C;
    for (int i = 0; i < ntcNum; i++)
    {
        if (i == ntcNum - 1)
        {
            retTemperature = gNtc100K_Mapping[i][0];
            return retTemperature;
        }
        if ((rt <= gNtc100K_Mapping[i][1]) && (rt > gNtc100K_Mapping[i + 1][1]))
        {
            retTemperature = gNtc100K_Mapping[i][0];
            return retTemperature;
        }
    }
    temperature = retTemperature;
    return retTemperature;
}
//temperature.cpp