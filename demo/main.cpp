
/***********************************************************************
 * @file 	:	main.cpp
 * @author	:	( $ _ $ ) 自由的假发 ( $ _ $ )
 * @brief	:	
 * @version:	v1.0
 * @Copyright (C)  2020-11-21  .cdWFVCEL. all right reserved
***********************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "modules/ntc.h"
#include "temperature.h"
#include "heartbeat.h"
#include "BLE_client.h"

/* #includes -------------------------------------------------------------------------- */

/* define  -------------------------------------------------------------------------- */

#define IO_1V8 18
#define LED 19
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 6        /* Time ESP32 will go to sleep (in seconds) */

#define COLLECT_NUM 500 //数据采集次数

/*  ---------------------------------------------------------------------------------------------------------------*/
/* variable  -------------------------------------------------------------------------- */

// double temperature = 0;

RTC_DATA_ATTR int bootCount = 0;

/* func declare -------------------------------------------------------------------------- */
void print_wakeup_reason();

/* func define  -------------------------------------------------------------------------- */

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

/*  -------------------------------------------------------------------------- */

/*  -------------------------------------------------------------------------- */

void setup()
{
  pinMode(IO_1V8, OUTPUT);
  digitalWrite(IO_1V8, HIGH);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // Temp_Init();

  Serial.begin(115200);
  Serial.println("running...");

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  // digitalWrite(IO_1V8, HIGH);
  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  HB_Init();

  delay(2000);
  BLEC_Init();
  delay(2000);
}

uint32_t ble_c = 0; //control BLE transimit frequency
uint32_t adc_c = 0;

void loop()
{
  // for (int i = 0; i < COLLECT_NUM; i++)
  // {
  //   HB_Scan();
  //   // temperature = Temp_Scan();

  //   char str[64] = {0};
  //   sprintf(str, "T:%d IR:%d BMP:%d ABMP:%d F:%d", (int)temperature, (int)irValue, (int)beatsPerMinute, beatAvg, finger_f);
  //   Serial.println(str);
  // }

  // if (finger_f)
  // {
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
  }
  // }

  BLEC_Prog();

  Serial.flush();
  digitalWrite(IO_1V8, LOW);
  Serial.println("sleep...");
  esp_deep_sleep_start();
}
