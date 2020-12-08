
/***********************************************************************
 * @file 	:	main.cpp
 * @author	:	( $ _ $ ) 自由的假发 ( $ _ $ )
 * @brief	:	
 * @version:	v1.0
 * @Copyright (C)  2020-11-21  .cdWFVCEL. all right reserved
***********************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "BLEDevice.h"
#include "modules/ntc.h"

/* #includes -------------------------------------------------------------------------- */

/* define  -------------------------------------------------------------------------- */

#define IO_1V8 32
#define LED 25
#define TEMPERATURE_PIN A5
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10       /* Time ESP32 will go to sleep (in seconds) */

/*  ---------------------------------------------------------------------------------------------------------------*/

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a1");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;
/* variable  -------------------------------------------------------------------------- */
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
long irValue;
bool finger_f = false;

double temperature = 0;

RTC_DATA_ATTR int bootCount = 0;

/* func declare -------------------------------------------------------------------------- */
void max30105Init();
void max30105Running();
double getTemperature();

void BLE_Init();
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
void max30105Init()
{
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                    //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
}

double getTemperature()
{
  int sensorValue = analogRead(TEMPERATURE_PIN);
  double voltage = sensorValue * (3.3 / 4096.0);

  // Serial.printf("sensorValue:%d \r\n", (int)sensorValue);
  // char str[16] = {0};
  // dtostrf(voltage, 2, 1, str), Serial.printf("voltage:%s \r\n", str);

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
  return retTemperature;
}

void max30105Running()
{
  irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE;                    //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.printf("IR=%d,BPM=%d,Avg BPM=%d", (int)irValue, (int)beatsPerMinute, beatAvg);
  Serial.println();
  if (irValue < 50000)
  {
    finger_f = false;
    // Serial.println(" No finger?");
  }
  else
  {
    finger_f = true;
  }
}

/*  -------------------------------------------------------------------------- */

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
    // Serial.printf("%d,%d,%d,%d", freeHeap, heapSize, freePsram, psramSize);
    Serial.println("onConnect");
  }

  void onDisconnect(BLEClient *pclient)
  {
    // uint32_t freeHeap = ESP.getFreeHeap();
    // uint32_t heapSize = ESP.getHeapSize();
    // uint32_t freePsram = ESP.getFreePsram();
    // uint32_t psramSize = ESP.getPsramSize();

    // Serial.printf("%d,%d,%d,%d", freeHeap, heapSize, freePsram, psramSize);
    connected = false;
    Serial.println("onDisconnect");
  }
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
    {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  }   // onResult
};    // MyAdvertisedDeviceCallbacks

void BLE_Init()
{
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  Serial.println((char *)pData);
}

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return connected;
}

void BLE_Running()
{
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected)
  {
    // String newValue = "Time since boot: " + String(millis() / 1000);
    // Serial.println("Setting new characteristic value to \"" + newValue + "\"");

    // // Set the characteristic's value to be the array of bytes that is actually a string.
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());

    char json[64] = {0};
    sprintf(json, "{'temp':%d,'IR':%d,'BPM':%d,'ABPM':%d,'finger':%d}", (int)temperature, (int)irValue, (int)beatsPerMinute, beatAvg, finger_f);
    Serial.println(json);
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(json, strlen(json));
  }
  else if (doScan)
  {
    BLEDevice::getScan()->start(0); // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
}

/*  -------------------------------------------------------------------------- */
void setup()
{
  pinMode(IO_1V8, OUTPUT);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  Serial.begin(115200);
  Serial.println("running...");

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  digitalWrite(IO_1V8, HIGH);
  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  max30105Init();
  BLE_Init();
}

uint32_t ble_c = 0; //control BLE transimit frequency
uint32_t adc_c = 0;
// uint32_t sleep_c = 0;

void loop()
{
  for (int i = 0; i < 2000; i++)
  {
    max30105Running();
  }
  temperature = getTemperature();

  if (finger_f)
  {
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }
  }

  BLE_Running();

  // Serial.flush();
  digitalWrite(IO_1V8, LOW);
  Serial.println("sleep...");
  esp_deep_sleep_start();
}
