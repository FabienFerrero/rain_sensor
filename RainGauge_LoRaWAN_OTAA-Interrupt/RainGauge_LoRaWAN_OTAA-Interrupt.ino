#include "Adafruit_SHTC3.h"
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

// Set pin number
#define buttonPin PB5

// Set Interrupt
int ledToggle;
int previousState = HIGH;
unsigned int previousPress;
volatile int buttonFlag, buttonFlag_falseDetect;
const int buttonDebounce = 20;
volatile int lastDetect = 0;

volatile int rainFlag;    // turn on this flag if it is rain
volatile int notRainFlag; // turn on this flag if it is not rain
volatile unsigned int rainGaugeCount = 0;
unsigned long time1 = 0;

uint32_t estimatedNextUplink = 0;

// Set sensor variables
int temper;
int humi;

// Rain Stop Time
uint64_t lastRain = 0; // the last time when it was rain
uint64_t elapsedRain;
uint64_t spendTime; // the remaining time before wake up in period OTAA

bool bucketPositionA = false; // one of the two positions of tipping-bucket
// const double bucketAmount = 0.01610595;   // inches equivalent of ml to trip tipping-bucket

#define OTAA_PERIOD (30000)
// #define RAIN_STOP_TIME   (6000)
/*************************************

   LoRaWAN band setting:
     RAK_REGION_EU433
     RAK_REGION_CN470
     RAK_REGION_RU864
     RAK_REGION_IN865
     RAK_REGION_EU868
     RAK_REGION_US915
     RAK_REGION_AU915
     RAK_REGION_KR920
     RAK_REGION_AS923

 *************************************/
#define OTAA_BAND     (RAK_REGION_US915)
#define OTAA_DEVEUI   {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xD0, 0xD4}
#define OTAA_APPEUI   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define OTAA_APPKEY   {0xA3, 0x5B, 0xBA, 0x58, 0xDA, 0xDE, 0x09, 0x57, 0xFE, 0x3D, 0x4C, 0xBE, 0x4A, 0xCB, 0xA3, 0xEA}

/** Packet buffer for sending */
uint8_t collected_data[64] = {0};

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void recvCallback(SERVICE_LORA_RECEIVE_T *data)
{
  if (data->BufferSize > 0)
  {
    Serial.println("Something received!");
    for (int i = 0; i < data->BufferSize; i++)
    {
      Serial.printf("%x", data->Buffer[i]);
    }
    Serial.print("\r\n");
  }
}

void joinCallback(int32_t status)
{
  Serial.printf("Join status: %d\r\n", status);
}

void sendCallback(int32_t status)
{
  if (status == 0)
  {
    Serial.println("Successfully sent");
  }
  else
  {
    Serial.println("Sending failed");
  }
}

void setup()
{
  Serial.begin(115200, RAK_AT_MODE);

  // Initialize Interrupt
  Serial.println("RAKwireless Arduino Interrupt Example");
  Serial.println("------------------------------------------------------");
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), button_ISR, FALLING);

  buttonFlag = 0;
  buttonFlag_falseDetect = 0;
  lastDetect = 0;
  
  // Initialize SHTC3
//  Serial.println("SHTC3 test");
//  if (!shtc3.begin())
//  {
//    Serial.println("Couldn't find SHTC3");
//     while (1) delay(1);
//  }
//  Serial.println("Found SHTC3 sensor");

  //Initialize AHT20
  Serial.begin(115200);
  Serial.println("Adafruit AHT10/AHT20 demo!");

  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  // Wake-up
  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, PA0);

  // Initialize LoRaWan OTAA
  Serial.println("RAKwireless LoRaWan OTAA Example");
  Serial.println("------------------------------------------------------");

  // OTAA Device EUI MSB first
  uint8_t node_device_eui[8] = OTAA_DEVEUI;
  // OTAA Application EUI MSB first
  uint8_t node_app_eui[8] = OTAA_APPEUI;
  // OTAA Application Key MSB first
  uint8_t node_app_key[16] = OTAA_APPKEY;

  if (!api.lorawan.appeui.set(node_app_eui, 8))
  {
    Serial.printf("LoRaWan OTAA - set application EUI is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.appkey.set(node_app_key, 16))
  {
    Serial.printf("LoRaWan OTAA - set application key is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.deui.set(node_device_eui, 8))
  {
    Serial.printf("LoRaWan OTAA - set device EUI is incorrect! \r\n");
    return;
  }

  if (!api.lorawan.band.set(OTAA_BAND))
  {
    Serial.printf("LoRaWan OTAA - set band is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A))
  {
    Serial.printf("LoRaWan OTAA - set device class is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.njm.set(RAK_LORA_OTAA)) // Set the network join mode to OTAA
  {
    Serial.printf("LoRaWan OTAA - set network join mode is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.join()) // Join to Gateway
  {
    Serial.printf("LoRaWan OTAA - join fail! \r\n");
    return;
  }

  /** Wait for Join success */
  while (api.lorawan.njs.get() == 0)
  {
    Serial.print("Wait for LoRaWAN join...");
    api.lorawan.join();
    delay(10000);
  }

  if (!api.lorawan.adr.set(true))
  {
    Serial.printf("LoRaWan OTAA - set adaptive data rate is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.rety.set(1))
  {
    Serial.printf("LoRaWan OTAA - set retry times is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.cfm.set(1))
  {
    Serial.printf("LoRaWan OTAA - set confirm mode is incorrect! \r\n");
    return;
  }

  /** Check LoRaWan Status*/
  Serial.printf("Duty cycle is %s\r\n", api.lorawan.dcs.get() ? "ON" : "OFF");            // Check Duty Cycle status
  Serial.printf("Packet is %s\r\n", api.lorawan.cfm.get() ? "CONFIRMED" : "UNCONFIRMED"); // Check Confirm status
  uint8_t assigned_dev_addr[4] = {0};
  api.lorawan.daddr.get(assigned_dev_addr, 4);
  Serial.printf("Device Address is %02X%02X%02X%02X\r\n", assigned_dev_addr[0], assigned_dev_addr[1], assigned_dev_addr[2], assigned_dev_addr[3]); // Check Device Address
  Serial.printf("Uplink period is %ums\r\n", OTAA_PERIOD);
  Serial.println("");
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);
}

void uplink_routine()
{
  /** Payload of Uplink */
  uint8_t data_len = 0;
  collected_data[data_len++] = (uint8_t)buttonFlag;
  collected_data[data_len++] = (uint8_t)temper >> 8;
  collected_data[data_len++] = (uint8_t)(temper) & 0xFF;
  collected_data[data_len++] = (uint8_t)humi & 0xFF;

  Serial.println("Data Packet:");
  for (int i = 0; i < data_len; i++)
  {
    Serial.printf("0x%02X ", collected_data[i]);
  }
  Serial.println("");

  /** Send the data package */
  if (api.lorawan.send(data_len, (uint8_t *)&collected_data, 2, true, 1))
  {
    Serial.println("Sending is requested");
  }
  else
  {
    Serial.println("Sending failed");
  }
}

void loop()
{
  // Read SHTC3
//  sensors_event_t humidity, temp;
//  shtc3.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
//  temper = (temp.temperature) * 10;
//  humi = humidity.relative_humidity;
//  Serial.print("Sensors values : temp = ");
//  Serial.print(temper / 10);
//  Serial.println("deg");
//  Serial.print("hum= ");
//  Serial.print(humi);
//  Serial.println("%");

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  temper = (temp.temperature) * 10;
  humi = humidity.relative_humidity;
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  
  Serial.print("Value: ");
  Serial.println(buttonFlag);
  Serial.print(" | False Detect: ");
  Serial.println(buttonFlag_falseDetect);

  // LoRaWAN Uplink
  uplink_routine();
  buttonFlag = 0;

  // Set sleep until the next LoRaWAN Uplink
  Serial.printf("Try sleep %ums..", OTAA_PERIOD);
  estimatedNextUplink = millis() + OTAA_PERIOD;
  api.system.sleep.all(OTAA_PERIOD);

  // Re-check the wake up reason. If the wakeup caused by External Interrupt, go back to sleep mode
  while (estimatedNextUplink > millis())
  {
    uint32_t remainingSleepTime = estimatedNextUplink - millis();
    api.system.sleep.all(remainingSleepTime);
  }

  Serial.println("Wakeup..");
}

void button_ISR()
{
  int _now = millis();
  if ((_now - lastDetect) > buttonDebounce)
  {
    lastDetect = _now;
    buttonFlag++;
  }
  else
  {
    buttonFlag_falseDetect++;
  }
}
