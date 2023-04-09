#include "Arduino.h"
#include <HardwareSerial.h>
#include "cansat03.h"
HardwareSerial s(0);     // use UART0 serial debug or unit A-B intercom
HardwareSerial lora(1);  // use UART1
HardwareSerial gps(2);   // use UART2
#define CORE0TASKPRIO 3
#define CORE1TASKPRIO 3
#include "core0.h"
#include "core1.h"



void setup() {
  byte error, address;
  int nDevices;

  s.begin(115200);
  s.println();
  s.println("CANSAT boot....");
  Wire.begin(21, 22);
  s.println("Scanning I2C");
  s.println("IN219               0x40");
  s.println("GY-91 modul MPU6500 0x68");
  s.println("GY-91 modul BMP280  0x76");
  s.println("BME280              0x77");
  s.println("BME280 SDO pin direkt felhúzása nélkül azonos a két BME címe!");
  s.println("");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      s.print("I2C device found at address 0x");
      if (address < 16)
        s.print("0");
      s.print(address, HEX);
      s.println("  !");

      nDevices++;
    } else if (error == 4) {
      s.print("Unknown error at address 0x");
      if (address < 16)
        s.print("0");
      s.println(address, HEX);
    }
  }
  if (nDevices == 0)
    s.println("No I2C devices found\n");
  else
    s.println("done\n");

  s.println("Booting core 0/1");
  core0setup();
  core1setup();

  xTemp = xSemaphoreCreateMutex();
  xSemaphoreGive((xTemp));
  xPressure = xSemaphoreCreateMutex();
  xSemaphoreGive((xPressure));
  xHumidity = xSemaphoreCreateMutex();
  xSemaphoreGive((xHumidity));
  xImuok = xSemaphoreCreateMutex();
  xSemaphoreGive((xImuok));
  xBmpok = xSemaphoreCreateMutex();
  xSemaphoreGive((xBmpok));
  xBmeok = xSemaphoreCreateMutex();
  xSemaphoreGive((xBmeok));
  xAdxlok = xSemaphoreCreateMutex();
  xSemaphoreGive((xAdxlok));
  xInaok = xSemaphoreCreateMutex();
  xSemaphoreGive((xInaok));
  xDsok = xSemaphoreCreateMutex();
  xSemaphoreGive((xDsok));
  xPosx = xSemaphoreCreateMutex();
  xSemaphoreGive((xPosx));
  xPosy = xSemaphoreCreateMutex();
  xSemaphoreGive((xPosy));
  xPosz = xSemaphoreCreateMutex();
  xSemaphoreGive((xPosz));
  xQa = xSemaphoreCreateMutex();
  xSemaphoreGive((xQa));
  xQi = xSemaphoreCreateMutex();
  xSemaphoreGive((xQi));
  xQj = xSemaphoreCreateMutex();
  xSemaphoreGive((xQj));
  xQk = xSemaphoreCreateMutex();
  xSemaphoreGive((xQk));
  xLat = xSemaphoreCreateMutex();
  xSemaphoreGive((xLat));
  xLon = xSemaphoreCreateMutex();
  xSemaphoreGive((xLon));
  xAlt = xSemaphoreCreateMutex();
  xSemaphoreGive((xAlt));
  xGpsok = xSemaphoreCreateMutex();
  xSemaphoreGive((xGpsok));
  xMissionphase = xSemaphoreCreateMutex();
  xSemaphoreGive((xMissionphase));

  xCurrent = xSemaphoreCreateMutex();
  xSemaphoreGive((xCurrent));

  xVoltage = xSemaphoreCreateMutex();
  xSemaphoreGive((xVoltage));
}

void loop() {
  vTaskDelay(1);
}