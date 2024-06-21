#include <Wire.h>
#include "ZED-FP9.hpp"

#define ZED_FP9_ADDR 0x42

static emulation::ZED_FP9 rtk(ZED_FP9_ADDR);

void handle_request()
{
  Serial.println();
  Serial.println("OUT >>>>>>>>>>>>");

  rtk.service_request(Wire);
}

void handle_receive(int bytes_received)
{
  Serial.println();
  Serial.print("IN <<<<<<<<<<<<: "); Serial.println(bytes_received);

  rtk.service_receive(Wire, bytes_received);
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  Serial.begin(115200);
  Serial.println("ZED-FP9 Emulator");
  Serial.print("Wire slave tx buf: "); Serial.println(BUFFER_LENGTH);

  Wire.onReceive(handle_receive);
  Wire.onRequest(handle_request);
  Wire.begin(ZED_FP9_ADDR);
}

int itr = 0;
void loop()
{
  delay(1000);
  rtk.update(millis());
  digitalWrite(13, itr++ & 1);
}
