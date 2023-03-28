#include <Wire.h>
#include "ZED-FP9.hpp"

#define ZED_FP9_ADDR 0x42

static emulation::ZED_FP9 rtk;

// struct {
//   uint8_t sel_reg;
//   uint8_t rx_buf[512];
// } CTX;

void handle_request()
{
  Serial.println();
  Serial.println("Request: ");

  rtk.service_request(Wire);
}

void handle_receive(int bytes_received)
{
  Serial.println();
  Serial.print("Received: "); Serial.println(bytes_received);

  rtk.service_receive(Wire, bytes_received);
/*
  if (bytes_received == 1)
  { // we are being addressed first
    CTX.sel_reg = Wire.read();

    Serial.print("Selected reg: "); Serial.println(CTX.sel_reg, HEX);
  }
  else
  {
    uint8_t* s_ptr = CTX.rx_buf;
    uint8_t* r_ptr = CTX.rx_buf;
    while ((r_ptr - s_ptr) < bytes_received)
    {
      r_ptr += Wire.readBytes(r_ptr, bytes_received);
    }

    for (unsigned i = 0; i < bytes_received; i++)
    {
      Serial.print(CTX.rx_buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }
*/
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  Serial.begin(115200);
  Serial.println("ZED-FP9 Emulator");

  Wire.onReceive(handle_receive);
  Wire.onRequest(handle_request);
  Wire.begin(ZED_FP9_ADDR);
}

int itr = 0;
void loop()
{
  delay(1000);
  Serial.print(".");
  digitalWrite(13, itr++ & 1);
}
