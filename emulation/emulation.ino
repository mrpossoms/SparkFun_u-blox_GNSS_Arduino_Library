#include <Wire.h>
#include "ZED-FP9.hpp"

#define ZED_FP9_ADDR 0x42

emulation::ZED_FP9 rtk;

struct {
  uint8_t sel_reg;
  uint8_t rx_buf[512];
} CTX;

void handle_request()
{

}

void handle_receive(int bytes_received)
{
  if (bytes_received == 1)
  { // we are being addressed first
    CTX.sel_reg = Wire.read();

    Serial.print("Selected reg: "); Serial.println(CTX.sel_reg);
  }
  else
  {
    uint8_t* r_ptr = CTX.rx_buf;
    while (buf - r_ptr < bytes_received)
    {
      r_ptr += Wire.readBytes(r_ptr, bytes_received);
    }

    Serial.print("Received: ");
    for (unsigned i = 0; i < bytes_received; i++)
    {
      Serial.print(CTX.rx_buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }
}

void setup()
{
  Serial.begin(115200);

  Wire.onReceive(handle_receive);
  Wire.onRequest(handle_request);
  Wire.begin(ZED_FP9_ADDR);
}

void loop()
{

}
