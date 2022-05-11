// -----------------------------------------------------------------------------------
// standard external GPIO library
#pragma once

#include "../../Common.h"

#if defined(GPIO_DEVICE)

#include "Ds2413.h"
#if GPIO_DEVICE != DS2413

#include "Mcp23008.h"
#if GPIO_DEVICE != MCP23008

#include "Mcp23017.h"
#if GPIO_DEVICE != MCP23017

#include "Tca9555.h"
#if GPIO_DEVICE != X9555

#include "Pcf8575.h"
#if GPIO_DEVICE != X8575

class Gpio {
  public:
    bool init();

    void pinMode(int pin, int mode);

    int digitalRead(int pin);

    void digitalWrite(int pin, int value);

  private:
};

extern Gpio gpio;

#endif
#endif
#endif
#endif
#endif

#endif
