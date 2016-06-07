
#ifndef ENDSTOPS_MODULE_H
#define ENDSTOPS_MODULE_H

#include "libs/Module.h"
#include "libs/Pin.h"
#include "mbed.h"

class StepperMotor;
class Gcode;

class NES_Jog : public Module
{
  public:
      NES_Jog();
      void on_module_loaded();
        
  private:
      void load_config();
      uint32_t on_tick(uint32_t dummy);
      void execute_buttons(uint8_t button_data);
      bool debounced_get(int pin);
      uint8_t jog_rate;
      Pin    pins[5];
      float  jog_mm[3];
      float  fast_rates[3];
      float  slow_rates[3];
      uint16_t nes_button_map[8];
      mbed::SPI *spi;
      uint8_t debounce_count;
      uint32_t spi_frequency;

};

#endif
