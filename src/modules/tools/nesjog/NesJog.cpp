/* Add the following to the config file

nes_jog_module_enable         true

nes_spi_channel               0   # valid channels are 0 or 1

#nes_clock_pin                do not set, depends on SPI channel, SPI0=P0_15, SPI1=P0_7
nes_latch_pin                 0.23
#nes_data0_pin                do not set, depends on SPI channel, SPI0=P0_18, SPI1=P0_9
#nes_data1_pin                nc # Only available on turbo controllers
#nes_data2_pin                nc # Only available on turbo controllers

alpha_fast_jog_rate_mm_s      1500  # X-axis jog rate in mm/min
beta_fast_jog_rate_mm_s       1500
gamma_fast_jog_rate_mm_s      100

alpha_slow_jog_rate_mm_s      100
beta_slow_jog_rate_mm_s       100
gamma_slow_jog_rate_mm_s      10

# Define button actions, 
# N_plus or N_minus for increasing or decreasing axis moves
# "go_home_all" to move all axes to zero
# "set_zero_all" to zero all axes at current location
# "toggle_jog_rate" to switch between fast or slow jog rates

nes_a                         z_plus
nes_b                         z_minus
nes_select                    toggle_jog_rate
nes_start                     set_zero_all
nes_up                        y_plus
nes_down                      y_minus
nes_left                      x_minus
nes_right                     x_plus

*/

#include "libs/Module.h"
#include "libs/Kernel.h"
//#include "modules/communication/utils/Gcode.h"
//#include "modules/robot/Conveyor.h"
//#include "modules/robot/ActuatorCoordinates.h"
#include "NesJog.h"
//#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib
#include "Robot.h"
#include "Stepper.h"
#include "Config.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "StreamOutputPool.h"
#include "StepTicker.h"
#include "BaseSolution.h"
#include "SerialMessage.h"

#include <ctype.h>




#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

#define nes_jog_module_enable_checksum   CHECKSUM("nes_jog_module_enable")

#define nes_clock_pin_checksum           CHECKSUM("nes_clock_pin")
#define nes_latch_pin_checksum           CHECKSUM("nes_latch_pin")
#define nes_data1_pin_checksum           CHECKSUM("nes_data0_pin")
#define nes_data2_pin_checksum           CHECKSUM("nes_data1_pin")
#define nes_data3_pin_checksum           CHECKSUM("nes_data2_pin")

#define nes_spi_channel_checksum         CHECKSUM("nes_spi_channel")

#define alpha_fast_jog_rate_mm_checksum  CHECKSUM("alpha_fast_jog_rate_mm_s")
#define beta_fast_jog_rate_mm_checksum   CHECKSUM("beta_fast_jog_rate_mm_s")
#define gamma_fast_jog_rate_mm_checksum  CHECKSUM("gamma_fast_jog_rate_mm_s")

#define alpha_slow_jog_rate_mm_checksum  CHECKSUM("alpha_slow_jog_rate_mm_s")
#define beta_slow_jog_rate_mm_checksum   CHECKSUM("beta_slow_jog_rate_mm_s")
#define gamma_slow_jog_rate_mm_checksum  CHECKSUM("gamma_slow_jog_rate_mm_s")

#define alpha_jog_mm_checksum            CHECKSUM("alpha_jog_mm")
#define beta_jog_mm_checksum             CHECKSUM("beta_jog_mm")
#define gamma_jog_mm_checksum            CHECKSUM("gamma_jog_mm")

#define nes_a_checksum                   CHECKSUM("nes_a")
#define nes_b_checksum                   CHECKSUM("nes_b")
#define nes_select_checksum              CHECKSUM("nes_select")
#define nes_start_checksum               CHECKSUM("nes_start")
#define nes_up_checksum                  CHECKSUM("nes_up")
#define nes_down_checksum                CHECKSUM("nes_down")
#define nes_left_checksum                CHECKSUM("nes_left")
#define nes_right_checksum               CHECKSUM("nes_right")

#define x_plus_checksum                  CHECKSUM("x_plus")
#define x_minus_checksum                 CHECKSUM("x_minus")
#define y_plus_checksum                  CHECKSUM("y_plus")
#define y_minus_checksum                 CHECKSUM("y_minus")
#define z_plus_checksum                  CHECKSUM("z_plus")
#define z_minus_checksum                 CHECKSUM("z_minus")

#define zero_all_checksum                CHECKSUM("zero_all")
#define go_home_all_checksum             CHECKSUM("go_home_all")
#define toggle_jog_rate_checksum         CHECKSUM("toggle_jog_rate")

#define NES_BUTTONS    (8)

#define NES_BUTTON_A   (7)
#define NES_BUTTON_B   (6)
#define NES_BUTTON_E   (5)
#define NES_BUTTON_T   (4)
#define NES_BUTTON_U   (3)
#define NES_BUTTON_D   (2)
#define NES_BUTTON_L   (1)
#define NES_BUTTON_R   (0)

#define BUTTON_BIT_A   (1<<NES_BUTTON_A)
#define BUTTON_BIT_B   (1<<NES_BUTTON_B)
#define BUTTON_BIT_E   (1<<NES_BUTTON_E)
#define BUTTON_BIT_T   (1<<NES_BUTTON_T)
#define BUTTON_BIT_U   (1<<NES_BUTTON_U)
#define BUTTON_BIT_D   (1<<NES_BUTTON_D)
#define BUTTON_BIT_L   (1<<NES_BUTTON_L)
#define BUTTON_BIT_R   (1<<NES_BUTTON_R)

#define NES_PIN_CLOCK  (0)
#define NES_PIN_LATCH  (1)
#define NES_PIN_DATA1  (2)
#define NES_PIN_DATA2  (3)
#define NES_PIN_DATA3  (4)

#define X_AXIS         (0)
#define Y_AXIS         (1)
#define Z_AXIS         (2)

#define POSITIVE       (1)
#define NEGATIVE       (0)

NES_Jog::NES_Jog()
{
  this->jog_rate = 0;
  this->debounce_count = 10;
  
  // Set frequency to 250KHz
  this->spi_frequency = 250000;
}

// Attach controler read tickFrequency times per second
void NES_Jog::on_module_loaded()
{
  // Do not do anything if not enabled
  if ( THEKERNEL->config->value(nes_jog_module_enable_checksum)->by_default(true)->as_bool() == false ) 
    {
      delete this;
      return;
    }

  // Call the on_tick method ten times per second
  THEKERNEL->slow_ticker->attach(10, this, &NES_Jog::on_tick );

  // Load the settings from the config file
  this->load_config();
}

// Get config
void NES_Jog::load_config()
{
  this->pins[NES_PIN_CLOCK].from_string(THEKERNEL->config->value(nes_clock_pin_checksum)->by_default("nc")->as_string())->as_output();
  this->pins[NES_PIN_LATCH].from_string(THEKERNEL->config->value(nes_latch_pin_checksum)->by_default("nc")->as_string())->as_input();
  this->pins[NES_PIN_DATA1].from_string(THEKERNEL->config->value(nes_data1_pin_checksum)->by_default("nc")->as_string())->as_input();
  this->pins[NES_PIN_DATA2].from_string(THEKERNEL->config->value(nes_data2_pin_checksum)->by_default("nc")->as_string())->as_input();
  this->pins[NES_PIN_DATA3].from_string(THEKERNEL->config->value(nes_data3_pin_checksum)->by_default("nc")->as_string())->as_input();

  // Latch Pin is active high
  this->pins[1].set(false);

  this->fast_rates[X_AXIS] = THEKERNEL->config->value(alpha_fast_jog_rate_mm_checksum )->by_default(1500)->as_number();
  this->fast_rates[Y_AXIS] = THEKERNEL->config->value(beta_fast_jog_rate_mm_checksum  )->by_default(1500)->as_number();
  this->fast_rates[Z_AXIS] = THEKERNEL->config->value(gamma_fast_jog_rate_mm_checksum )->by_default(100)->as_number();

  this->slow_rates[X_AXIS] = THEKERNEL->config->value(alpha_slow_jog_rate_mm_checksum )->by_default(100)->as_number();
  this->slow_rates[Y_AXIS] = THEKERNEL->config->value(beta_slow_jog_rate_mm_checksum  )->by_default(100)->as_number();
  this->slow_rates[Z_AXIS] = THEKERNEL->config->value(gamma_slow_jog_rate_mm_checksum )->by_default(10)->as_number();

  this->jog_mm[X_AXIS] = THEKERNEL->config->value(alpha_jog_mm_checksum )->by_default(0.1f)->as_number();
  this->jog_mm[Y_AXIS] = THEKERNEL->config->value(beta_jog_mm_checksum  )->by_default(0.1f)->as_number();
  this->jog_mm[Z_AXIS] = THEKERNEL->config->value(gamma_jog_mm_checksum )->by_default(0.1f)->as_number();

  this->nes_button_map[NES_BUTTON_R] = THEKERNEL->config->value(nes_right_checksum )->by_default(x_plus_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_L] = THEKERNEL->config->value(nes_left_checksum  )->by_default(x_minus_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_D] = THEKERNEL->config->value(nes_down_checksum  )->by_default(y_minus_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_U] = THEKERNEL->config->value(nes_up_checksum    )->by_default(y_plus_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_T] = THEKERNEL->config->value(nes_start_checksum )->by_default(zero_all_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_E] = THEKERNEL->config->value(nes_select_checksum)->by_default(toggle_jog_rate_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_B] = THEKERNEL->config->value(nes_b_checksum     )->by_default(z_minus_checksum)->as_number();
  this->nes_button_map[NES_BUTTON_A] = THEKERNEL->config->value(nes_a_checksum     )->by_default(z_plus_checksum)->as_number();
  
  
  // Select which SPI channel to use
  int spi_channel = THEKERNEL->config->value(nes_spi_channel_checksum)->by_default(0)->as_number();
  PinName miso;
  PinName mosi;
  PinName sclk;
  if(spi_channel == 0) 
  {
      // Channel 0
      mosi=P0_18; miso=P0_17; sclk=P0_15;
  } 
  else 
  {
      // Channel 1
      mosi=P0_9;  miso=P0_8;  sclk=P0_7;
  }

  delete spi;
  spi = new mbed::SPI(mosi, miso, sclk);
  
  // Spi settings: 8 bits, mode 0 (default)
  spi->format(8);
  
  // Set frequency
  spi->frequency(this->spi_frequency);
  
}

// Runs every update cycle
uint32_t NES_Jog::on_tick(uint32_t dummy)
{
  uint8_t button_data = 255;
  
  // latch the controller buttons
  this->pins[NES_PIN_LATCH].set(true);
  wait_us(1); 
  this->pins[NES_PIN_LATCH].set(false);
  
  // Read from the controller
  button_data = spi->write(0);
  
  // Act on button presses
  this->execute_buttons(button_data);
  return 0;
}

void NES_Jog::execute_buttons(uint8_t button_data)
{
  // Go through the button byte and execute their assigned actions
  for (uint8_t button_index = 0; button_index < NES_BUTTONS; button_index++)
  {
    // If the button's bit is low, find its assigned action and execute
    if (! (button_data & ( 1 << button_index)))
    {
      // Switch on the button's assigned action
      switch (nes_button_map[button_index])
      {
        case x_plus_checksum:
          // Positive x-axis move
          STEPPER[X_AXIS]->move(POSITIVE, this->jog_mm[X_AXIS]*STEPS_PER_MM(X_AXIS), 0);
          
        case x_minus_checksum:
          // Negative x-axis move
          STEPPER[X_AXIS]->move(NEGATIVE, this->jog_mm[X_AXIS]*STEPS_PER_MM(X_AXIS), 0);
          
        case y_plus_checksum:
          // Positive x-axis move
          STEPPER[Y_AXIS]->move(POSITIVE, this->jog_mm[Y_AXIS]*STEPS_PER_MM(Y_AXIS), 0);
          
        case y_minus_checksum:
          // Negative y-axis move
          STEPPER[Y_AXIS]->move(NEGATIVE, this->jog_mm[Y_AXIS]*STEPS_PER_MM(Y_AXIS), 0);
          
        case z_plus_checksum:
          // Positive z-axis move
          STEPPER[Z_AXIS]->move(POSITIVE, this->jog_mm[Z_AXIS]*STEPS_PER_MM(Z_AXIS), 0);
          
        case z_minus_checksum:
          // Negative z-axis move
          STEPPER[Z_AXIS]->move(NEGATIVE, this->jog_mm[Z_AXIS]*STEPS_PER_MM(Z_AXIS), 0);
          
        case toggle_jog_rate_checksum:
          // Toggle the jog rate between fast or slow
          this->jog_rate ^= 1;
          
        case zero_all_checksum:
          // Zero all the axes
          THEKERNEL->robot->reset_position_from_current_actuator_position();
          
      }
    }
  }
}

// Use for any buttons that have a direct connection, not the sampled buttons from a shift register
bool NES_Jog::debounced_get(int pin)
{
    uint8_t debounce = 0;
    while(this->pins[pin].get()) {
        if ( ++debounce >= this->debounce_count ) {
            // pin triggered
            return true;
        }
    }
    return false;
}
