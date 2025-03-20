#ifndef VBCoreG4_arduino_system_h
#define VBCoreG4_arduino_system_h
#include "stm32_def.h"
#include "stm32g4xx_hal_fdcan.h"
#include "Arduino.h"

#define STM32_G
#define HAL_FDCAN_MODULE_ENABLED

#define LED1 PD2
#define LED2 PA5
#define USR_BTN PC13
#define pinSDA PB_7_ALT1
#define pinSCL PC6 

class CanFD
{ 
public:
  CanFD(): hfdcan1() {};
  FDCAN_HandleTypeDef* get_hfdcan() {
    return &hfdcan1;
  }

  void init();
  void write_default_params();
  void write_500kb_params();
  void write_default_params_classic();
  void default_start();
  void apply_config();

private:
  FDCAN_HandleTypeDef hfdcan1;
};

void SystemClock_Config(void);
#endif
