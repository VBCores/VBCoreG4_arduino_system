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
  CanFD();
  void can_init();
  FDCAN_HandleTypeDef*  get_hfdcan();
  FDCAN_TxHeaderTypeDef create_header(uint32_t ID);
  ~CanFD();
private:
  FDCAN_HandleTypeDef  hfdcan1;
  void MX_FDCAN1_Init(void);
  void CANFD_Start( void );

};

void SystemClock_Config(void);
#endif
