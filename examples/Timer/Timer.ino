#include <VBCoreG4_arduino_system.h>

// Пример работы с таймером и stm32duino, 
// по мотивам https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
// В этом примере таймер настроен на 1 герц, для платы VBCore на STM32G746RE 



void setup() {
  Serial.begin(115200);
  //LED_BUILTIN привязан к пину PA5 в библиотеке VBCoreG4_arduino_system
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  // Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *timer = new HardwareTimer(TIM3);
  timer->pause(); // останавливаем таймер перед настройкой
  timer->setOverflow(1, HERTZ_FORMAT); // 1 Hz 
  timer->attachInterrupt(func_timer); // активируем прерывание
  timer->refresh(); // обнулить таймер 
  timer->resume(); // запускаем таймер

}

void loop() {
  // put your main code here, to run repeatedly:
}

void func_timer() // обработчик прерывания
{
  
  digitalWrite(LED1, !digitalRead(LED1));
 
} 
