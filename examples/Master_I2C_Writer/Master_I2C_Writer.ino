#include <VBCoreG4_arduino_system.h>
#include <Wire.h>

// Wire Master Writer

// Демонстрирует использование библиотеки Wire
// Запись данных с master-устройства в slave-устройство по протоколу I2C

//I2С4_SDA подключен к пину PB_7_ALT1
#define pinSDA PB_7_ALT1
//I2С4_SCL подключен к пину PC6
#define pinSCL PC6




void setup()
{
  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Wire.begin();

  //если сообщения отправляются, мы увидим мигающий светодиод
  pinMode(LED2, OUTPUT);
}

byte x = 0;

void loop()
{
  Wire.beginTransmission(5); // передать на устройство 5
  Wire.write("x is ");        // отправляет пять байтов
  Wire.write(x);              // отправляет один байт  
  Wire.endTransmission();    // прекратить передачу

  x++;
  digitalWrite(LED2, !digitalRead(LED2)); //поморгать светодиодом
  delay(200);
}
