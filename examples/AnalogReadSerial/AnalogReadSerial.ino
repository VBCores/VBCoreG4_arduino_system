#include <VBCoreG4_arduino_system.h>

 /*AnalogReadSerial
  Считывает аналоговый сигнал с пина PC5, печатает результат в Serial Monitor.
  Графическое представление доступно с помощью Serial Plotter (меню Tools > Serial Plotter).
*/

// процедура настройки запускается один раз при нажатии кнопки reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
}

// инициализировать последовательную связь на скорости 115200 бит в секунду:
void loop() {
  // считывание входного сигнала на аналоговом выводе PC5:
  int sensorValue = analogRead(PC5);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1);        // задержка между считываниями для стабильности
}
