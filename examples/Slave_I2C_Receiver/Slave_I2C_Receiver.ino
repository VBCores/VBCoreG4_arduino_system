#include <VBCoreG4_arduino_system.h>
#include <Wire.h>

// Wire Slave Receiver

// Демонстрирует использование библиотеки Wire
// Получает данные как slave-устройство от master-устройства по протоколу I2C 



//SDA подключен к пину PB_7_ALT1
#define pinSDA PB_7_ALT1
//SCL подключен к пину PC6
#define pinSCL PC6


void setup()
{
  
 Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  //в качестве аргумента begin передаем адрес slave-устройства
  Wire.begin(5);                // подключение к шине i2c с адресом 5 
  Wire.onReceive(receiveEvent); // регистрация события, аргумент - функция, обрабоатывающая событие
  Serial.begin(115200);
}

void loop()
{
  delay(100);
}

// функция, которая выполняется всякий раз, когда данные получены от master-устройства
// эта функция регистрируется как событие, см. setup()
void receiveEvent(int howMany)
{
  Serial.println("ok");
  while(1 < Wire.available()) // цикл по всем элементам
  {
    char c = Wire.read(); // получаем байт в виде символа
    Serial.print(c);         // печатаем символ
  }
  int x = Wire.read();    // получить байт в виде целого числа
  Serial.println(x);         // вывести целое число
}
