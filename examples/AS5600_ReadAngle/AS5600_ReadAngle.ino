#include <Wire.h>
#include <VBCoreG4_arduino_system.h>
//необходимо скачать библиотеку Seeed_Arduino_AS5600-master можно по ссылке https://github.com/Seeed-Studio/Seeed_Arduino_AS5600
//и скопировать ее в libraries
#include <AS5600.h>

//SDA подключен к пину PB_7_ALT1
//#define pinSDA PB_7_ALT1
//SCL подключен к пину PC6
//#define pinSCL PC6

AMS_5600 ams5600;

int ang = 0;

void setup()
{
  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Serial.begin(115200);
  
  Wire.begin();
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ){
            Serial.print("Current Magnitude: ");
            Serial.println(ams5600.getMagnitude());
            break;
        }
        else{
            Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }
}
/*******************************************************
/* Функция: convertRawAngleToDegrees
/* Вход: данные об угле из AMS_5600::getRawAngle
/* Выход: человекочитаемые градусы в виде float
/* Описание: берет необработанный угол и вычисляет
/* его float значение в градусах.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
  /* Исходные данные отображают 0 - 4095 тиков, что составляет 0,087 градуса */
  float retVal = newAngle * 0.087;
  ang = retVal;
  return ang;
}
void loop()
{
    Serial.println(String(convertRawAngleToDegrees(ams5600.getRawAngle()),DEC));
}
