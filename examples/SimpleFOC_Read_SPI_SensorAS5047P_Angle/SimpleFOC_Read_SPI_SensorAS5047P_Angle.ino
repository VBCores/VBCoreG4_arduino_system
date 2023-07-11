//инструкция по установке библиотеки SimpleFOC https://docs.simplefoc.com/library_download
#include <SimpleFOC.h>
#include <VBCoreG4_arduino_system.h>

//пины SPI3 - mosi, miso, sclk
SPIClass SPI_3(PC12, PC11, PC10);
//пин chip select (может называться cs, nss, ss) - PA15 
//разрешение магнитного датчика - 14
//регистр чтения углов - по умолчанию 0x3FFF
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
void setup() {
  Serial.begin(115200);
  sensor.init(&SPI_3);
  Serial.println("sensor ready");
  delay(1000);

}

void loop() {
  sensor.update();
  Serial.println(sensor.getAngle());
  delay(1);
}
