#include <VBCoreG4_arduino_system.h>

#define EN_GATE PB3
float sensitivity = 45; // mV/A

float i_A_offset =0;
float i_B_offset = 0;
float i_C_offset = 0;
void setup() {
  analogReadResolution(12);
  Serial.begin(115200);

  pinMode(PC1, INPUT); //ph A
  pinMode(PC2, INPUT); // ph B
  pinMode(PC3, INPUT); // ph C
  
  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, HIGH);
  delay(10);
  
  //посчитаем offset для каждой из фаз
  for (int i = 0; i < 64; i++) {
    i_A_offset += (3.3f * (float)analogRead(PC1) / 4096.0f);
    i_B_offset += (3.3f * (float)analogRead(PC2)/ 4096.0f);
    i_C_offset += (3.3f * (float)analogRead(PC3) /4096.0f);
    delay(1);
}
//усредняем
i_A_offset /= 64.0f;
i_B_offset /= 64.0f;
i_C_offset /= 64.0f;
}

float i_A, i_B, i_C;
void loop() {
  i_A = (3.3f*(float)analogRead(PC1)/4096.0f - i_A_offset )/(sensitivity/1000); //A
  i_B = (3.3f*(float)analogRead(PC2)/4096.0f- i_B_offset )/(sensitivity/1000); //A
  i_C = (3.3f*(float)analogRead(PC3)/4096.0f- i_C_offset )/(sensitivity/1000); //A

   Serial.print(" A: ");
   Serial.print(i_A);
   Serial.print("\t ");
   Serial.print(" B: ");
   Serial.print(i_B);
   Serial.print("\t");
   Serial.print(" C: ");
   Serial.print(" ");
   Serial.println(i_C);
  delay(100);
}
