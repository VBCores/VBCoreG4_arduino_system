#include <VBCoreG4_arduino_system.h>
#include <stm32g4xx_hal_fdcan.h>

#include <SimpleFOC.h>


SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
InlineCurrentSense current_sense = InlineCurrentSense(0.045, 1, PC1, PC2, PC3);
//InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);
float shunt_resistor = 0.045; //Om

float i_A_offset =0;
float i_B_offset = 0;
float i_C_offset = 0;
//Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void setup() {
analogReadResolution(12);
Serial.begin(115200);
 motor.useMonitoring(Serial);

  pinMode(PB3, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);
  digitalWrite(PB3, HIGH);
  digitalWrite(PB13, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB15, HIGH);

  pinMode(PC1, INPUT);
  pinMode(PC2, INPUT);
  pinMode(PC3, INPUT);
  pinMode(PB5, INPUT);
  // driver config
  driver.voltage_power_supply = 15;
  driver.init();

  for (int i = 0; i < 64; i++) {
    i_A_offset += (3.3f * (float)analogRead(PC1) / 4096.0f);
    i_B_offset += (3.3f * (float)analogRead(PC2)/ 4096.0f);
    i_C_offset += (3.3f * (float)analogRead(PC3) /4096.0f);
    delay(1);
  }
  i_A_offset /= 64.0f;
  i_B_offset /= 64.0f;
  i_C_offset /= 64.0f;
  // set torque mode:
  motor.torque_controller = TorqueControlType::foc_current; 
  // set motion control loop to be used
   motor.controller = MotionControlType::torque;

   // velocity PI controller parameters
  motor.PID_velocity.P = 1.8f;//0.2f;
  motor.PID_velocity.I = 6.5f;//10; //20
  motor.PID_velocity.D = 0;

   
  motor.velocity_limit = 20;
  motor.current_limit = 2;
  motor.P_angle.P = 20;
  
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01f; 


  // initialise magnetic sensor hardware
  sensor.init(&SPI_3);
  // link the motor to the encoder
  motor.linkSensor(&sensor);
  // link the motor to the driver
  motor.linkDriver(&driver);
  current_sense.init();
  current_sense.linkDriver(&driver);
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();


   // add target command T
//  command.add('T', doTarget, "target current");

  Serial.println(F("Motor ready."));
//  Serial.println(F("Set the target current using serial terminal:"));
  motor.target = 1;
  _delay(1000);
  motor.target = 0;
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->pause(); // останавливаем таймер перед настройкой
  timer->setOverflow(1000, HERTZ_FORMAT); // 1 Hz 
  timer->attachInterrupt(func_timer); // активируем прерывание
  timer->refresh(); // обнулить таймер 
  timer->resume(); // запускаем таймер
 
}
int t;
float i_A, i_B, i_C;
void loop() {
  t = micros();
  
  //command.run();

  //I = U/R
  i_A = (3.3f*(float)analogRead(PC1)/4096.0f - i_A_offset )/shunt_resistor; //A
  i_B = (3.3f*(float)analogRead(PC2)/4096.0f- i_B_offset )/shunt_resistor; //A
  i_C = (3.3f*(float)analogRead(PC3)/4096.0f- i_C_offset )/shunt_resistor; //A

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


void func_timer(){
 
// motor.loopFOC();
// motor.move();

}












