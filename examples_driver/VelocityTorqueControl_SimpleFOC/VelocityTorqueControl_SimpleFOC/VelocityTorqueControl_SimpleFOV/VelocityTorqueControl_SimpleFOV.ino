#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#define EN_GATE PB3 // PB3 - включает/отключает драйвер

SPIClass SPI_3(PC12, PC11, PC10);

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  bit_resolution - magnetic sensor resolution
//  angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);

BLDCMotor motor = BLDCMotor(11); //11 -пар полюсов

//  BLDCDriver3PWM( int phA, int phB, int phC)
//  - phA, phB, phC - A,B,C phase pwm pins
BLDCDriver3PWM driver = BLDCDriver3PWM(PA10, PA9, PA8);



Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd);}
//Serial.println(digitalRead(PB5)); }
void setup() {

Serial.begin(115200);
 motor.useMonitoring(Serial);

  pinMode(PB5, INPUT);
  pinMode(EN_GATE, OUTPUT);
  

  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);
  
  digitalWrite(EN_GATE, HIGH);
  delay(10);
  // driver config
  driver.voltage_power_supply = 15;
  driver.init();
  driver.enable();

  
  // set control loop type to be used
  motor.controller = MotionControlType::velocity; //angle - управление по углу

   // velocity PI controller parameters
  motor.PID_velocity.P = 1.8f;//0.2f;
  motor.PID_velocity.I = 6.5f;//10; //20
  motor.PID_velocity.D = 0;

  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01f; 

  motor.P_angle.P = 20;
  motor.P_angle.output_ramp = 10000;
  // initialise magnetic sensor hardware
  sensor.init(&SPI_3);
  // link the motor to the encoder
  motor.linkSensor(&sensor);
  // link the motor to the driver
  motor.linkDriver(&driver);
  
  motor.current_limit = 1.94;
  motor.voltage_limit = 24;
  motor.velocity_limit - 5;
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();


 // add target command T
  command.add('T', doTarget, "target current");

  Serial.println(F("Motor ready."));
  _delay(1000);
}

int t;
void loop() {
  t = micros();
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();
  delayMicroseconds(100-micros()+t);
}
