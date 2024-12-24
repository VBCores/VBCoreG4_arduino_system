#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#define EN_GATE PB3 // PB3 - включает/отключает драйвер

SPIClass SPI_3(PC12, PC11, PC10);

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  bit_resolution - magnetic sensor resolution
//  angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
float sensitivity = 45.0; // mV/A 

// InlineCurrentSensor constructor
//  - mVpA  - mV per Amp ratio
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
//                                                          phA  phB  phC
InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3); 
HardwareTimer *timer = new HardwareTimer(TIM15);
BLDCMotor motor = BLDCMotor(14); //11 -пар полюсов

//  BLDCDriver3PWM( int phA, int phB, int phC)
//  - phA, phB, phC - A,B,C phase pwm pins
BLDCDriver3PWM driver = BLDCDriver3PWM(PA10, PA9, PA8);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd);}
//Serial.println(digitalRead(PB5)); }
void setup() {

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  Wire.setSDA(PB_7_ALT1);
  Wire.setSCL(PC6);
  Wire.begin();

  while (!Serial) delay(10);  // wait for serial port to open!
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  pinMode(PB5, INPUT);
  pinMode(EN_GATE, OUTPUT);
  pinMode(LED2, OUTPUT);
  

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

  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
 // motor.torque_controller = TorqueControlType::foc_current;
 // motor.controller = MotionControlType::torque;
  // set torque mode:
  // set motion control loop to be used

  motor.controller = MotionControlType::velocity;
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

  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 10; //20
  motor.PID_velocity.D = 0;


  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01f;

  motor.current_limit = 5;
  motor.voltage_limit = 15;
  motor.velocity_limit = 360; //[rad/s] 
 // motor.KV_rating = 14;
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();


 // add target command T
  command.add('T', doTarget, "target current");

  Serial.println(F("Motor ready."));
  _delay(1000);

  timer->pause();
  timer->setOverflow(2000, HERTZ_FORMAT);
  timer->attachInterrupt(FOC_func);
  timer->refresh();
  timer->resume();
}

int t;

float headingVel = 0;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
float ACCEL_VEL_TRANSITION =  (float)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
float ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
float DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
sensors_event_t orientationData , linearAccelData;
void loop() {
  //Serial.println(micros()-t);
  //t = micros();
  // main FOC algorithm function
  //motor.loopFOC();
 /* bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);*/
  // Motion control function
  motor.move();

  // user communication
  command.run();
  //delay(1);
  //Serial.println(micros()-t);
  //delayMicroseconds(1000-micros()+t);
  delayMicroseconds(500);
  
}

void FOC_func(){
  //digitalWrite(LED2, !digitalRead(LED2));
  //Serial.println("ok");
  motor.loopFOC();
  
}

