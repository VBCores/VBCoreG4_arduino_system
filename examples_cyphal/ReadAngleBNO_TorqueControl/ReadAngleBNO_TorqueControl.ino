#include <SimpleFOC.h>

#include <VBCoreG4_arduino_system.h>   // системный хэдер
#include <Libcanard2.h>                // базовые библиотеки (canard, uavcan, etc.)
#include <cyphal.h>                    // сам libcyphal
#include <uavcan/si/unit/voltage/Scalar_1_0.h> // тип сообщения, которе будем использовать
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/si/sample/angle/Scalar_1_0.h>
#include <uavcan/si/sample/angular_velocity/Scalar_1_0.h>


#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


CanFD canfd;
FDCAN_HandleTypeDef* hfdcan1;


uint32_t uptime = 0;
HardwareTimer *timer = new HardwareTimer(TIM3);
HardwareTimer *timer_2 = new HardwareTimer(TIM4);
HardwareTimer *timer_3 = new HardwareTimer(TIM2);

float target = 0;
CyphalInterface* interface;
void error_handler() {Serial.println("error"); while (1) {};}
uint64_t micros_64() {return micros();}


SUBSCRIPTION_CLASS_MESSAGE(VoltageReader, uavcan_si_unit_voltage_Scalar_1_0, 1114)
void VoltageReader::handler(const uavcan_si_unit_voltage_Scalar_1_0& voltage, CanardRxTransfer* transfer) {
    target = voltage.volt;
    //Serial.print("target:   ");
    //Serial.println(target);
 
}
VoltageReader* reader;


#define EN_GATE PB3 // PB3 - включает/отключает драйвер

SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
float sensitivity = 45.0; // mV/A 
InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3); 
BLDCMotor motor = BLDCMotor(14); //14 -пар полюсов
BLDCDriver3PWM driver = BLDCDriver3PWM(PA10, PA9, PA8);


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB_7_ALT1);
  Wire.setSCL(PC6);
  Wire.begin();
  
  while (!Serial) delay(10);  // wait for serial port to open!
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }


  canfd.can_init();
  hfdcan1 = canfd.get_hfdcan();
  interface = new CyphalInterface(99);
    interface->setup<G4CAN, SystemAllocator>(hfdcan1);
  reader = new VoltageReader(interface);

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

  driver.voltage_power_supply = 15;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);
  
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  
  
  sensor.init(&SPI_3);
  motor.linkSensor(&sensor);
  

  current_sense.init();
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
 
  motor.current_limit = 2;
  motor.voltage_limit = 15;
  motor.velocity_limit = 6;
  motor.init();
  motor.initFOC();

  delay(1);
// Настраиваем таймер на прерывания раз в секунду
  timer->pause();
  timer->setOverflow(1000, HERTZ_FORMAT);
  timer->attachInterrupt(FOC_func);
  timer->refresh();
  timer->resume();

  timer_2->pause();
  timer_2->setOverflow(1, HERTZ_FORMAT);
  timer_2->attachInterrupt(hbeat_func);
  timer_2->refresh();
  timer_2->resume();
  
  timer_3->pause();
  timer_3->setOverflow(150, HERTZ_FORMAT);
  timer_3->attachInterrupt(move_func);
  timer_3->refresh();
  timer_3->resume();

}
float headingVel = 0;
uint64_t last_send = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board

sensors_event_t orientationData, angVelData;

#define MICROS_SEC 1000

void loop() {
 
  //motor.loopFOC();
  interface->loop();

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  send_data(motor.shaftVelocity(), angVelData.gyro.z, orientationData.orientation.x);
 
  //Serial.println(micros());
  delay(1);

}


PREPARE_MESSAGE(uavcan_node_Heartbeat_1_0, hbeat)
void send_heartbeat() {
    // Заполняем сообщение
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    // Отправляем сообщение
    interface->SEND_MSG(uavcan_node_Heartbeat_1_0, &heartbeat_msg, hbeat_buf, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
}


PREPARE_MESSAGE(uavcan_si_sample_angular_velocity_Scalar_1_0, ang_vel)
PREPARE_MESSAGE(uavcan_si_sample_angle_Scalar_1_0, angle)  // создаст angle_buf, angle_transfer_id


#define MOTOR_VEL_DATA_ID 1111
#define PEND_VEL_DATA_ID 1112
#define PEND_ANG_DATA_ID 1113
void send_data(float motorVel, float pendVel, float pendAng ){
  
   uavcan_si_sample_angular_velocity_Scalar_1_0 motorVel_msg = {
        .timestamp = {.microsecond=micros_64()},
        .radian_per_second = motorVel
    };
    interface->SEND_MSG(uavcan_si_sample_angular_velocity_Scalar_1_0, &motorVel_msg, ang_vel_buf, MOTOR_VEL_DATA_ID, &ang_vel_transfer_id);
  
  uavcan_si_sample_angular_velocity_Scalar_1_0 pendVel_msg = {
        .timestamp = {.microsecond=micros_64()},
        .radian_per_second = pendVel
    };
    interface->SEND_MSG(uavcan_si_sample_angular_velocity_Scalar_1_0, &pendVel_msg, ang_vel_buf, PEND_VEL_DATA_ID, &ang_vel_transfer_id);

  uavcan_si_sample_angle_Scalar_1_0 pendAng_msg = {
        .timestamp = {.microsecond=micros_64()},
        .radian = pendAng
    };
    interface->SEND_MSG(uavcan_si_sample_angle_Scalar_1_0, &pendAng_msg, angle_buf, PEND_ANG_DATA_ID, &angle_transfer_id);

    

}

void hbeat_func() {
    digitalWrite(LED2, !digitalRead(LED2));

    send_heartbeat();
    uptime += 1;
    
    //Serial.println(" beat ");
}
void move_func(){
   motor.move(target);
  //Serial.print(target);
  //Serial.println("  move ");
  }


void FOC_func(){
  
  sensor.update();
  motor.loopFOC();
  
  //Serial.println("  FOC ");
}
