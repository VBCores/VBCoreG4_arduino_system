#include <SimpleFOC.h>

#include <VBCoreG4_arduino_system.h>   // системный хэдер
#include <Libcanard2.h>                // базовые библиотеки (canard, uavcan, etc.)
#include <cyphal.h>                    // сам libcyphal
#include <uavcan/si/unit/electric_current/Scalar_1_0.h> // тип сообщения, которе будем использовать
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/si/sample/angle/Scalar_1_0.h>
// Настройка fdcan из VBCoreG4
CanFD canfd;
FDCAN_HandleTypeDef* hfdcan1;

// Таймер, по прерыванию которого будем посылать сообщения
uint32_t uptime = 0;
HardwareTimer *timer = new HardwareTimer(TIM3);

float target = 0;
// Класс с API для доступа к cyphal
CyphalInterface* interface;

// Объявим функцию, которую libcyphal будем вызывать для обработки ошибок
void error_handler() {Serial.println("error"); while (1) {};}
uint64_t micros_64() {return micros();}
float t;

SUBSCRIPTION_CLASS_MESSAGE(VoltageReader, uavcan_si_unit_electric_current_Scalar_1_0, 1114)

void VoltageReader::handler(const uavcan_si_unit_electric_current_Scalar_1_0& current, CanardRxTransfer* transfer) {
   // Serial.print(+transfer->metadata.remote_node_id);
   // Serial.print(": ");
    //Serial.println(voltage.volt);
    target = current.ampere;
    //Serial.println(micros()-t);
}
VoltageReader* reader;


#define EN_GATE PB3 // PB3 - включает/отключает драйвер

SPIClass SPI_3(PC12, PC11, PC10);

MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);

float sensitivity = 45.0; // mV/A 

InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3); 

BLDCMotor motor = BLDCMotor(14); //11 -пар полюсов

BLDCDriver3PWM driver = BLDCDriver3PWM(PA10, PA9, PA8);

void setup() {
    Serial.begin(115200);
    // запускаем can
    canfd.can_init();
    hfdcan1 = canfd.get_hfdcan();

    // "Запускаем" cyphal, id нашего узла будет 99
    interface = new CyphalInterface(99);
    // Инициализация - мы находимся на G4, алокатор памяти - системный
    // NOTE: в следующей версии, setup может пропасть и достаточно будет просто сделать
    // new CyphalInterface<G4CAN, SystemAllocator>(99, hfdcan1);
    interface->setup<G4CAN, SystemAllocator>(hfdcan1);
    // Создаем обработчик сообщений, который объявили выше
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
  // driver config
  driver.voltage_power_supply = 15;
  driver.init();
  driver.enable();

  
  // set torque mode:
  motor.torque_controller = TorqueControlType::foc_current; 
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;


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
 
  motor.current_limit = 1.90;
  motor.voltage_limit = 15;
  motor.velocity_limit = 6;
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();


    // Настраиваем таймер на прерывания раз в секунду
    timer->pause();
    timer->setOverflow(1, HERTZ_FORMAT);
    timer->attachInterrupt(hbeat_func);
    timer->refresh();
    timer->resume();
}

uint64_t last_send = 0;
#define MICROS_SEC 1000
void loop() {
    // Вся обработка и fdcan, и cyphal, есть в CyphalInterface::loop
    //Serial.println(target);
    
    
    interface->loop();
    sensor.update();
    auto now = micros_64();
    if ( (now - last_send) > MICROS_SEC ) {
        last_send = now;
        send_angle(sensor.getAngle());  
    }
    motor.loopFOC();
    motor.move(target); 
    //Serial.println(motor.shaftVelocity());
    //delay(1); 
}


PREPARE_MESSAGE(uavcan_node_Heartbeat_1_0, hbeat)
void send_heartbeat() {
    // Заполняем сообщение
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    // Отправляем сообщение
    interface->SEND_MSG(uavcan_node_Heartbeat_1_0, &heartbeat_msg, hbeat_buf, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
}

PREPARE_MESSAGE(uavcan_si_sample_angle_Scalar_1_0, angle)  // создаст angle_buf, angle_transfer_id
#define ANGLE_PORT_ID 1111
void send_angle(float radian) {
  //Serial.println(radian);
    uavcan_si_sample_angle_Scalar_1_0 angle_msg = {
        .timestamp = {.microsecond=micros_64()},
        .radian = radian
    };
    interface->SEND_MSG(uavcan_si_sample_angle_Scalar_1_0, &angle_msg, angle_buf, ANGLE_PORT_ID, &angle_transfer_id);
}
// Функция таймера
void hbeat_func() {
    digitalWrite(LED2, !digitalRead(LED2));
    send_heartbeat();
    uptime += 1;
}