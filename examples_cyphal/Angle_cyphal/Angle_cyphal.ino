#include <SimpleFOC.h>

#include <VBCoreG4_arduino_system.h>
#include <Libcanard2.h>
#include <cyphal.h>
#include <uavcan/si/sample/angle/Scalar_1_0.h>


CanFD canfd;
FDCAN_HandleTypeDef* hfdcan1;

CyphalInterface* interface;

void error_handler() {Serial.println("error"); while (1) {};}
uint64_t micros_64() {return micros();}

//пины SPI3 - mosi, miso, sclk
SPIClass SPI_3(PC12, PC11, PC10);
//пин chip select (может называться cs, nss, ss) - PA15 
//разрешение магнитного датчика - 14
//регистр чтения углов - по умолчанию 0x3FFF
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);

void setup() {
    Serial.begin(115200);
    canfd.can_init();
    hfdcan1 = canfd.get_hfdcan();

    interface = new CyphalInterface(99);
    interface->setup<G4CAN, SystemAllocator>(hfdcan1);

    sensor.init(&SPI_3);
    Serial.println("sensor ready");
    delay(1000);
}

PREPARE_MESSAGE(uavcan_si_sample_angle_Scalar_1_0, angle)  // создаст angle_buf, angle_transfer_id
#define ANGLE_PORT_ID 1111
void send_angle(float radian) {
    uavcan_si_sample_angle_Scalar_1_0 angle_msg = {
        .timestamp = {.microsecond=micros_64()},
        .radian = radian
    };
    interface->SEND_MSG(uavcan_si_sample_angle_Scalar_1_0, &angle_msg, angle_buf, ANGLE_PORT_ID, &angle_transfer_id);
}

uint64_t last_send = 0;
#define MICROS_SEC 1000000
void loop() {
    interface->loop();
    sensor.update();
    auto now = micros_64();
    int count = 0;
    if ( (now - last_send) > MICROS_SEC ) {
        last_send = now;
        //float some_angle = 3.14;
        send_angle(sensor.getAngle());
       
    }
}