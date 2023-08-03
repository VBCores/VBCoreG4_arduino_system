#include <VBCoreG4_arduino_system.h>   // системный хэдер
#include <Libcanard2.h>                // базовые библиотеки (canard, uavcan, etc.)
#include <cyphal.h>                    // сам libcyphal
#include <uavcan/si/unit/voltage/Scalar_1_0.h> // тип сообщения, которе будем использовать
#include <uavcan/node/Heartbeat_1_0.h>
// Настройка fdcan из VBCoreG4
CanFD canfd;
FDCAN_HandleTypeDef* hfdcan1;

// Таймер, по прерыванию которого будем посылать сообщения
uint32_t uptime = 0;
HardwareTimer *timer = new HardwareTimer(TIM3);

// Класс с API для доступа к cyphal
CyphalInterface* interface;

// Объявим функцию, которую libcyphal будем вызывать для обработки ошибок
void error_handler() {Serial.println("error"); while (1) {};}
uint64_t micros_64() {return micros();}

// Макрос, объявляющий класс VoltageReader, обрабатывающий ПОЛУЧЕНИЕ uavcan_si_unit_voltage_Scalar_1_0
SUBSCRIPTION_CLASS_MESSAGE(VoltageReader, uavcan_si_unit_voltage_Scalar_1_0, 1111)
// Если все же пользоваться макросами (что я рекомендую), то остается только реализовать функцию-обработчик получения сообщений:
// Сигнатура у них всегда (const ТИП_СООБЩЕНИЯ&, CanardRxTransfer*). Обратите внимание, что первое это константная ссылка, а второе указатель
void VoltageReader::handler(const uavcan_si_unit_voltage_Scalar_1_0& voltage, CanardRxTransfer* transfer) {
    Serial.print(+transfer->metadata.remote_node_id);
    Serial.print(": ");
    Serial.println(voltage.volt);
}
VoltageReader* reader;

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

    // Настраиваем таймер на прерывания раз в секунду
    timer->pause();
    timer->setOverflow(1, HERTZ_FORMAT);
    timer->attachInterrupt(hbeat_func);
    timer->refresh();
    timer->resume();
}

void loop() {
    // Вся обработка и fdcan, и cyphal, есть в CyphalInterface::loop
    interface->loop();
}

/*
 * Макрос, прячущий шаблонный код для объявления всякого для ОТПРАВКИ сообщений
 * Раскрывается в:

uint8_t hbeat_buf[uavcan_node_Heartbeat_1_0_EXTENT_BYTES_];
CanardTransferID hbeat_transfer_id = 0;

 * Объявлен в cyphal/cyphal.h
 */
PREPARE_MESSAGE(uavcan_node_Heartbeat_1_0, hbeat)
void send_heartbeat() {
    // Заполняем сообщение
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    // Отправляем сообщение
    interface->SEND_MSG(uavcan_node_Heartbeat_1_0, &heartbeat_msg, hbeat_buf, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
}

// Функция таймера
void hbeat_func() {
    digitalWrite(LED2, !digitalRead(LED2));
    send_heartbeat();
    uptime += 1;
}

/*
 * Итого:
 * 1) Создаем CyphalInterface с правильными параметрами для обработки fdcan/can и cyphal
 * 2) Объявляем классы-обработчики для сообщений, которые будем ПОЛУЧАТЬ
 * 3) Объявляем буферы и transfer_id для сообщений, которые будем ОТПРАВЛЯТЬ
 * 4) В цикле вызываем interface->loop
 * 5) ???
 * 6) Profit
 *
 * Код (особенно пункты 2 и 3) можно сильно сократить с помощью макросов.
 * cyphal/subscriptions/subscription.h - макросы для подписок на сообщения
 * cyphal/cyphal.tpp - для обработки и отправки сообщений
 *
 * Иначе можно просто честно наследоваться от классов и писать все самому (пример выше в комментариях).
 *
 * Отдельный PREPARE_MESSAGE - чтоб объявить буферы и тд.
 * Он самый спорный тк создает глобальные переменные и экономит всего одну строку. Пример и с ним, и без него есть выше,
 * решайте сами пользоваться или нет.
 */