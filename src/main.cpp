#include <SimpleFOC.h>
extern "C" {
#include "can2040.h"
#include "libcanard/canard.h"
}

#include <cstdlib>  // Для malloc и free

// CAN2040 и libcanard
static struct can2040 cbus;
static CanardInstance canard;
static uint8_t canard_memory_pool[1024];

// Cyphal параметры
#define NODE_ID 42
#define TRANSFER_ID_TIMEOUT_USEC 1000000
#define CYCLIC_PORT_ID 1234

// Cyphal идентификатор и счетчики
static uint8_t transfer_id = 0;
const uint32_t CAN_TARGET_ANGLE_SUB_ID = 0x100; // Условный идентификатор для управления углом

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(28, 27, 26, 15);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Прочие переменные
double target_angle = 0;


// CAN обработчик
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  Serial.print("Notify");
  Serial.println(notify);
  Serial.print("ID Can Message");
  Serial.println(msg->id);
  if (notify == CAN2040_NOTIFY_RX) {
    size_t payload_size = static_cast<size_t>(msg->dlc);
    CanardFrame frame = {
    .extended_can_id = msg->id,           // ID сообщения CAN
    .payload = {
      payload_size,      //размер данных
      .data = msg->data,                // Указатель на данные
          
    }
};

    CanardRxTransfer transfer;
    CanardMicrosecond timestamp = 0;  // Используйте подходящее значение времени

    // вызов
    int32_t result = canardRxAccept(
        &canard,                   // Экземпляр Canard
        timestamp,                 // Метка времени (например, 0, если не используется)
        &frame,                    // Указатель на структуру CanardFrame
        0,                         // Индекс интерфейса (обычно 0, если интерфейс один)
        &transfer,                 // Указатель для результата
        nullptr                    // Указатель на подписку (nullptr, если не используется)
    );
Serial.print("Result");
Serial.println(result);
// Обработка результата
if (result > 0) {
    if (transfer.metadata.port_id == CAN_TARGET_ANGLE_SUB_ID) {
        // Преобразуем payload в const void*
        const void* payload_data = transfer.payload.data;
        memcpy(&target_angle, payload_data, sizeof(float));
        Serial.print("Received target_angle: ");
        Serial.println(target_angle);
      }
    }
  }
}

// Отправка Cyphal-сообщений
void sendCanData() {
  float current_angle = motor.shaft_angle; // Текущий угол
  uint8_t payload[4];
  memcpy(payload, &current_angle, sizeof(float));  // Копируем данные в payload

  // Инициализация метаданных
  CanardTransferMetadata metadata = {
      .priority = CanardPriorityNominal,
      .transfer_kind = CanardTransferKindMessage,
      .port_id = CYCLIC_PORT_ID,
      .remote_node_id = CANARD_NODE_ID_UNSET,
      .transfer_id = transfer_id++,  // Инкрементируем идентификатор передачи
  };

  // Определяем полезную нагрузку
  CanardPayload canard_payload = {
      .size = sizeof(payload),  
      .data = payload,          // Указатель на данные
       // Размер данных
  };
CanardTxQueue tx_queue = {
    .capacity = 8,             // Максимальное количество кадров в очереди
    .mtu_bytes = 64,           // Размер максимального кадра (например, 64 байта)
    .size = 0,                 // Изначально очередь пуста
    .priority_root = NULL,     // Очередь приоритетов пуста
    .deadline_root = NULL,     // Очередь дедлайнов пуста
    .memory = {0},             // Инициализация ресурса памяти, по умолчанию можно оставить пустым
    .user_reference = NULL,    // Референс пользователя, можно оставить NULL
    .stats = {0}               // Статистика очереди, инициализируем нулями
};
  // Вызов canardTxPush с правильными параметрами
  int32_t result = canardTxPush(
      &tx_queue,
      &canard,                  // Экземпляр Canard
      0,                        // Время отправки (можно передать 0 или время в микросекундах)
      &metadata,                // Метаданные
      canard_payload,           // Полезная нагрузка
      0                         // Время отправки (можно передать 0 или текущее время)
  );

  if (result < 0) {
    Serial.println("Error sending CAN data");
  } else {
    Serial.println("CAN data sent successfully");
  }
}
void my_free(void* user_reference, size_t size, void* pointer) {
      free(pointer);  // Используем стандартный free
  }

void* my_malloc(void* user_reference, size_t size) {
      return malloc(size);  // Используем стандартный malloc
  }
void setup() {
  
  delay(3000);
  Serial.begin(115200);

  // Настройка CAN2040
  uint32_t sys_clock = 125000000, bitrate = 500000;
  uint32_t gpio_rx = 8, gpio_tx = 9;
  can2040_setup(&cbus, 0);  // Используем PIO 0
  can2040_callback_config(&cbus, can2040_cb); // Регистрируем обработчик
  can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);

 

  // Обертка для функции освобождения памяти
    
  // Определите массив для хранения памяти
 

  // Определите структуру для выделения и освобождения памяти
  CanardMemoryResource canard_memory_resource = {
      .user_reference = nullptr,
      .deallocate = my_free,  // Может быть NULL или указателем на ваш пользовательский объект
      .allocate = my_malloc,      // Используем обертку для malloc
            // Используем обертку для free
  };

  // Инициализируем экземпляр Canard
  canard = canardInit(canard_memory_resource);

// Инициализация экземпляра Canard
  
  canard.node_id = NODE_ID;

  // Настройка двигателя
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 32;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.2;
  motor.PID_velocity.D = 0.0001;
  motor.LPF_velocity.Tf = 0.12;
  motor.velocity_limit = 25;
  motor.current_limit = 1;
  motor.voltage_limit = 11;

  motor.init();
  motor.initFOC();

  Serial.println("Setup complete.");
}

void loop() {
  motor.loopFOC();
  motor.move(target_angle);
  Serial.print("Target_Angle");
  Serial.println(target_angle);
  // Отправка данных
  sendCanData();

  delay(10);
}
