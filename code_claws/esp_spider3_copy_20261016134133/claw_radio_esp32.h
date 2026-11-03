#pragma once
// Добавьте этот файл в папку основного ESP32-скетча и #include "claw_radio_esp32.h".
// Плата v13: J19/J20/J21 = PCA1(0x40), физ. каналы 6/5/4;
//             J22/J23/J24 = PCA2(0x41), физ. каналы 6/5/4.
// Назначение: 19/22 поворот, 20/23 подъём, 21/24 захват.
// J_RF: CE=GPIO25, CSN=GPIO26, SCK=18, MOSI=23, MISO=19.
// J_RF.2 = 5 V только на ВХОД YL-105, не на радио.

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace ClawRadio {
namespace detail {
constexpr uint16_t kMagic = 0xB6C2;
constexpr uint8_t kVersion = 1;
constexpr uint8_t kPipe[5] = {'C', 'L', 'A', 'W', '1'};
constexpr uint8_t kChannel[6] = {6, 5, 4, 6, 5, 4};
constexpr uint32_t kStepMs = 20;
constexpr uint32_t kLostLinkMs = 300;
constexpr float kYawSpeedUsPerSecond = 420.0f;
constexpr float kPitchSpeedUsPerSecond = 420.0f;
constexpr float kGripSpeedUsPerSecond = 500.0f;

// Начальные пределы намеренно умеренные. Подгоните их на собранных клешнях.
// Порядок: левая поворот/подъём/захват, правая поворот/подъём/захват.
constexpr int16_t kMinUs[6] = {1250, 1250, 1250, 1250, 1250, 1250};
constexpr int16_t kMaxUs[6] = {1750, 1750, 1750, 1750, 1750, 1750};
constexpr int16_t kOpenUs[2] = {1350, 1650};
constexpr int16_t kClosedUs[2] = {1650, 1350};
// Зеркальная установка серв справа. Если движение неверное, смените знак.
constexpr int8_t kYawDirection[2] = {+1, -1};
constexpr int8_t kPitchDirection[2] = {+1, -1};

struct __attribute__((packed)) Packet {
  uint16_t magic;
  uint8_t version;
  uint8_t gripBits;
  int8_t lx, ly, rx, ry;
  uint16_t sequence;
};
static_assert(sizeof(Packet) == 10, "RF packet size changed");

RF24 radio(25, 26);
Adafruit_PWMServoDriver* pca[2] = {nullptr, nullptr};
SemaphoreHandle_t busMutex = nullptr;
TaskHandle_t taskHandle = nullptr;
float currentUs[6] = {1500, 1500, 1500, 1500, 1500, 1500};
float desiredUs[6] = {1500, 1500, 1350, 1500, 1500, 1650};
uint16_t lastSentUs[6] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
Packet command{};
uint32_t lastPacketAt = 0;
uint32_t lastStepAt = 0;
bool ready = false;
bool received = false;

float clampPulse(float us, uint8_t joint) {
  if (us < kMinUs[joint]) return kMinUs[joint];
  if (us > kMaxUs[joint]) return kMaxUs[joint];
  return us;
}

float moveToward(float from, float to, float step) {
  if (to > from + step) return from + step;
  if (to < from - step) return from - step;
  return to;
}

bool valid(const Packet& p) {
  return p.magic == kMagic && p.version == kVersion &&
         (p.gripBits & 0xFC) == 0 &&
         p.lx >= -100 && p.lx <= 100 && p.ly >= -100 && p.ly <= 100 &&
         p.rx >= -100 && p.rx <= 100 && p.ry >= -100 && p.ry <= 100;
}
}  // namespace detail

inline void update();
inline void task(void*) {
  for (;;) {
    update();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Вызывайте ПОСЛЕ Wire.begin(21,22), pca.begin() и pca.setPWMFreq(50).
// busMutex должен также защищать все записи ног в те же PCA9685.
inline bool begin(Adafruit_PWMServoDriver& leftPca,
                  Adafruit_PWMServoDriver& rightPca,
                  SemaphoreHandle_t busMutex) {
  if (busMutex == nullptr) return false;
  detail::pca[0] = &leftPca;
  detail::pca[1] = &rightPca;
  detail::busMutex = busMutex;
  SPI.begin(18, 19, 23, 26);
  if (!detail::radio.begin()) return false;
  detail::radio.setPayloadSize(sizeof(detail::Packet));
  detail::radio.setChannel(76);
  detail::radio.setDataRate(RF24_250KBPS);
  detail::radio.setPALevel(RF24_PA_MIN);
  detail::radio.setCRCLength(RF24_CRC_16);
  detail::radio.setAutoAck(true);
  detail::radio.openReadingPipe(1, detail::kPipe);
  detail::radio.startListening();
  detail::lastStepAt = millis();
  detail::ready = true;
  if (xTaskCreate(task, "claw_radio", 4096, nullptr, 1,
                  &detail::taskHandle) != pdPASS) {
    detail::ready = false;
    return false;
  }
  return true;
}

// Работает в отдельной задаче ESP32. Не вызывайте вручную из loop().
inline void update() {
  if (!detail::ready) return;
  const uint32_t now = millis();
  while (detail::radio.available()) {
    detail::Packet incoming{};
    detail::radio.read(&incoming, sizeof(incoming));
    if (detail::valid(incoming)) {
      detail::command = incoming;
      detail::lastPacketAt = now;
      detail::received = true;
    }
  }
  if (!detail::received) return;  // до первой команды сервы не двигаются

  const uint32_t elapsed = now - detail::lastStepAt;
  if (elapsed < detail::kStepMs) return;
  detail::lastStepAt = now;
  const float dt = (elapsed > 50 ? 50 : elapsed) / 1000.0f;
  const bool link = (now - detail::lastPacketAt <= detail::kLostLinkMs);

  if (link) {
    const int8_t axes[4] = {detail::command.lx, detail::command.ly,
                            detail::command.rx, detail::command.ry};
    for (uint8_t side = 0; side < 2; ++side) {
      const uint8_t yaw = side * 3;
      const uint8_t pitch = yaw + 1;
      detail::desiredUs[yaw] = detail::clampPulse(
          detail::desiredUs[yaw] + axes[side * 2] * 0.01f *
          detail::kYawSpeedUsPerSecond * dt * detail::kYawDirection[side], yaw);
      detail::desiredUs[pitch] = detail::clampPulse(
          detail::desiredUs[pitch] + axes[side * 2 + 1] * 0.01f *
          detail::kPitchSpeedUsPerSecond * dt * detail::kPitchDirection[side], pitch);
      const bool closed = (detail::command.gripBits & (1u << side)) != 0;
      detail::desiredUs[yaw + 2] = closed ? detail::kClosedUs[side]
                                               : detail::kOpenUs[side];
    }
  } else {
    // Остановить поворот и подъём в фактической позиции даже если запись
    // предыдущего шага I²C задержалась из-за работы ног.
    for (uint8_t side = 0; side < 2; ++side) {
      detail::desiredUs[side * 3] = detail::currentUs[side * 3];
      detail::desiredUs[side * 3 + 1] = detail::currentUs[side * 3 + 1];
    }
  }
  // При потере связи положение и состояние захвата сохраняются; движение
  // поворота/подъёма останавливается после 300 мс без корректного пакета.
  if (xSemaphoreTake(detail::busMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  for (uint8_t joint = 0; joint < 6; ++joint) {
    const float speed = (joint % 3 == 2) ? detail::kGripSpeedUsPerSecond
                      : (joint % 3 == 1) ? detail::kPitchSpeedUsPerSecond
                                         : detail::kYawSpeedUsPerSecond;
    detail::currentUs[joint] = detail::moveToward(
        detail::currentUs[joint], detail::desiredUs[joint], speed * dt);
    const uint16_t pulseUs = static_cast<uint16_t>(detail::currentUs[joint] + 0.5f);
    if (pulseUs != detail::lastSentUs[joint]) {
      detail::pca[joint / 3]->writeMicroseconds(detail::kChannel[joint], pulseUs);
      detail::lastSentUs[joint] = pulseUs;
    }
  }
  xSemaphoreGive(detail::busMutex);
}

inline bool connected() {
  return detail::ready && detail::received &&
         millis() - detail::lastPacketAt <= detail::kLostLinkMs;
}
}  // namespace ClawRadio
