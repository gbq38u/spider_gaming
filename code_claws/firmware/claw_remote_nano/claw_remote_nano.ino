// Пульт: Arduino Nano 5 V + два XY-джойстика + nRF24L01 через YL-105.
// Левая клешня: VRx=A0, VRy=A1, SW=D3.
// Правая клешня: VRx=A2, VRy=A3, SW=D4.
// Радио: CE=D9, CSN=D10, SCK=D13, MOSI=D11, MISO=D12.
// Линии CE/CSN/SCK/MOSI идут через делители 1 кОм + 1,8 кОм
// по схеме пульта. YL-105 питается от 5 V, само радио — от адаптера.

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

namespace {
constexpr uint8_t kLeftX = A0, kLeftY = A1, kLeftButton = 3;
constexpr uint8_t kRightX = A2, kRightY = A3, kRightButton = 4;
constexpr uint8_t kAxisPins[4] = {kLeftX, kLeftY, kRightX, kRightY};
constexpr int8_t kAxisSign[4] = {+1, -1, +1, -1};
// Если конкретный джойстик движется наоборот, поменяйте знак нужной оси.
constexpr uint16_t kMagic = 0xB6C2;
constexpr uint8_t kVersion = 1;
constexpr uint8_t kPipe[5] = {'C', 'L', 'A', 'W', '1'};
constexpr uint32_t kSendEveryMs = 20;
constexpr uint32_t kDebounceMs = 35;

struct __attribute__((packed)) Packet {
  uint16_t magic;
  uint8_t version;
  uint8_t gripBits;  // бит 0: левая закрыта; бит 1: правая закрыта
  int8_t lx, ly, rx, ry;  // [-100; 100], Y > 0 означает «вперёд/вверх»
  uint16_t sequence;
};
static_assert(sizeof(Packet) == 10, "RF packet size changed");

struct Button {
  uint8_t pin;
  bool raw;
  bool stable;
  uint32_t changedAt;
};

RF24 radio(9, 10);
int16_t center[4] = {512, 512, 512, 512};
Button leftButton{kLeftButton, false, false, 0};
Button rightButton{kRightButton, false, false, 0};
uint8_t gripBits = 0;  // после включения обе клешни открыты
uint16_t sequenceNumber = 0;
uint32_t lastSendAt = 0;

void calibrateCenters() {
  // Не трогайте джойстики во время включения пульта.
  int32_t sums[4] = {0, 0, 0, 0};
  for (uint8_t sample = 0; sample < 64; ++sample) {
    for (uint8_t axis = 0; axis < 4; ++axis) sums[axis] += analogRead(kAxisPins[axis]);
    delay(2);  // только однократная калибровка в setup()
  }
  for (uint8_t axis = 0; axis < 4; ++axis) center[axis] = sums[axis] / 64;
}

int8_t readAxis(uint8_t axis) {
  const int32_t delta = analogRead(kAxisPins[axis]) - center[axis];
  const int32_t span = delta >= 0 ? 1023 - center[axis] : center[axis];
  int32_t value = span > 0 ? delta * 100 / span : 0;
  value *= kAxisSign[axis];
  if (value > 100) value = 100;
  if (value < -100) value = -100;
  // Мёртвая зона убирает дрожание в центре, остальной ход остаётся плавным.
  if (value >= -7 && value <= 7) return 0;
  if (value > 0) value = (value - 7) * 100 / 93;
  else value = (value + 7) * 100 / 93;
  return static_cast<int8_t>(value);
}

void updateButton(Button& button, uint8_t bit, uint32_t now) {
  const bool pressed = digitalRead(button.pin) == LOW;
  if (pressed != button.raw) {
    button.raw = pressed;
    button.changedAt = now;
  }
  if (pressed != button.stable && now - button.changedAt >= kDebounceMs) {
    button.stable = pressed;
    if (pressed) gripBits ^= bit;  // одно переключение на одно нажатие
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(kLeftButton, INPUT_PULLUP);
  pinMode(kRightButton, INPUT_PULLUP);
  leftButton.raw = leftButton.stable = (digitalRead(kLeftButton) == LOW);
  rightButton.raw = rightButton.stable = (digitalRead(kRightButton) == LOW);
  calibrateCenters();

  SPI.begin();
  if (!radio.begin()) {
    Serial.println(F("nRF24 не найден: проверьте питание, SPI и YL-105"));
    while (true) delay(1000);
  }
  radio.setPayloadSize(sizeof(Packet));
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);  // PA/LNA на дистанции до 10 м
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);
  radio.openWritingPipe(kPipe);
  radio.stopListening();
  Serial.println(F("Пульт клешней готов"));
}

void loop() {
  const uint32_t now = millis();
  updateButton(leftButton, 0x01, now);
  updateButton(rightButton, 0x02, now);
  if (now - lastSendAt < kSendEveryMs) return;
  lastSendAt = now;

  Packet packet{};
  packet.magic = kMagic;
  packet.version = kVersion;
  packet.gripBits = gripBits;
  packet.lx = readAxis(0);
  packet.ly = readAxis(1);
  packet.rx = readAxis(2);
  packet.ry = readAxis(3);
  packet.sequence = sequenceNumber++;
  radio.write(&packet, sizeof(packet));
}
