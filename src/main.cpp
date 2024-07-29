#define RADIOLIB_DEBUG_PROTOCOL 1

#include <Arduino.h>
#include <heltec_unofficial.h>
#include <Wire.h>
#include "AMG8833.h"
#include <LoRaWAN_ESP32.h>
#include <arduino-timer.h>
#include "config.h"
#include <limits>

#define DBG_NO_LORA 0

const int MILLISEC = 1;
const int SECOND = 1000 * MILLISEC;
const int MINUTE = 60 * SECOND;
const int HOUR = 60 * MINUTE;
const int CELSIUS = 4;

const int IR_READ_INTERVAL = 300 * MILLISEC;
const int UPLOAD_INTERVAL = 20 * SECOND;
const bool SLEEP_IR_SENSOR = IR_READ_INTERVAL > 3 * SECOND;
const int HUMAN_TEMP = 27.5 * CELSIUS;
const int IR_AVG_MINVAL = 21 * CELSIUS;
const int IR_AVG_MAXVAL = 29 * CELSIUS;

const int IR_I2C_ADDR = 0xe9;
AMG8833 IR_sensor(&Wire1, SDA, SCL, IR_I2C_ADDR);

u64_t temp_totals[64] = { 0 };
u64_t IR_num_reads = 0;

const bool DEBUG_LORA_DUTY_CYCLE = false;

LoRaWANNode* node;

auto upload_timer = timer_create_default();
auto IR_read_timer = timer_create_default();

void hang();
bool upload(void *arg);
bool IR_read(void *arg);
void setup_lora();

typedef AMG8833_error_t Error;
#define IR_command(command)         \
  do {                              \
    Error err = command;            \
    if (err) {                      \
      both.println("IR error:");    \
      both.println(err.str());      \
      delay(100);                   \
    }                               \
    else break;                     \
  } while (true)                    \


void setup() {

  u32_t start_time = millis();

  heltec_setup();
  delay(6000);
  both.println("start");

  IR_command(IR_sensor.begin());
  assert(IR_READ_INTERVAL >= 100);

  if (IR_READ_INTERVAL >= 1000)
    IR_command(IR_sensor.set_framerate(FPS1));
  else
    IR_command(IR_sensor.set_framerate(FPS10));

  #if !DBG_NO_LORA
    setup_lora();
  #endif

  upload_timer.every(UPLOAD_INTERVAL, upload);
  IR_read_timer.every(IR_READ_INTERVAL, IR_read);
}

void loop() {
  heltec_loop();
  upload_timer.tick();
  IR_read_timer.tick();
}

void hang() {
  both.println("Hanged");
  while (true) { heltec_loop(); }
}

void setup_lora() {
  // initialize radio
  Serial.println("Radio init");
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    both.println("Radio did not initialize.");
    hang();
  }

  if (!persist.isProvisioned())
    persist.provision();

  node = persist.manage(&radio);

  RADIOLIB(node->setTxPower(EU868.powerMax));
  while (!node->isActivated()) {
    Serial.println("joining");
    RADIOLIB(node->activateOTAA());
  }
  both.println("Successfully joined");
  node->setDutyCycle(DEBUG_LORA_DUTY_CYCLE);
}

bool IR_read(void *arg) {
  short temps[64];
  Error err = IR_sensor.read_pixels_raw(temps);
  // return false; would mean that the timer stops executing this
  if (err) {
    Serial.println("Read failed:");
    Serial.println(err.str());
    return true;
  }

  IR_num_reads++;
  for (int i = 0; i < 64; i++) {
    temp_totals[i] += temps[i];
  }

  if (IR_num_reads % 10 == 0) {
    short min_tmp = 10000, max_tmp = 0, sum = 0;
    int num_above = 0;
    for (int i = 0; i < 64; i++) {
      min_tmp = min(temps[i], min_tmp);
      max_tmp = max(temps[i], max_tmp);
      sum += temps[i];
      num_above += temps[i] > HUMAN_TEMP;
    }

    Serial.printf("min %i, max %i, avg %i, num: %i\n", min_tmp, max_tmp, sum / 64, num_above);
  }
  return true;
}

// convert inval to u8 with zero corresponding to minval
// and 255 corresponding to maxval and a linear relationship in between
//
u8_t tempsum2u8(u64_t inval) {
  u64_t out = inval; // [SHORTMIN, SHORTMAX]
  out = max(out, (u64_t)IR_AVG_MINVAL * IR_num_reads);
  out = min(out, (u64_t)IR_AVG_MAXVAL * IR_num_reads);
  // [minval, maxval]
  out -= IR_AVG_MINVAL * IR_num_reads;

  // always multiply first
  out *= 255;
  out /= IR_AVG_MAXVAL - IR_AVG_MINVAL;
  out /= IR_num_reads;
  // [0; 255], voilá
  return out;
}

bool upload(void *arg) {
  Serial.printf("\nuploading %i reads\n", IR_num_reads);

  // split data into two messages because 65 bytes is too long
  byte message1[33]; // 32 pixel data + 1 metadata (battery life, id, etc.)
  byte message2[32]; // 32 pixel data

  for (int i = 0; i < 64; i++) {
    byte *msg = i < 32 ? message1 : message2;
    int idx = i < 32 ? i : i - 32;
    msg[idx] = tempsum2u8(temp_totals[i]);

    if (i % 8 == 0)
      Serial.println();
    Serial.printf("%02x ", msg[idx]);
  }
  Serial.println();

  message1[32] = heltec_battery_percent();
  Serial.printf("battery: %i%", message1[32]);

  #if !DBG_NO_LORA
    uint8_t downlinkData[256];
    size_t lenDown = sizeof(downlinkData);
    RADIOLIB(node->sendReceive(message1, sizeof(message1), 1, downlinkData, &lenDown));
    RADIOLIB(node->sendReceive(message2, sizeof(message2), 1, downlinkData, &lenDown));
  #endif

  for (u64_t &val : temp_totals)
    val = 0;
  IR_num_reads = 0;
  return true;
}

bool print_temps(void *arg) {
  float temps[64];
  auto err = IR_sensor.read_pixels(temps);
  if (err) {
    both.println("Error reading pixels:");
    both.println(err.str());
  }
  for (size_t y = 0; y < 8; y++) {
    String line;
    for (size_t x = 0; x < 8; x++) {
      line.concat(temps[y*8 + x]);
      line += " ";
    }
    Serial.println(line);
  }
  Serial.println();
  return true;
}
