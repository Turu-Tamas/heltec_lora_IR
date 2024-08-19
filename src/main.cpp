#define RADIOLIB_DEBUG_PROTOCOL 1

#include <Arduino.h>
#include <heltec_unofficial.h>
#include <Wire.h>
#include "AMG8833.h"
#include <LoRaWAN_ESP32.h>
#include <arduino-timer.h>
#include "config.h"
#include <limits>
#include <SD.h>

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
const int IR_MINVAL = 20 * CELSIUS;
const int IR_MAXVAL = 32 * CELSIUS;
const int IR_READS_PER_SD_WRITE = 30;
u8_t IR_READ_BUFFER[IR_READS_PER_SD_WRITE * 64];

const int IR_I2C_ADDR = 0xe9;
AMG8833 IR_sensor(&Wire1, SDA, SCL, IR_I2C_ADDR);

u64_t temp_totals[64] = { 0 };
u64_t IR_num_reads = 0;

const bool DEBUG_LORA_DUTY_CYCLE = false;
const bool DEBUG_SD = true;

LoRaWANNode* node;
SPIClass SD_SPI;

auto upload_timer = timer_create_default();
auto IR_read_timer = timer_create_default();

void hang();
bool upload(void *arg);
bool IR_read(void *arg);
void setup_lora();
u8_t temp2u8(u16_t inval);
void write_SD();

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

enum SensorErrorType {
  SENSOR_ERROR_NONE,
  SENSOR_SD_CARD_WRITE_ERROR,
  SENSOR_IR_READ_ERROR,
  SENSOR_SD_CARD_SETUP_ERROR,
  SENSOR_IR_SETUP_ERROR,
};

SensorErrorType error = SENSOR_ERROR_NONE;

void setup() {

  u32_t start_time = millis();

  heltec_setup();
  delay(6000);
  both.println("start");

  SD_SPI.begin(GPIO_NUM_33, GPIO_NUM_35, GPIO_NUM_34, SPI_CS1_GPIO_NUM);

  if (!SD.begin(SPI_CS1_GPIO_NUM, SD_SPI)) {
    both.println("SD card begin failed");
    error = SENSOR_SD_CARD_SETUP_ERROR;
    hang();
  }

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
  if (IR_num_reads == IR_READS_PER_SD_WRITE) {
    write_SD();
    IR_num_reads = 0;
  }
}

bool upload_error(void *_) {
  byte msg[1] = { error };
  RADIOLIB(node->sendReceive(msg, 1));
  return true;
}

void hang() {
  both.println("Hanged");
  auto error_upload_timer = timer_create_default();
  error_upload_timer.every(UPLOAD_INTERVAL, upload_error);
  while (true) {
    heltec_loop();
    error_upload_timer.tick();
  }
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

  for (int i = 0; i < 64; i++) {
    size_t idx = IR_num_reads * 64 + i;
    IR_READ_BUFFER[idx] = temp2u8(temps[i]);
  }
  IR_num_reads++;

  if (IR_num_reads % 10 == 0) {
    short min_tmp = 10000, max_tmp = 0, sum = 0;
    int num_above = 0;
    for (int i = 0; i < 64; i++) {
      min_tmp = min(temps[i], min_tmp);
      max_tmp = max(temps[i], max_tmp);
      sum += temps[i];
      num_above += temps[i] > HUMAN_TEMP;
    }

    Serial.printf(
      "min %i, max %i, avg %i, num: %i\n",
      min_tmp, max_tmp, sum / 64, num_above
    );
  }
  return true;
}

// convert inval to u8 with zero corresponding to minval
// and 255 corresponding to maxval and a linear relationship in between
u8_t temp2u8(u16_t inval) {
  u16_t out = inval; // [SHORTMIN, SHORTMAX]
  out = max(out, (u16_t)(IR_MINVAL * IR_num_reads));
  out = min(out, (u16_t)(IR_MAXVAL * IR_num_reads));
  // [minval, maxval]
  out -= IR_MINVAL * IR_num_reads;

  // always multiply first
  out *= 255;
  out /= IR_MAXVAL - IR_MINVAL;
  out /= IR_num_reads;
  // [0; 255], voilá
  return out;
}

bool upload(void *arg) {
  Serial.printf("\nuploading %i reads\n", IR_num_reads);

  byte mintemp = 255;
  byte maxtemp = 0;
  u8_t *last_frame = IR_READ_BUFFER + (IR_READS_PER_SD_WRITE - 1) * 64;
  for (int i = 0; i < 64; i++) {
    mintemp = min(mintemp, last_frame[i]);
    maxtemp = max(maxtemp, last_frame[i]);
  }

  byte bat_percent = heltec_battery_percent();
  byte message[4] = {
    error,
    bat_percent,
    mintemp,
    maxtemp
  };

  #if !DBG_NO_LORA
    RADIOLIB(node->sendReceive(message, sizeof(message)));
  #endif

  for (u64_t &val : temp_totals)
    val = 0;
  IR_num_reads = 0;
  return true;
}

void write_SD() {
  String filename = String("/sd/") + millis() + ".bin";
  File file = SD.open(filename, FILE_WRITE);
  if (!file || file.write((byte *)IR_READ_BUFFER, sizeof(IR_READ_BUFFER))) {
    both.println("SD write or file open failed");
    error = SENSOR_SD_CARD_WRITE_ERROR;
    hang();
  }
  file.close();

  if (DEBUG_SD) {
    byte buf[sizeof(IR_READ_BUFFER)];
    file = SD.open(filename, FILE_READ);
    file.readBytes((char *)buf, sizeof(buf));
    for (int i = 0; i < sizeof(IR_READ_BUFFER); i++) {
      assert(IR_READ_BUFFER[i] == buf[i]);
    }
    file.close();
  }
}
