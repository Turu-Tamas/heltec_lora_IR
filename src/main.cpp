#include <Arduino.h>
#include <heltec_unofficial.h>
#include <Wire.h>
#include "AMG8833.h"
#include <LoRaWAN_ESP32.h>
#include <arduino-timer.h>

const int MILLISEC = 1;
const int SECOND = 1000 * MILLISEC;
const int MINUTE = 60 * SECOND;
const int HOUR = 60 * MINUTE;

const int IR_READ_INTERVAL = 100 * MILLISEC;
const int UPLOAD_INTERVAL = HOUR;
const bool SLEEP_IR_SENSOR = IR_READ_INTERVAL > 3 * SECOND;

const int IR_I2C_ADDR = 0xe9;
AMG8833 IR_sensor(&Wire1, SDA, SCL, IR_I2C_ADDR);

u64_t temp_totals[64] = { 0 };
int IR_num_reads = 0;

LoRaWANNode* node;

auto upload_timer = timer_create_default();
auto IR_read_timer = timer_create_default();

void hang();
bool upload(void *arg);
bool IR_read(void *arg);
void setup_lora();

typedef AMG8833_error_t Error;
#define IR_command(command)   \
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

  setup_lora();

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
  if (state != RADIOLIB_ERR_NONE)
    both.println("Radio did not initialize.");

  node = persist.manage(&radio);

  if (!node->isActivated())
    both.println("Could not join network.");
  
  node->setDutyCycle(false);
}

bool IR_read(void *arg) {
  
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
