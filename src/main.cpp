#include <Arduino.h>
#include <heltec_unofficial.h>
#include <Wire.h>

const int I2C_SDA = 41;
const int I2C_SCL = 42;
const uint8_t I2C_ADDRESS = 0x69;

const uint8_t REG_POWER_CONTROL = 0x00;
const uint8_t REG_RESET = 0x01;
const uint8_t REG_FRAME_RATE = 0x02;
const uint8_t REG_INT_CONTROL = 0x03;
const uint8_t REG_STATUS = 0x04;
const uint8_t REG_STATUS_CLEAR = 0x05;
const uint8_t REG_TEMP = 0x80;

const uint8_t CMD_NORMAL_MODE[] = { 0x00, 0x00 };
const uint8_t CMD_INITIAL_RESET[] = { 0x01, 0x3f };
const uint8_t CMD_DISABLE_INT[] = { 0x03, 0x00 };
const uint8_t CMD_SET_FRAMERATE_1[] = { 0x02, 0x01 };
const uint8_t CMD_READ_TEMPS[] = { 0x80 };


#define send_command(cmd)             \
  Wire.beginTransmission(I2C_ADDRESS);  \
  Wire.write(cmd, sizeof cmd);        \
  Wire.endTransmission()

void IR_init();
void IR_read_temps(float buf[64]);
void IR_read_reg(uint8_t reg, size_t nbytes, char buf[]);
void hang();

void setup() {
  heltec_setup();
  IR_init();
}

uint32_t last_loop = 0;

void loop() {
  heltec_loop();

  uint32_t current_time = millis();
  if (current_time - last_loop < 1000) {
    return;
  }
  last_loop = current_time;

  float temps[64];
  IR_read_temps(temps);
  for (size_t y = 0; y < 8; y++) {
    String line;
    for (size_t x = 0; x < 8; x++) {
      line.concat(temps[y*8 + x]);
      line += " ";
    }
    Serial.println(line);
  }
}

void IR_init() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  digitalWrite(I2C_SDA, HIGH);
  digitalWrite(I2C_SCL, HIGH);

  send_command(CMD_NORMAL_MODE);
  send_command(CMD_INITIAL_RESET);
  send_command(CMD_DISABLE_INT);
  send_command(CMD_SET_FRAMERATE_1);

  delay(2000);
}

void IR_read_reg(uint8_t reg, size_t nbytes, void *buf) {
  uint8_t cmd[] = { reg };
  send_command(cmd);
  Wire.requestFrom(I2C_ADDRESS, nbytes);

  size_t i = 0;
  for (; i < nbytes && Wire.available(); i++) {
    ((uint8_t *)buf)[i] = Wire.read();
  }
  if (i != nbytes) {
    Serial.println("IR_read_reg recieved fewer bytes than expected");
    hang();
  }
}

void IR_read_temps(float buf[64]) {
  short raw_buf[64];
  IR_read_reg(REG_TEMP, 128, raw_buf);
  for (size_t y = 0; y < 8; y++) {
    for (size_t x = 0; x < 8; x++) {
      buf[y*8 + x] = (float)raw_buf[y*8 + x] * 0.25; 
    }
  }
}

void hang() { 
    both.println("Hanged");
    while (true) { heltec_loop(); }
  }
