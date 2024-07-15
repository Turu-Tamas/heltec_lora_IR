#include <Arduino.h>
#include <heltec_unofficial.h>
#include <SoftWire.h>
#include "AsyncDelay.h"

const uint8_t I2C_ADDRESS = 0x69;

const uint8_t REG_POWER_CONTROL = 0x00;
const uint8_t REG_RESET = 0x01;
const uint8_t REG_FRAME_RATE = 0x02;
const uint8_t REG_INT_CONTROL = 0x03;
const uint8_t REG_STATUS = 0x04;
const uint8_t REG_STATUS_CLEAR = 0x05;
const uint8_t REG_TEMP = 0x80;

const uint8_t CMD_NORMAL_MODE[] = { 0x00, 0x00 };
const uint8_t CMD_SLEEP_MODE[] = { 0x00, 0x10 };
const uint8_t CMD_FLAG_RESET[] = { 0x01, 0x30 };
const uint8_t CMD_INITIAL_RESET[] = { 0x01, 0x3f };
const uint8_t CMD_SET_FRAMERATE_1[] = { 0x02, 0x01 };
const uint8_t CMD_SET_FRAMERATE_10[] = { 0x02, 0x00 };
const uint8_t CMD_DISABLE_INT[] = { 0x03, 0x00 };
const uint8_t CMD_READ_TEMPS[] = { 0x80 };

byte softWireRxBuffer[128];
byte softWireTxBuffer[4];
SoftWire AMG8833(GPIO_NUM_42, GPIO_NUM_41);

AsyncDelay loopInterval(1000, AsyncDelay::units_t::MILLIS);

#define send_command_nostop(cmd) \
  AMG8833.beginTransmission(I2C_ADDRESS);  \
  AMG8833.write(cmd, sizeof cmd);        \
  IR_debug(AMG8833.endTransmission(false), __LINE__); \

#define send_command(cmd)             \
  AMG8833.beginTransmission(I2C_ADDRESS);  \
  AMG8833.write(cmd, sizeof cmd);        \
  IR_debug(AMG8833.endTransmission(), __LINE__); \

void IR_init();
void IR_read_temps(float buf[64]);
void IR_read_reg(uint8_t reg, size_t nbytes, char buf[]);
void hang();
void IR_debug(u8_t res, int line);
void I2C_reset();

void setup() {
  heltec_setup();
  delay(8000);
  both.println("Start");
  IR_init();
}

uint32_t last_loop = 0;

void loop() {
  heltec_loop();

  if (!loopInterval.isExpired())
    return;
  loopInterval.restart();

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
  Serial.println();
}

void IR_init() {
  SDA; SCL;
  AMG8833.setDelay_us(5);
  AMG8833.setRxBuffer(softWireRxBuffer, sizeof softWireRxBuffer);
  AMG8833.setTxBuffer(softWireTxBuffer, sizeof softWireTxBuffer);
  // AMG8833.setTimeout(1000);
  AMG8833.enablePullups();
  AMG8833.begin();

  AMG8833.sclHigh();
  AMG8833.sdaHigh();
  I2C_reset();

  send_command(CMD_NORMAL_MODE);
  send_command(CMD_INITIAL_RESET);
  // send_command(CMD_FLAG_RESET);
  send_command(CMD_DISABLE_INT);
  send_command(CMD_SET_FRAMERATE_10);
}

void IR_read_reg(uint8_t reg, size_t nbytes, void *buf) {

  memset(buf, 69, nbytes);

  uint8_t cmd[] = { reg };
  send_command_nostop(cmd);
  AMG8833.requestFrom(I2C_ADDRESS, nbytes);
  size_t num_read = AMG8833.readBytes((u8_t *)buf, nbytes);
  if (num_read != nbytes) {
    both.printf("IR_read_reg recieved %u, expected %u", num_read, nbytes);
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

void IR_debug(u8_t res, int line) {
  switch (res)
  {
  case 0:
    return;
    break;
  case 1:
    both.printf("%i: I2C: data too long", line);
    break;
  case 2:
    both.printf("%i: I2C: address NACK", line);
    break;
  case 3:
    both.printf("%i: I2C: data NACK", line);
    break;
  case 4:
    both.printf("%i: I2C: timed out", line);
    break;
  // case 4:
  //   both.printf("%i: I2C: other err", line);
  //   break;
  // case 5:
  //   both.printf("%i: I2C: timed out", line);
  //   break;
  default:
    both.printf("%i: I2C: invalid result value", line);
    break;
  }
  both.println();
  hang();
}

void I2C_reset() {
  AMG8833.sclHigh();
  AMG8833.sdaHigh();

  for (int i = 0; i < 10; i++) {
    delayMicroseconds(AMG8833.getDelay_us());
    AMG8833.sclLow();
    delayMicroseconds(AMG8833.getDelay_us());
    AMG8833.sclHigh();
  }

  AMG8833.stop();
}
