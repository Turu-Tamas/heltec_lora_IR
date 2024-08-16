#include "AMG8833.h"
#include <Arduino.h>

typedef AMG8833_error_t Error;

const char *AMG8833_error_messages[] = {
    "Success",
    "data too long to fit in transmit buffer",
    "received NACK on transmit of address",
    "received NACK on transmit of data",
    "other error",
    "timed out",
    "sensor is sleeping"
};

const uint8_t REG_POWER_CONTROL = 0x00;
const uint8_t REG_RESET = 0x01;
const uint8_t REG_FRAME_RATE = 0x02;
const uint8_t REG_INT_CONTROL = 0x03;
const uint8_t REG_STATUS = 0x04;
const uint8_t REG_STATUS_CLEAR = 0x05;
const uint8_t REG_PIXELS = 0x80;
 
const byte CMD_NORMAL_MODE[] =      { 0x00, 0x00 };
const byte CMD_SLEEP_MODE[] =       { 0x00, 0x10 };
const byte CMD_FLAG_RESET[] =       { 0x01, 0x30 };
const byte CMD_INITIAL_RESET[] =    { 0x01, 0x3f };
const byte CMD_SET_FRAMERATE_1[] =  { 0x02, 0x01 };
const byte CMD_SET_FRAMERATE_10[] = { 0x02, 0x00 };
const byte CMD_DISABLE_INT[] =      { 0x03, 0x00 };
const byte CMD_READ_TEMPS[] =       { 0x80 };

Error AMG8833::send_command(const byte cmd[2]) {
    if (!this->awake)
        return Error::SLEEPING;
    wire->beginTransmission(this->i2c_addr);
    wire->write(cmd, 2);
    return wire->endTransmission();
}

AMG8833_error_t AMG8833::read_register(const byte reg, byte buf[], size_t num_bytes) {
    if (!this->awake)
        return Error::SLEEPING;
    wire->beginTransmission(this->i2c_addr);
    wire->write(reg);
    Error err = wire->endTransmission(false);
    if (err)
        return err;
    wire->requestFrom(this->i2c_addr, num_bytes);
    size_t num_read = wire->readBytes(buf, num_bytes);
    if (num_read != num_bytes)
        return Error::OTHER_ERROR;
    return Error::SUCCESS;
}

Error AMG8833::begin() {

    this->wire->begin(sda, scl, 400*1000);

    Error err;
    err = send_command(CMD_NORMAL_MODE);
    if (err)
        return err;
    err = send_command(CMD_INITIAL_RESET);
    if (err)
        return err;
    err = send_command(CMD_FLAG_RESET);
    if (err)
        return err;
    err = send_command(CMD_DISABLE_INT);
    if (err)
        return err;
    err = send_command(CMD_SET_FRAMERATE_10);;
    if (err)
        return err;
    delay(50);
    return Error::SUCCESS;
}

AMG8833_error_t AMG8833::read_pixels_raw(short buf[64]) {
    return read_register(REG_PIXELS, (byte *)buf, 128);
}

Error AMG8833::read_pixels(float out_buf[64]) {
    short raw_buf[64];
    Error err = read_register(REG_PIXELS, (byte *)raw_buf, 128);
    if (err)
        return err;
    for (size_t y = 0; y < 8; y++) {
        for (size_t x = 0; x < 8; x++) {
            out_buf[y*8 + x] = (float)raw_buf[y*8 + x] * 0.25; 
        }
    }

    return Error::SUCCESS;
}

Error AMG8833::sleep() {
    if (!awake)
        return Error::SUCCESS;
    Error err = send_command(CMD_SLEEP_MODE);
    if (!err)
        this->awake = false;
    return err;
}

Error AMG8833::wake() {
    if (awake)
        return Error::SUCCESS;
    Error err;
    err = send_command(CMD_NORMAL_MODE);
    if (err)
        return err;
    
    delay(50);
    err = send_command(CMD_INITIAL_RESET);
    if (err)
        return err;

    delay(2);
    err = send_command(CMD_FLAG_RESET);
    if(err)
        return err;

    u32_t frametime = 1000 / (framerate == FPS10 ? 10 : 1);
    delay(frametime * 2);

    this->awake = true;
    return Error::SUCCESS;
}

Error AMG8833::is_sleeping() {
    return !awake;
}
Error AMG8833::set_framerate(AMG8833_framerate_t fr) {
    if (fr == framerate)
        return Error::SUCCESS;
    return fr == FPS10 ?
        send_command(CMD_SET_FRAMERATE_10) :
        send_command(CMD_SET_FRAMERATE_1);
}
AMG8833_framerate_t AMG8833::get_framerate() {
    return framerate;
}
