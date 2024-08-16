#ifndef AMG8883_h
#define AMG8883_h

#include <Wire.h>

extern const char *AMG8833_error_messages[];

class AMG8833_error_t {
    public:
        enum ErrorCode {
            SUCCESS = 0,
            DATA_TOO_LONG = 1,
            ADDRESS_NACK = 2,
            DATA_NACK = 3,
            OTHER_ERROR = 4,
            TIMEOUT = 5,
            SLEEPING = 6,
        };

        AMG8833_error_t() : value(SUCCESS) {  }
        AMG8833_error_t(int code) {
            this->value = static_cast<ErrorCode>(code);
        }

        constexpr operator ErrorCode() const { return this->value; }
        // err == true -> you have an error
        // err == false -> err == SUCCESS
        constexpr operator bool() const { return this->value != SUCCESS; }

        constexpr char const *str() const { return AMG8833_error_messages[this->value]; }
        constexpr ErrorCode code() const { return this->value; }
        AMG8833_error_t operator=(ErrorCode code) { value = code; return *this; }
        bool operator==(ErrorCode code) { return code == this->value; }

    private:
        ErrorCode value;
};

enum AMG8833_framerate_t {
    FPS1 = 1,
    FPS10 = 0
};

class AMG8833 {
    private:
        TwoWire *wire;
        int sda, scl;
        bool awake;
        AMG8833_framerate_t framerate;
        int i2c_addr;

        AMG8833_error_t send_command(const byte cmd[2]);
        AMG8833_error_t read_register(const byte reg, byte buf[], size_t num_bytes);

    public:

        AMG8833(TwoWire *wire, int sda, int scl, int i2c_addr) {
            this->wire = wire;
            this->sda = sda;
            this->scl = scl;
            this->i2c_addr = i2c_addr;
            awake = true;
            framerate = FPS10;
        }

        AMG8833_error_t read_pixels(float buf[64]);
        AMG8833_error_t read_pixels_raw(short buf[64]);
        AMG8833_error_t begin();
        AMG8833_error_t sleep();
        AMG8833_error_t wake();
        AMG8833_error_t is_sleeping();
        AMG8833_error_t set_framerate(AMG8833_framerate_t fr);
        AMG8833_framerate_t get_framerate();
};

#endif // AMG8883_h
