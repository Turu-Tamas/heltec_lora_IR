#include <Arduino.h>
#include <heltec_unofficial.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

const int I2C_SDA = GPIO_NUM_41;
const int I2C_SCL = GPIO_NUM_42;
uint8_t I2C_ADDRESS;
Adafruit_AMG88xx amg;

uint32_t last_loop = 0;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

void hang();
void find_IR_I2C_addr();

void setup() {
  heltec_setup();
  delay(4000);
  Serial.println(F("AMG88xx test"));

  bool status;

  Wire.begin(I2C_SDA, I2C_SCL, 400*1000);

  find_IR_I2C_addr();

  status = amg.begin(I2C_ADDRESS);
  if (!status) {
      both.println("No valid sensor");
      hang();
  }
  amg.disableInterrupt();
  amg.setMovingAverageMode(false);
  delay(100); // let sensor boot up
}

void loop() {
  heltec_loop();

  if (millis() - last_loop < 1000)
    return;
  last_loop = millis(); 

  Serial.print("Thermistor Temperature = ");
  Serial.print(amg.readThermistor());
  Serial.println(" ˚C");

  memset(pixels, 69, AMG88xx_PIXEL_ARRAY_SIZE);
  amg.readPixels(pixels);

  for(int i=1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++){
    Serial.print(pixels[i-1]);
    Serial.print(", ");
    if( i%8 == 0 ) Serial.println();
  }

  Serial.println();
}

void find_IR_I2C_addr() {
  byte mask = 0b1111000;
  for (I2C_ADDRESS = 0x00; I2C_ADDRESS < 0xff; I2C_ADDRESS++) {
    // if ((I2C_ADDRESS & mask) == 0 || (I2C_ADDRESS & mask) == mask)
    //   continue;

    Serial.printf("Checking addr: %2x\n", I2C_ADDRESS);
    Wire.beginTransmission(I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
      both.printf("i2c addr: %x\n", I2C_ADDRESS);
      // return;
    }
  }
  both.println("I2C NACK");
  hang();
}

void hang() { 
  both.println("Hanged");
  while (true) { heltec_loop(); }
}
