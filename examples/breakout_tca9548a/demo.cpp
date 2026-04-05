#include "pico/stdlib.h"
#include "common/pimoroni_common.hpp"

#include "tca9548a.hpp"

using namespace pimoroni;

  I2C i2c(BOARD::SERVO_2040);
  TCA9548A tca9548a(&i2c);

  int main() {
    stdio_init_all();

    printf("TCA9548A Demo\n");
    

    //printf("Connect_0: %s \n",as5600.isConnected()? "true" : "false");
    tca9584a.tcaselect(1);


    //printf("Gain: C1: %fx C2: %fx C3: %fx\n", reading.gain(1), reading.gain(2), reading.gain(3));

    sleep_ms(1000);
  

    return 0;
  }