#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>

class Imu {
  private:
    double theta_;
    UARTClass *uart_;

  public:
    Imu(UARTClass& uart) {
      uart_ = &uart;
      theta_ = 0.0;
    }
    double readAngle() {
      int state = 0;
      int counter = 0;
      int complete = 0;
      byte buffer[13] = {};

      for (int i = 0; i < 30; i++) {
        if (uart_->available() > 0) {
          byte data = uart_->read();
          switch (state) {
            case 0:
              if (data == 0xaa) {
                state++;
              }
              break;
            case 1:
              if (data == 0x00) {
                state++;
              } else {
                counter = 0;
                state = 0;
              }
              break;
            case 2:
              buffer[counter++] = data;
              if (counter >= 13) {
                int sum = 0;
                for (int j = 0; j < 11; j++) sum += buffer[j];
                if ((sum & 0xff) == buffer[12]) {
                  theta_ = (buffer[2] << 8) | buffer[1];

                  if ((theta_ / 100) >= 475) {
                    theta_ = (((theta_ / 10) - (4750 + 1800)) * -1) / 10;
                  } else {
                    theta_ = ((theta_ / 10) * -1) / 10;
                  }
                  //Serial.println(theta);
                  uart_->flush();
                  complete = 1;
                }
              }
              break;
          }
        }
        if (complete) break;
      }
      return theta_;
    }
};

#endif
