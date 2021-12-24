#ifndef SPI_HPP
#define SPI_HPP

#include "defines.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

class ICM20689 {
public:
  ICM20689();
  virtual ~ICM20689();

  void init();
  void write1byte(const uint8_t address, const uint8_t data);
  uint8_t read1byte(const uint8_t address);
  void setup();
  int read_gyro_z();

private:
  spi_device_handle_t spi;
};

#endif