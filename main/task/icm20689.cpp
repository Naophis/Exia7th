#include "include/icm20689.hpp"

ICM20689::ICM20689() {}
ICM20689::~ICM20689() {}
void ICM20689::init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = EN_MOSI,
      .miso_io_num = EN_MISO,
      .sclk_io_num = EN_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 2, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 7 * 1000 * 1000, // aaaaaaaaaaa
      .spics_io_num = EN_GN_SSL,
      .queue_size = 7,
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
}
void ICM20689::write1byte(const uint8_t address, const uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_RXDATA;
  uint16_t tx_data = (address) << 8 | (0x0f & data);
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
}

uint8_t ICM20689::read1byte(const uint8_t address) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_RXDATA;
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data
  return data;
}
void ICM20689::setup() {
  // uint8_t whoami = mpu9250_read1byte(spi, 0x75);
  write1byte(0x6B, 0x80); //スリープ解除?
  vTaskDelay(250 / portTICK_PERIOD_MS);
  write1byte(0x68, 0x04); //ジャイロリセット
  vTaskDelay(250 / portTICK_PERIOD_MS);
  write1byte(0x6A, 0x10); // uercontrol i2c=disable
  vTaskDelay(250 / portTICK_PERIOD_MS);
  write1byte(0x1B, 0x18); // 2000
  vTaskDelay(250 / portTICK_PERIOD_MS);
}
int ICM20689::read_gyro_z() {
  return (signed short)(read1byte(0x47) << 8 | read1byte(0x48));
}