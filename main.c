#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/task.h>

// constants
#define HTU21D_ADDR			0x40
#define TRIGGER_TEMP_MEASURE_NOHOLD  	0xE3
#define _port I2C_NUM_0


#define I2C_EXAMPLE_TAG "HTU21D"
#define I2C_EXAMPLE_MASTER_SCL_IO 22
#define I2C_EXAMPLE_MASTER_SDA_IO 21
#define I2C_EXAMPLE_SLAVE_ADDR 0x40

static esp_err_t i2c_master_init() {
	int i2c_master_port = 0;
	i2c_config_t conf = {
	    .mode = I2C_MODE_MASTER,
	    .sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO,         // select SDA GPIO specific to your project
	    .sda_pullup_en = GPIO_PULLUP_ENABLE,
	    .scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO,         // select SCL GPIO specific to your project
	    .scl_pullup_en = GPIO_PULLUP_ENABLE,
	    .master.clk_speed = 100000,  // select frequency specific to your project
	    .clk_flags = 0,                          // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
	};
  int err = i2c_param_config(I2C_NUM_0, &conf);

  if (err == ESP_FAIL) {
      ESP_LOGE(I2C_EXAMPLE_TAG, "I2C config failed");
      return err;
  }
  err = i2c_driver_install(I2C_NUM_0, conf.mode, 0,0, 0);
  if (err == ESP_FAIL) {
      ESP_LOGE(I2C_EXAMPLE_TAG, "Failed to install driver");
      return err;
  }
  printf("Installed Successfully!");
  return ESP_OK;
}
static esp_err_t read_command(){
// send the command
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
i2c_master_write_byte(cmd, TRIGGER_TEMP_MEASURE_NOHOLD, true);
i2c_master_stop(cmd);
int ret = i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_PERIOD_MS);
i2c_cmd_link_delete(cmd);

// wait for the sensor (50ms)
vTaskDelay(50 / portTICK_PERIOD_MS);

// receive the answer
uint8_t msb, lsb, crc;
cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_READ, true);
i2c_master_read_byte(cmd, &msb, 0x00);
i2c_master_read_byte(cmd, &lsb, 0x00);
i2c_master_read_byte(cmd, &crc, 0x01);
i2c_master_stop(cmd);
ret = i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_PERIOD_MS);
if(ret == ESP_OK){
	ESP_LOGI(I2C_EXAMPLE_TAG,"Successful!");
	uint16_t raw_value = ((uint16_t) msb << 8) | (uint16_t) lsb;
	float temperature = (raw_value * 175.72 / 65536.0) - 46.85;
	float humidity = (raw_value * 125.0 / 65536.0) - 6.0;
	ESP_LOGI(I2C_EXAMPLE_TAG,"Humdity %f and temp %f ", humidity , temperature);
}else{
	ESP_LOGI(I2C_EXAMPLE_TAG,"ERROR!");
}
i2c_cmd_link_delete(cmd);
return ESP_OK;
}

void app_main(void)
{

	i2c_master_init();
	ESP_LOGI(I2C_EXAMPLE_TAG,"Installed Successfully!");
    while (true) {
        read_command();
        sleep(1);
    }
}
