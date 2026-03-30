#define DT_DRV_COMPAT tsys01_sensor

#include <math.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <custom-dev.h>
#include <spi2c-com.h>

static const uint8_t TMP_SENSOR_RESET = 0x1E;
static const uint8_t TMP_SENSOR_ADC_READ = 0x00;
static const uint8_t TMP_SENSOR_ADC_TEMP_CONVERT = 0x48;

static const uint8_t K0_VAR = 0xAA;
static const uint8_t K1_VAR = 0xA8;
static const uint8_t K2_VAR = 0xA6;
static const uint8_t K3_VAR = 0xA4;
static const uint8_t K4_VAR = 0xA2;

struct temp_sensor_config {
	struct i2c_dt_spec temp_sensor;
};

struct temp_sensor_data {
	const struct device *spi2c_dev;
	uint8_t reg_idx;
	uint32_t temperature;
};

static int init_device(const struct device *dev) {
	const struct temp_sensor_config *cfg = dev->config;
	return i2c_write_dt(&cfg->temp_sensor, &TMP_SENSOR_RESET, 1);
}

static uint16_t get_single_K_value(const struct temp_sensor_config *cfg, uint8_t cmd) {
	uint8_t buf[2];
	if (i2c_write_read_dt(&cfg->temp_sensor, &cmd, 1, buf, sizeof(buf))) {
		printk("tsys01: failed to read K value 0x%X\n", cmd);
	}
	return ((uint16_t)buf[0] << 8) | buf[1];
}

static void get_calibration_data(const struct temp_sensor_config *cfg, uint16_t *calibration_data) {
	k_msleep(10);
	calibration_data[0] = get_single_K_value(cfg, K0_VAR);
	calibration_data[1] = get_single_K_value(cfg, K1_VAR);
	calibration_data[2] = get_single_K_value(cfg, K2_VAR);
	calibration_data[3] = get_single_K_value(cfg, K3_VAR);
	calibration_data[4] = get_single_K_value(cfg, K4_VAR);
}

static uint32_t read_temp_result(const struct temp_sensor_config *cfg) {
	if (i2c_write_dt(&cfg->temp_sensor, &TMP_SENSOR_ADC_TEMP_CONVERT, 1)) {
		printk("tsys01: failed to start temp conversion\n");
		return 0;
	}
	k_msleep(10);
	uint8_t buf[3];
	if (i2c_write_read_dt(&cfg->temp_sensor, &TMP_SENSOR_ADC_READ, 1, buf, sizeof(buf))) {
		printk("tsys01: failed to read ADC result\n");
		return 0;
	}
	return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

static double calculate_temperature(uint16_t *calibration_data, uint32_t adc_val) {
	double temp_val = adc_val / 256.0;
	return (-2.0) * calibration_data[4] * 1e-21 * pow(temp_val, 4) +
	       (4.0) * calibration_data[3] * 1e-16 * pow(temp_val, 3) +
	       (-2.0) * calibration_data[2] * 1e-11 * pow(temp_val, 2) +
	       (1.0) * calibration_data[1] * 1e-6 * temp_val +
	       (-1.5) * calibration_data[0] * 1e-2;
}

// custom_dev_api: bind
static void temp_sensor_bind(const struct device *dev, const struct device *spi2c_dev, uint8_t reg_idx) {
	struct temp_sensor_data *data = dev->data;
	data->spi2c_dev = spi2c_dev;
	data->reg_idx = reg_idx;
}

// custom_dev_api: receive (called by spi2c write thread when master writes to this register)
static void temp_sensor_receive(const struct device *dev, uint8_t *in_data, size_t data_size, spi2c_reg_notify_t notify) {
	struct temp_sensor_data *data = dev->data;
	const struct temp_sensor_config *cfg = dev->config;

	uint16_t calibration_data[5];
	get_calibration_data(cfg, calibration_data);
	uint32_t adc_val = read_temp_result(cfg);
	if (adc_val == 0) {
		notify(data->spi2c_dev, data->reg_idx, DEVICE_ERROR);
		return;
	}
	data->temperature = (uint32_t)calculate_temperature(calibration_data, adc_val);
	notify(data->spi2c_dev, data->reg_idx, DEVICE_UPDATE);
}

// custom_dev_api: get_data
static uint8_t *temp_sensor_get_data(const struct device *dev) {
	struct temp_sensor_data *data = dev->data;
	return (uint8_t *)&data->temperature;
}

// custom_dev_api: get_data_size
static size_t temp_sensor_get_data_size(const struct device *dev) {
	return sizeof(uint32_t);
}

// custom_dev_api: get_flags
static uint8_t temp_sensor_get_flags(const struct device *dev) {
	return REG_READABLE | REG_WRITEABLE;
}

static const struct custom_dev_api temp_sensor_driver_api = {
	.bind = temp_sensor_bind,
	.receive = temp_sensor_receive,
	.get_data = temp_sensor_get_data,
	.get_data_size = temp_sensor_get_data_size,
	.get_flags = temp_sensor_get_flags,
};

#define TMPSENSOR_DEFINE(inst)                                              \
    static struct temp_sensor_data temp_sensor_data_##inst;                 \
    static const struct temp_sensor_config temp_sensor_config_##inst = {    \
        .temp_sensor = I2C_DT_SPEC_INST_GET(inst),                         \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                             \
        init_device, NULL,                                                  \
        &temp_sensor_data_##inst,                                           \
        &temp_sensor_config_##inst,                                         \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
        &temp_sensor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TMPSENSOR_DEFINE)
