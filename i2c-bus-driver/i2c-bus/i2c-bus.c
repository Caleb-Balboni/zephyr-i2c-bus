#define DT_DRV_COMPAT hn_i2c_bus 

#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <custom-dev.h>
#include <spi2c-com.h>

////////////////////////////////////////////
// SHARED PACKET STRUCTS AND DEFINES BETWEEN MASTER AND SLAVE I2C BUS DRIVERS

// for i2c_bus_msg_hdr flags
#define I2C_BUS_FLAG_READ 0
#define I2C_BUS_FLAG_WRITE 1

// for i2c_bus_packet_out status
#define I2C_BUS_STATUS_OK 0
#define I2C_BUS_STATUS_ERROR 1

struct i2c_bus_msg_hdr {
  uint8_t flags;
  uint8_t len;
  // the actual buffer of data comes after this (use length field to identify)
}__attribute__((packed));

// the packet structure from master -> slave
struct i2c_bus_packet_in {
  uint8_t num_msgs;
  uint8_t addr;
  struct i2c_bus_msg_hdr msgs[];
}__attribute__((packed));

struct i2c_bus_packet_out {
  uint8_t status;     // status of the execution of each command
  uint8_t data[];     // each read and write command
}__attribute__((packed));

////////////////////////////////////////////

#define MAX_TX_SIZE (MAX_TRANSACTION_SIZE - sizeof(struct spi2c_response))
#define MAX_RX_SIZE (MAX_TRANSACTION_SIZE - sizeof(struct spi2c_request))

struct i2c_bus_cfg {
	const struct device* i2c_controller;    // the actual peripheral (eg: flexcomm) attached to the physical i2c devices
};

struct i2c_bus_data {
	const struct device* spi2c_dev;         // reference the the slaves spi driver
	uint8_t reg_idx;                        // the register index of the driver itself (not associated with the address of i2c devices)
	uint8_t rx_buf[MAX_RX_SIZE];            // incoming packet from master (i2c command list)
	uint8_t tx_buf[MAX_TX_SIZE];            // outgoing packet to master (status + read results)
};

static void i2c_bus_receive(const struct device *dev, uint16_t data_size, uint16_t offset, spi2c_reg_notify_t notify) {
	struct i2c_bus_data* data = dev->data;
	const struct i2c_bus_cfg* cfg = dev->config;
  struct i2c_bus_packet_in* in = (struct i2c_bus_packet_in*)data->rx_buf;
  struct i2c_bus_packet_out* out = (struct i2c_bus_packet_out*)data->tx_buf;
  uint16_t rx_read_offset = 0;  // the current offset into the rx data
  uint8_t* cursor = (uint8_t*)in->msgs;
  struct i2c_msg msgs[in->num_msgs];
  for (int i = 0; i < in->num_msgs; i++) {
    struct i2c_bus_msg_hdr* cur_in_msg = (struct i2c_bus_msg_hdr*)cursor;
    cursor += sizeof(struct i2c_bus_msg_hdr);
    msgs[i].len = cur_in_msg->len;
    if (cur_in_msg->flags == I2C_BUS_FLAG_READ) {
      msgs[i].flags = I2C_MSG_READ;
      msgs[i].buf = &out->data[rx_read_offset];
      rx_read_offset += cur_in_msg->len;
    } else{
      msgs[i].flags = I2C_MSG_WRITE;
      msgs[i].buf = cursor;
      cursor += cur_in_msg->len;
    }
    if (i > 0) {
      msgs[i].flags |= I2C_MSG_RESTART;
    }
  }
  msgs[in->num_msgs - 1].flags |= I2C_MSG_STOP;
  int ret = i2c_transfer(cfg->i2c_controller, msgs, in->num_msgs, in->addr);
  if (ret) {
    out->status = I2C_BUS_STATUS_ERROR;
  } else {
    out->status = I2C_BUS_STATUS_OK;
  }
	notify(data->spi2c_dev, data->reg_idx, DEVICE_UPDATE);
}

static void i2c_bus_init_reg(const struct device* dev, struct spi2c_reg* reg, const struct device* spi2c_dev) {
  struct i2c_bus_data* data = (struct i2c_bus_data*)dev->data;
  data->spi2c_dev = spi2c_dev;
  data->reg_idx = reg->idx;
  reg->flags = REG_READABLE | REG_WRITEABLE;
  reg->rx_size = sizeof(data->rx_buf);
  reg->rx_buffer = data->rx_buf;
  reg->tx_size = sizeof(data->tx_buf);
  reg->tx_buffer = data->tx_buf;
}

static const struct custom_dev_api i2c_bus_driver_api = {
	.init_reg = i2c_bus_init_reg,
	.receive = i2c_bus_receive,
};

#define I2CBUS_DEFINE(inst)                                                     \
    static struct i2c_bus_data i2c_bus_data_##inst;                             \
    static const struct i2c_bus_cfg i2c_bus_cfg_##inst = {                      \
        .i2c_controller = DEVICE_DT_GET(DT_INST_PHANDLE(inst, i2c_controller)), \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(inst,                                                 \
        NULL, NULL,                                                             \
        &i2c_bus_data_##inst,                                                   \
        &i2c_bus_cfg_##inst,                                                    \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                        \
        &i2c_bus_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2CBUS_DEFINE)
