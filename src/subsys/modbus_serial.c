/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(modbusc, LOG_LEVEL_INF);

static int client_iface;

static struct modbus_iface_param client_param = {
	.mode = MODBUS_MODE_RTU,
	.rx_timeout = 100000,//us
    .rtu_timeout = 15000,//us
	.serial = {
		.baud = 9600,
		.parity = UART_CFG_PARITY_NONE,
        // .stop_bits = UART_CFG_STOP_BITS_1
	}
};

int modbus_serial_init_modbus_client(uint32_t baud)
{
	const char *iface_name = {DT_PROP(DT_INST(0, zephyr_modbus_serial), label)};
	// const char *iface_name = {DT_LABEL(DT_NODELABEL(uart2))};

	client_iface = modbus_iface_get_by_name(iface_name);

    if (baud)
    {
       client_param.serial.baud = baud; 
    }

	return modbus_init_client(client_iface, client_param);
}

int modbus_serial_disable_modbus_client(void)
{
	return modbus_disable(client_iface);
}

int modbus_serial_read_hregs(const uint8_t unit_id,
			     const uint16_t start_addr,
			     uint16_t *const reg_buf,
			     const uint16_t num_regs)
{
    int err = modbus_read_holding_regs(client_iface, unit_id, start_addr, reg_buf, num_regs);
	if (err != 0) {
		LOG_ERR("FC03 failed with %d", err);
	}

    return err;
}

int modbus_serial_read_iregs(const uint8_t unit_id,
			     const uint16_t start_addr,
			     uint16_t *const reg_buf,
			     const uint16_t num_regs)
{
    int err = modbus_read_input_regs(client_iface, unit_id, start_addr, reg_buf, num_regs);
	if (err != 0) {
		LOG_ERR("FC04 failed with %d", err);
	}

    return err;
}
