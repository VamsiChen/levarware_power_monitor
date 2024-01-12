#ifndef MODBUS_SERIAL_H_
#define MODBUS_SERIAL_H_

#include <zephyr.h>

int modbus_serial_init_modbus_client(uint32_t baud);
int modbus_serial_disable_modbus_client(void);
int modbus_serial_read_hregs(const uint8_t unit_id,
			     const uint16_t start_addr,
			     uint16_t *const reg_buf,
			     const uint16_t num_regs);
int modbus_serial_read_iregs(const uint8_t unit_id,
			     const uint16_t start_addr,
			     uint16_t *const reg_buf,
			     const uint16_t num_regs);

#endif // MODBUS_SERIAL_H_ 