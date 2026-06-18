#ifndef _BLE_SERIAL_H_
#define _BLE_SERIAL_H_

void ble_serial_init(const char *uart_name, int timeout);
int  ble_serial_getc_timeout(int timeout_ms);
int  ble_serial_getc_nowait(void);
int  ble_serial_putc(char c);

#endif

