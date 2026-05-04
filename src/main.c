// SPDX-License-Identifier: MIT

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <tusb.h>

#include "ws2812.pio.h"

// Defined in usb-descriptors.c
void usbd_serial_init(void);

// Hardware Configuration for VCC-GND Board
#define WS2812_PIN 23
#define ONBOARD_LED_PIN 25

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define BUFFER_SIZE 4096

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
	uart_inst_t *const inst;
	uint irq;
	void *irq_fn;
	uint8_t tx_pin;
	uint8_t rx_pin;
	uint8_t dtr_pin;
	uint8_t rts_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	queue_t uart_to_usb_fifo;
	queue_t usb_to_uart_fifo;
    volatile bool baud_updated;
} uart_data_t;

// Color packing helper
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t) (r) << 16) | ((uint32_t) (g) << 8) | (uint32_t) (b);
}

static inline void put_pixel(uint32_t pixel_rgb) {
    uint8_t r = (pixel_rgb >> 16) & 0xFF;
    uint8_t g = (pixel_rgb >> 8) & 0xFF;
    uint8_t b = pixel_rgb & 0xFF;
    uint32_t grb = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
    pio_sm_put_blocking(pio0, 0, grb);
}

void uart0_irq_fn(void);

const uart_id_t UART_ID = {
	.inst = uart0,
	.irq = UART0_IRQ,
	.irq_fn = &uart0_irq_fn,
	.tx_pin = 0,
	.rx_pin = 1,
	.dtr_pin = 2,
	.rts_pin = 3,
};

uart_data_t UART_DATA;

uint32_t last_connection = 0;
uint32_t rx_activity_timer = 0;
uint32_t tx_activity_timer = 0;

static inline uint databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5: return 5;
		case 6: return 6;
		case 7: return 7;
		default: return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1: return UART_PARITY_ODD;
		case 2: return UART_PARITY_EVEN;
		default: return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2: return 2;
		default: return 1;
	}
}

void update_uart_cfg(void)
{
	const uart_id_t *ui = &UART_ID;
	uart_data_t *ud = &UART_DATA;

	mutex_enter_blocking(&ud->lc_mtx);
    cdc_line_coding_t usb_lc = ud->usb_lc;
    mutex_exit(&ud->lc_mtx);

    // Apply baudrate
    if (usb_lc.bit_rate != ud->uart_lc.bit_rate) {
        uart_set_baudrate(ui->inst, usb_lc.bit_rate);
        ud->uart_lc.bit_rate = usb_lc.bit_rate;
    }

    // Apply format
    uart_set_format(ui->inst,
            databits_usb2uart(usb_lc.data_bits),
            stopbits_usb2uart(usb_lc.stop_bits),
            parity_usb2uart(usb_lc.parity));
    
    ud->uart_lc.data_bits = usb_lc.data_bits;
    ud->uart_lc.parity = usb_lc.parity;
    ud->uart_lc.stop_bits = usb_lc.stop_bits;
}

void usb_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA;
	uint8_t buffer[64];
	
    uint32_t avail = tud_cdc_n_available(itf);
    if (avail) {
		uint32_t len = MIN(avail, sizeof(buffer));
		uint32_t count = tud_cdc_n_read(itf, buffer, len);
		for (uint32_t i = 0; i < count; i++) {
			if (!queue_try_add(&ud->usb_to_uart_fifo, &buffer[i])) break;
		}
	}
}

void usb_write_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA;
	uint8_t ch;

	while (tud_cdc_n_write_available(itf) && queue_try_remove(&ud->uart_to_usb_fifo, &ch)) {
		tud_cdc_n_write(itf, &ch, 1);
	}
	tud_cdc_n_write_flush(itf);
}

// Host changes line coding
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding)
{
    uart_data_t *ud = &UART_DATA;
    mutex_enter_blocking(&ud->lc_mtx);
    ud->usb_lc = *p_line_coding;
    ud->baud_updated = true;
    mutex_exit(&ud->lc_mtx);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	const uart_id_t *ui = &UART_ID;
    gpio_put(ui->rts_pin, rts);
    gpio_put(ui->dtr_pin, dtr);
}

void usb_cdc_process(uint8_t itf)
{
	usb_read_bytes(itf);
	usb_write_bytes(itf);
}

void update_leds(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    static uint32_t last_led_update = 0;
    if (now - last_led_update < 20) return;
    last_led_update = now;

    if (!tud_ready()) {
        put_pixel(urgb_u32(16, 0, 0)); // Red - Disconnected
        gpio_put(ONBOARD_LED_PIN, 0);
    } else {
        gpio_put(ONBOARD_LED_PIN, 1); 
        
        if (now < rx_activity_timer) {
            put_pixel(urgb_u32(0, 0, 32)); // Blue - RX
        } else if (now < tx_activity_timer) {
            put_pixel(urgb_u32(32, 16, 0)); // Orange - TX
        } else {
            put_pixel(urgb_u32(0, 8, 0)); // Green - Idle
        }
    }
}

void core1_entry(void)
{
	tusb_init();
	while (1) {
		tud_task();
		if(tud_ready()) usb_cdc_process(0);
        update_leds();
	}
}

void uart_read_bytes(void)
{
	uart_data_t *ud = &UART_DATA;
	const uart_id_t *ui = &UART_ID;
	while (uart_is_readable(ui->inst)) {
		uint8_t ch = uart_getc(ui->inst);
		queue_try_add(&ud->uart_to_usb_fifo, &ch);
		rx_activity_timer = to_ms_since_boot(get_absolute_time()) + 50;
	}
}

void uart0_irq_fn(void) { uart_read_bytes(); }

void uart_write_bytes(void)
{
	uart_data_t *ud = &UART_DATA;
	const uart_id_t *ui = &UART_ID;
	uint8_t ch;
	while (uart_is_writable(ui->inst) && queue_try_remove(&ud->usb_to_uart_fifo, &ch)) {
		uart_putc_raw(ui->inst, ch);
		tx_activity_timer = to_ms_since_boot(get_absolute_time()) + 50;
	}
}

void init_uart_data(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID;
	uart_data_t *ud = &UART_DATA;

	gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
	gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);
	
	gpio_init(ui->rts_pin); gpio_set_dir(ui->rts_pin, GPIO_OUT); gpio_put(ui->rts_pin, 1);
	gpio_init(ui->dtr_pin); gpio_set_dir(ui->dtr_pin, GPIO_OUT); gpio_put(ui->dtr_pin, 1);
    gpio_init(ONBOARD_LED_PIN); gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

	ud->usb_lc.bit_rate = DEF_BIT_RATE; 
    ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY; 
    ud->usb_lc.stop_bits = DEF_STOP_BITS;
	ud->uart_lc = ud->usb_lc;
    ud->baud_updated = false;

	queue_init(&ud->uart_to_usb_fifo, 1, BUFFER_SIZE);
	queue_init(&ud->usb_to_uart_fifo, 1, BUFFER_SIZE);
	mutex_init(&ud->lc_mtx);

	uart_init(ui->inst, ud->usb_lc.bit_rate);
    uart_set_fifo_enabled(ui->inst, true);
	uart_set_hw_flow(ui->inst, false, false);
	uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
			stopbits_usb2uart(ud->usb_lc.stop_bits), parity_usb2uart(ud->usb_lc.parity));
	
	irq_set_exclusive_handler(ui->irq, ui->irq_fn);
	irq_set_enabled(ui->irq, true);
	uart_set_irq_enables(ui->inst, true, false);
}

int main(void)
{
    stdio_init_all();
    multicore_reset_core1();
	usbd_serial_init();
	init_uart_data(0);

    // WS2812 Init
    uint offset = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, 0, offset, WS2812_PIN, 800000, true);

	multicore_launch_core1(core1_entry);

	while (1) {
        if (UART_DATA.baud_updated) {
            update_uart_cfg();
            UART_DATA.baud_updated = false;
        }
		uart_write_bytes();
	}
	return 0;
}
