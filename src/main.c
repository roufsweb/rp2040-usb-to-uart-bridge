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

#define BUFFER_SIZE 2560

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
} uart_data_t;

// PIO helper

// Color packing helper (for WS2812 GRB/RGB order)
// This version packs as R, G, B in the bytes, and put_pixel shifts it.
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 16) |
            ((uint32_t) (g) << 8) |
            (uint32_t) (b);
}

static inline void put_pixel(uint32_t pixel_rgb) {
    // We send 24 bits. Most WS2812 programs expect G, R, B order.
    // Our packed format is 0x00RRGGBB.
    // To send G, R, B:
    uint8_t r = (pixel_rgb >> 16) & 0xFF;
    uint8_t g = (pixel_rgb >> 8) & 0xFF;
    uint8_t b = pixel_rgb & 0xFF;
    
    // Pack as GRB for the PIO
    uint32_t grb = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
    pio_sm_put_blocking(pio0, 0, grb);
}

void uart0_irq_fn(void);

// Using UART0 for best pins (0, 1, 2, 3)
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
		case 5:
			return 5;
		case 6:
			return 6;
		case 7:
			return 7;
		default:
			return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1:
			return UART_PARITY_ODD;
		case 2:
			return UART_PARITY_EVEN;
		default:
			return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

void update_uart_cfg(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID;
	uart_data_t *ud = &UART_DATA;

	mutex_enter_blocking(&ud->lc_mtx);

	if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
		uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
		ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	}

	if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
	    (ud->usb_lc.parity != ud->uart_lc.parity) ||
	    (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
		uart_set_format(ui->inst,
				databits_usb2uart(ud->usb_lc.data_bits),
				stopbits_usb2uart(ud->usb_lc.stop_bits),
				parity_usb2uart(ud->usb_lc.parity));
		ud->uart_lc.data_bits = ud->usb_lc.data_bits;
		ud->uart_lc.parity = ud->usb_lc.parity;
		ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
	}

	mutex_exit(&ud->lc_mtx);
}

void usb_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA;
	uint8_t buffer[64];
	uint32_t len = tud_cdc_n_available(itf);

	if (len) {
		len = MIN(len, sizeof(buffer));
		uint32_t count = tud_cdc_n_read(itf, buffer, len);
		for (uint32_t i = 0; i < count; i++) {
			queue_try_add(&ud->usb_to_uart_fifo, &buffer[i]);
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

void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms) {
        const uart_id_t *ui = &UART_ID;

        if(duration_ms == 0xffff) {
                uart_set_break(ui->inst, true);
        } else if(duration_ms == 0x0000) {
                uart_set_break(ui->inst, false);
        } else {
                uart_set_break(ui->inst, true);
                sleep_ms(duration_ms);
                uart_set_break(ui->inst, false);
        }
}

/* Invoked when line state has changed */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	const uart_id_t *ui = &UART_ID;

	if((!dtr && !rts) || (to_ms_since_boot(get_absolute_time())-1000 < last_connection)){
		gpio_put(ui->rts_pin, !rts);
		gpio_put(ui->dtr_pin, !dtr);
		last_connection = to_ms_since_boot(get_absolute_time());
	} else {
		gpio_put(ui->rts_pin, rts);
		gpio_put(ui->dtr_pin, dtr); 
	}
}

void usb_cdc_process(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA;

	mutex_enter_blocking(&ud->lc_mtx);
	tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
	
	mutex_exit(&ud->lc_mtx);

	usb_read_bytes(itf);
	usb_write_bytes(itf);
}

// LED update logic
void update_leds(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    static uint32_t last_led_update = 0;
    
    // Rate limit LED updates to 100Hz
    if (now - last_led_update < 10) return;
    last_led_update = now;

    if (!tud_ready()) {
        put_pixel(urgb_u32(16, 0, 0)); // Dim Red - Not Ready
        gpio_put(ONBOARD_LED_PIN, 0);
    } else {
        gpio_put(ONBOARD_LED_PIN, 1); // USB Connected
        
        if (now < rx_activity_timer) {
            put_pixel(urgb_u32(0, 0, 32)); // Dim Blue - RX Activity
        } else if (now < tx_activity_timer) {
            put_pixel(urgb_u32(32, 16, 0)); // Dim Orange - TX Activity
        } else {
            put_pixel(urgb_u32(0, 8, 0)); // Very Dim Green - Idle
        }
    }
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		tud_task();

		if(tud_ready()) {
			usb_cdc_process(0);
		}
        update_leds();
	}
}

static inline void uart_read_bytes(void)
{
	uart_data_t *ud = &UART_DATA;
	const uart_id_t *ui = &UART_ID;

	while (uart_is_readable(ui->inst)) {
		uint8_t ch = uart_getc(ui->inst);
		if (!queue_try_add(&ud->uart_to_usb_fifo, &ch)) {
			// FIFO full, data dropped
		}
		rx_activity_timer = to_ms_since_boot(get_absolute_time()) + 50;
	}
}

void uart0_irq_fn(void)
{
	uart_read_bytes();
}

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

	/* Pinmux */
	gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
	gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);
	
	gpio_init(ui->rts_pin);
	gpio_set_dir(ui->rts_pin, GPIO_OUT);
	gpio_put(ui->rts_pin, 1);

	gpio_init(ui->dtr_pin);
	gpio_set_dir(ui->dtr_pin, GPIO_OUT);
	gpio_put(ui->dtr_pin, 1);

    // Onboard LED
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

	/* USB CDC LC */
	ud->usb_lc.bit_rate = DEF_BIT_RATE;
	ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY;
	ud->usb_lc.stop_bits = DEF_STOP_BITS;

	/* UART LC */
	ud->uart_lc.bit_rate = DEF_BIT_RATE;
	ud->uart_lc.data_bits = DEF_DATA_BITS;
	ud->uart_lc.parity = DEF_PARITY;
	ud->uart_lc.stop_bits = DEF_STOP_BITS;

	/* Queues */
	queue_init(&ud->uart_to_usb_fifo, 1, BUFFER_SIZE);
	queue_init(&ud->usb_to_uart_fifo, 1, BUFFER_SIZE);

	/* Mutex */
	mutex_init(&ud->lc_mtx);

	/* UART start */
	uart_init(ui->inst, ud->usb_lc.bit_rate);
	uart_set_hw_flow(ui->inst, false, false);
	uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
			stopbits_usb2uart(ud->usb_lc.stop_bits),
			parity_usb2uart(ud->usb_lc.parity));
	uart_set_fifo_enabled(ui->inst, false);
	uart_set_translate_crlf(ui->inst, false);

	/* UART RX Interrupt */
	irq_set_exclusive_handler(ui->irq, ui->irq_fn);
	irq_set_enabled(ui->irq, true);
	uart_set_irq_enables(ui->inst, true, false);
}

int main(void)
{
    set_sys_clock_khz(125000, false);
    multicore_reset_core1();

	usbd_serial_init();

	init_uart_data(0);

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, true);
	put_pixel(urgb_u32(128,0,0));

	multicore_launch_core1(core1_entry);

	while (1) {
			update_uart_cfg(0);
			uart_write_bytes();
	}

	return 0;
}
