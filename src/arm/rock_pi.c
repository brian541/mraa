/*
 * Author: Thomas Ingleby <thomas.c.ingleby@intel.com>
 * Author: Michael Ring <mail@michael-ring.org>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <mraa/common.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/rock_pi.h"
#include "common.h"

#define DT_BASE "/proc/device-tree"
#define PLATFORM_NAME_ROCK_PI_4B "ROCK PI 4B"


#define MAX_SIZE 64
#define MMAP_PATH "/dev/mem"

// Rock Pi 4B
int rockpi4b_ls_gpio_pins[MRAA_ROCK_PI_4B_GPIO_COUNT] = { 1006, 1002, 1041, 1042, 1121, 1128,
                                                          1124, 1131, 1125, 1132, 1050, 1055 };

const char* rockpi4b_serialdev[MRAA_ROCK_PI_4B_UART_COUNT] = { "/dev/ttyS2" };

void
mraa_rockpi4b_pininfo(mraa_board_t* board, int index, int sysfs_pin, int is_gpio, char* fmt, ...)
{
    va_list arg_ptr;
    if (index > board->phy_pin_count)
        return;

    mraa_pininfo_t* pininfo = &board->pins[index];
    va_start(arg_ptr, fmt);
    vsnprintf(pininfo->name, MRAA_PIN_NAME_SIZE, fmt, arg_ptr);
    if (is_gpio) {
        // skip the read argument
        va_arg(arg_ptr, int);
        pininfo->gpio.gpio_chip = va_arg(arg_ptr, int);
        pininfo->gpio.gpio_line = va_arg(arg_ptr, int);
        pininfo->capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    } else {
        pininfo->capabilities = (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 };
    }
    va_end(arg_ptr);
    pininfo->gpio.pinmap = sysfs_pin;
    pininfo->gpio.mux_total = 0;
}

mraa_board_t*
mraa_rock_pi()
{
    int i;
    int* ls_gpio_pins = NULL;
    int(*chardev_map)[MRAA_ROCK_PI_4B_GPIO_COUNT][2] = NULL;

    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    // pin mux for buses are setup by default by kernel so tell mraa to ignore them
    b->no_bus_mux = 1;
    b->phy_pin_count = MRAA_ROCK_PI_4B_PIN_COUNT + 1;

    if (mraa_file_exist(DT_BASE "/model")) {
        // We are on a modern kernel, great!!!!
        if (mraa_file_contains(DT_BASE "/model", "ROCK PI 4B")) {
            b->platform_name = PLATFORM_NAME_ROCK_PI_4B;
            ls_gpio_pins = rockpi4b_ls_gpio_pins;
            b->uart_dev[0].device_path = (char*) rockpi4b_serialdev[0];
        } 
    }

    // UART
    b->uart_dev_count = MRAA_ROCK_PI_4B_UART_COUNT;
    b->def_uart_dev = 0;

    // I2C
    if (strncmp(b->platform_name, PLATFORM_NAME_ROCK_PI_4B, MAX_SIZE) == 0) {
        b->i2c_bus_count = MRAA_ROCK_PI_4B_I2C_COUNT;
        b->def_i2c_bus = 0;
        b->i2c_bus[0].bus_id = 7;
    } 

    // SPI
    b->spi_bus_count = MRAA_ROCK_PI_4B_SPI_COUNT;
    b->spi_bus[0].bus_id = 1;
    b->def_spi_bus = 0;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }

    mraa_rockpi4b_pininfo(b, 0, -1, 0, "INVALID");
    mraa_rockpi4b_pininfo(b, 1, -1, 0, "3V3");
    mraa_rockpi4b_pininfo(b, 2, -1, 0, "5V");
    mraa_rockpi4b_pininfo(b, 3, -1, 0, "SDA0");
    mraa_rockpi4b_pininfo(b, 4, -1, 0, "5V");
    mraa_rockpi4b_pininfo(b, 5, -1, 0, "SCL0");
    mraa_rockpi4b_pininfo(b, 6, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 7, 75, 1, "GPIO2_B3");
    mraa_rockpi4b_pininfo(b, 8, -1, 0, "UART_TX");
    mraa_rockpi4b_pininfo(b, 9, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 10, -1, 0, "UART_RX");
    mraa_rockpi4b_pininfo(b, 11, 146, 1, "GPIO4_C2");
    mraa_rockpi4b_pininfo(b, 12, 131, 1, "GPIO4_A3");
    mraa_rockpi4b_pininfo(b, 13, 150, 1, "GPIO4_C6");
    mraa_rockpi4b_pininfo(b, 14, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 15, 149, 1, "GPIO4_C5");
    mraa_rockpi4b_pininfo(b, 16, 154, 1, "GPIO4_D2");
    mraa_rockpi4b_pininfo(b, 17, -1, 0, "3V3");
    mraa_rockpi4b_pininfo(b, 18, 156, 1, "GPIO4_D4");
    mraa_rockpi4b_pininfo(b, 19, -1, 0, "SPI_MOSI");
    mraa_rockpi4b_pininfo(b, 20, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 21, -1, 0, "SPI_MISO");
    mraa_rockpi4b_pininfo(b, 22, 157, 1, "GPIO4_D5");
    mraa_rockpi4b_pininfo(b, 23, -1, 0, "SPI_CLK");
    mraa_rockpi4b_pininfo(b, 24, -1, 0, "SPI_CS0");
    mraa_rockpi4b_pininfo(b, 25, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 26, -1, 0, "ADC_IN0");
    mraa_rockpi4b_pininfo(b, 27, 64, 1, "GPIO2_A0");
    mraa_rockpi4b_pininfo(b, 28, 64, 1, "GPIO2_A1");
    mraa_rockpi4b_pininfo(b, 29, 74, 1, "GPIO2_B2");
    mraa_rockpi4b_pininfo(b, 30, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 31, 73, 1, "GPIO2_B1");
    mraa_rockpi4b_pininfo(b, 32, 112, 1, "GPIO3_C0");
    mraa_rockpi4b_pininfo(b, 33, 76, 1, "GPIO2_B4");
    mraa_rockpi4b_pininfo(b, 34, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 35, 133, 1, "GPIO4_A5");
    mraa_rockpi4b_pininfo(b, 36, 132, 1, "GPIO4_A4");
    mraa_rockpi4b_pininfo(b, 37, 158, 1, "GPIO4_D6");
    mraa_rockpi4b_pininfo(b, 38, 134, 1, "GPIO4_A6");
    mraa_rockpi4b_pininfo(b, 39, -1, 0, "GND");
    mraa_rockpi4b_pininfo(b, 40, 135, 1, "GPIO4_A7");

    b->aio_count = 0;
    b->adc_raw = 0;
    b->adc_supported = 0;

    return b;
}
