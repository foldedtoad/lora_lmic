/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "lmic.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(hal);

static struct device * dev_spi;
static struct spi_config spi_cfg;
static struct device * dev_gpio;
static struct spi_cs_control spi_cs;

static struct gpio_callback dio0_event;

#define HAL_GPIO_NAME    DT_INST_0_IBM_SX1276_RESET_GPIOS_CONTROLLER

#define HAL_RESET_PIN    DT_INST_0_IBM_SX1276_RESET_GPIOS_PIN
#define HAL_RESET_FLAG   DT_INST_0_IBM_SX1276_RESET_GPIOS_FLAGS

#define HAL_DIO0_PIN     DT_INST_0_IBM_SX1276_DIO0_GPIOS_PIN
#define HAL_DIO0_FLAG    DT_INST_0_IBM_SX1276_DIO0_GPIOS_FLAGS

#define HAL_CS_PIN       DT_NORDIC_NRF_SPI_SPI_1_CS_GPIOS_PIN
#define HAL_CS_FLAG      DT_NORDIC_NRF_SPI_SPI_1_CS_GPIOS_FLAGS


// -----------------------------------------------------------------------------
// I/O

// HAL state
struct {
    int irqlevel;
    u32_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// I/O

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u8_t val) {
}

// set radio NSS pin to given value
void hal_pin_nss (u8_t val) {
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u8_t val)
{
    switch (val) {
        case 0:
            gpio_pin_configure(dev_gpio, HAL_RESET_PIN, GPIO_OUTPUT | HAL_RESET_FLAG);
            gpio_pin_set(dev_gpio, HAL_RESET_PIN, 0);  // enter reset
            break;
        case 1:
            gpio_pin_configure(dev_gpio, HAL_RESET_PIN, GPIO_OUTPUT | HAL_RESET_FLAG);
            gpio_pin_set(dev_gpio, HAL_RESET_PIN, 1);  // exit reset
            break;
        default:
            gpio_pin_configure(dev_gpio, HAL_RESET_PIN, GPIO_INPUT);  // HI-Z 
            break;
    }
}

// generic EXTI IRQ handler for all channels
void EXTI_IRQHandler () {
}

#if 0 // CFG_lmic_clib
void EXTI0_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI1_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI2_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI3_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI4_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler () {
    EXTI_IRQHandler();
}
#endif // CFG_lmic_clib

// -----------------------------------------------------------------------------
// SPI

// perform SPI transaction with radio
u8_t hal_spi(u8_t reg, u8_t * data, int len) 
{
    struct spi_buf buf[2] = {
        {
            .buf = &reg,
            .len = sizeof(reg)
        },
        {
            .buf = data,
            .len = len
        }
    };

    struct spi_buf_set tx = {
        .buffers = buf,
        .count = ARRAY_SIZE(buf)
    };

    if ((reg & 0x80) == 0x00) {
        const struct spi_buf_set rx = {
            .buffers = buf,
            .count = ARRAY_SIZE(buf)
        };

        tx.count = 1;

        return spi_transceive(dev_spi, &spi_cfg, &tx, &rx);
    }

    tx.count = 2;
    //buf[1].buf = 0;

    return spi_write(dev_spi, &spi_cfg, &tx);
}

#ifdef CFG_lmic_clib

// -----------------------------------------------------------------------------
// TIME

u32_t hal_ticks () {
    return 0;
}

void hal_waitUntil (u32_t time) {
}

// check and rewind for target time
u8_t hal_checkTimer (u32_t time) {
    return 0;
}
  
void TIM9_IRQHandler () {
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
}

void hal_enableIRQs () {
}

void hal_sleep () {
}


// -----------------------------------------------------------------------------

/* DIO0 event callback */
static void hal_dio0_event(struct device * dev, 
                          struct gpio_callback * cb, 
                          u32_t pins)
{
    int pin_state;

    pin_state = gpio_pin_get(dev, HAL_DIO0_PIN);

    LOG_INF("%s: pin_state 0x%x", __func__, pin_state); 
}

// -----------------------------------------------------------------------------

int hal_init ()
{
    dev_gpio = device_get_binding(HAL_GPIO_NAME);
    if (!dev_gpio) {
        LOG_ERR("device not found. %s", HAL_GPIO_NAME);
        return -1;
    }

    gpio_pin_configure(dev_gpio, HAL_RESET_PIN, GPIO_OUTPUT | HAL_RESET_FLAG);
    //gpio_pin_configure(dev_gpio, HAL_DIO0_PIN,  GPIO_INPUT  | HAL_DIO0_FLAG);

    /* Setup DIO0 interrupts */

    int flags = (GPIO_INPUT      | 
                 GPIO_ACTIVE_LOW |  
                 GPIO_PULL_UP    | 
                 GPIO_INT_EDGE   | 
                 GPIO_INT_EDGE_BOTH);

    gpio_pin_configure(dev_gpio, HAL_DIO0_PIN, flags);

    gpio_pin_interrupt_configure(dev_gpio, HAL_DIO0_PIN, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&dio0_event, hal_dio0_event, BIT(HAL_DIO0_PIN));

    gpio_add_callback(dev_gpio, &dio0_event);

    /* Setup SPI */
    dev_spi = device_get_binding(DT_INST_0_IBM_SX1276_BUS_NAME);
    if (!dev_spi) {
        LOG_ERR("Cannot get pointer to %s device",
                DT_INST_0_IBM_SX1276_BUS_NAME);
        return -1;
    }

    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi_cfg.frequency = DT_INST_0_IBM_SX1276_SPI_MAX_FREQUENCY;
    spi_cfg.slave     = DT_INST_0_IBM_SX1276_BASE_ADDRESS;
    spi_cfg.cs = &spi_cs;

    spi_cs.gpio_dev = dev_gpio;
    spi_cs.gpio_pin = HAL_CS_PIN;

    LOG_INF("hal_init OK");
    return 0;
}

void hal_failed () {

}

#endif // CFG_lmic_clib
